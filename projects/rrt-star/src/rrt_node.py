import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import math
import random
import time
import signal
import os
import csv

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


# v4.0 SPEED-UP ONLY: v3.9 회피 안정성 유지, 속도 관련 값만 소폭 상향
# =====================================================================
# 시작 전 가장 먼저 확인할 플래그
# =====================================================================
# False  : 차량은 정지(speed=0)하고 RRT* 트리만 RViz에 시각화.
# True   : pure pursuit + (Stage 2) 장애물 회피 주행.
ENABLE_DRIVE = True
# =====================================================================


# =====================================================================
# RRT* 고정 goal 디버그 모드
# =====================================================================
# 아래 이미지처럼 "차량을 정지시킨 상태에서 차량 기준 전방 4m를 RRT* goal로 잡고
# RRT* 트리/경로가 어떻게 생성되는지 확인"하기 위한 기능.
#
# 사용법:
#   1) 기능 확인만 할 때:
#        USE_FIXED_FRONT_RRT_GOAL = True
#        FIXED_RRT_GOAL_STOP = True
#        FIXED_RRT_GOAL_ALWAYS_PLAN = True
#   2) 실제 주행으로 돌아갈 때:
#        USE_FIXED_FRONT_RRT_GOAL = False
#
# 주의:
#   - USE_FIXED_FRONT_RRT_GOAL=True이면 waypoint goal을 쓰지 않고,
#     차량 local frame 기준 (x=4.0m, y=0.0m)을 RRT* goal로 사용한다.
#   - FIXED_RRT_GOAL_STOP=True이면 ENABLE_DRIVE=True여도 차량 속도는 0으로 강제된다.
#   - goal이 장애물 inflation 셀 위에 있으면 perform_rrt_star()에서 가까운 free cell로 살짝 이동한다.
USE_FIXED_FRONT_RRT_GOAL = True   # 확인용 기본 ON. 실제 주행할 때는 False로 변경
FIXED_RRT_GOAL_X = 4.0
FIXED_RRT_GOAL_Y = 0.0
FIXED_RRT_GOAL_STOP = True
FIXED_RRT_GOAL_ALWAYS_PLAN = True
# =====================================================================


# Pure pursuit 파라미터
KP = 0.60
# 속도 비례 lookahead: 저속에서 타이트, 고속에서 멀리 봄.
#   12m/s -> 4.5m(클램프),  4m/s -> 1.6m
MIN_LOOKAHEAD = 2.30           # v3.9: 속도감 유지, 회피 반응성 위해 소폭 축소
MAX_LOOKAHEAD = 8.4            # v4.0: 직선/탈출 구간 속도 상향용 소폭 증가
LOOKAHEAD_SPEED_GAIN = 0.70    # v4.0: 고속에서 조금 더 멀리 봄
# RRT* 회피 중: 경로를 타이트하게 추종
RRT_LOOKAHEAD = 1.55
# 조향각 클램프 (f1tenth 물리 한계 ~0.4189)
# 고속일수록 작은 조향이 안전 — PP가 거리 비례라 고속엔 자연히 작아짐
MAX_STEERING_ANGLE = 0.40

WAYPOINTS_FILENAME = 'waypoints.csv'
WAYPOINTS_INTERVAL = 20        # 직선 boost: 곡률/경로 추종 정밀도 증가

# RRT* goal의 차량 기준 목표 거리 (m). grid 전방 한계(6m) 안쪽으로.
RRT_GOAL_TARGET_RADIUS = 4.5

# ---- Stage 2 파라미터 ----
# 막힘 검사 거리: dist = clamp(v * TIME, MIN, MAX)
#   10m/s -> 5.8m(상한),  4m/s -> 3.2m(하한)
BLOCK_CHECK_MIN = 2.4
BLOCK_CHECK_MAX = 5.8          # v3.9: 장애물을 더 일찍 보고 회피 시작
BLOCK_CHECK_TIME = 0.50        # v3.9: 고속에서도 회피 진입을 늦추지 않음

# PP 복귀 조건: N프레임 연속 뚫림 + PP target 직선 LOS 확보
CLEAR_FRAMES_TO_EXIT = 12

# ---- 속도 프로파일 (물리 기반, 7초대 랩 타깃) ----
# 3단 밴드(직선/중간/급) 대신 연속 프로파일:
#   v_curve = sqrt(LAT_ACCEL_MAX / κ)            (코너 한계 속도)
#   v(d)    = sqrt(v_curve² + 2·BRAKE_ACCEL·d)   (제동 거리 역산)
# 전방 SPEED_HORIZON 안의 모든 waypoint 곡률에 대해 min을 취해
# "코너 진입 전에 미리 감속, 탈출하면서 즉시 재가속"이 자동으로 된다.
V_MAX = 29.0                   # v4.0: 최고속 소폭 상향
V_MIN = 6.8                    # v4.0: 전체 평균속도 상향
LAT_ACCEL_MAX = 14.4           # v4.0: 코너/완만한 구간 속도 소폭 상향
BRAKE_ACCEL = 14.5             # v4.0: 재가속/늦은 감속 허용
SPEED_HORIZON = 6.0            # v4.0: 감속 시작을 아주 조금 늦춤
SPEED_RRT_CAP = 5.8            # v4.0: 회피 중 기본 속도 소폭 상향

# ---- 라이다 전방 제동 캡 (grid 6m 너머 장애물/벽 대비) ----
# V_MAX 제동거리(~12m)가 occupancy grid 전방 한계(6m)보다 길어서,
# grid가 보기 전의 장애물은 라이다 raw range로 직접 캡: 전방 ±콘 최소
# 거리 d에 대해 v ≤ sqrt(2·BRAKE·(d − margin)).
FWD_CONE_HALF_ANGLE = 0.070    # v3.9: 장애물 감지 폭 복구
FWD_SAFETY_MARGIN = 0.42       # v3.9: 장애물 주변 여유 확보
LASER_CAP_DISTANCE = 2.75       # v3.9: 장애물 근처 감속을 조금 더 일찍 적용
STRAIGHT_LASER_CAP_DISTANCE = 1.35  # v4.0: 직선에서 불필요한 감속 소폭 완화
RRT_FAST_CAP = 8.2              # v4.0: 회피 후 복귀/빈 공간에서 더 빠르게
BLOCKED_FRAMES_TO_ENTER = 1     # v3.9: 장애물 감지 즉시 회피 진입
STRAIGHT_CURVATURE_TH = 0.085  # v3.9: 완만한 곡선 boost는 유지하되 회피 오판 완화
STRAIGHT_MIN_SPEED = 20.4      # v4.0: 직선 속도만 소폭 강화

# (OPT 2) 시각화 throttle (odom/scan 주기 대비 1/N로 publish)
GRID_PUBLISH_EVERY = 8
PATH_VIZ_EVERY = 8
WAYPOINT_VIZ_EVERY = 80


PACKAGE_NAME = 'lab6'
try:
    waypoint_dir = get_package_share_directory(PACKAGE_NAME)
    waypoint_filepath = os.path.join(waypoint_dir, WAYPOINTS_FILENAME)
except (PackageNotFoundError, Exception):
    waypoint_filepath = os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     '..', WAYPOINTS_FILENAME))


class RRTStarNode:
    """RRT* 트리의 한 노드. (x, y)는 local(차량 base_link) frame 기준."""

    def __init__(self, x, y, parent=None, cost=0.0):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = cost


class RRTStar(Node):
    def __init__(self):
        super().__init__('rrt_star_node')

        # ---- Occupancy grid 파라미터 ----
        self.grid_length_x = 7
        self.grid_length_y = 7
        self.grid_resolution = 0.05
        self.grid_width = int(self.grid_length_x / self.grid_resolution)
        self.grid_height = int(self.grid_length_y / self.grid_resolution)
        self.x_offset = 1.0
        self.y_offset = self.grid_length_y / 2
        self.occupancy_thickness = 4        # v3.6: 장애물 회피 여유 확보

        # (OPT 1) inflation용 disk offset을 numpy로 미리 계산
        t = self.occupancy_thickness
        offs = [(dx, dy)
                for dx in range(-t, t + 1)
                for dy in range(-t, t + 1)
                if dx * dx + dy * dy <= t * t]
        self._disk_dx = np.array([o[0] for o in offs], dtype=np.int32)
        self._disk_dy = np.array([o[1] for o in offs], dtype=np.int32)

        # ---- RRT* 파라미터 ----
        self.rrt_goal_x = 0.0
        self.rrt_goal_y = 0.0
        self.pp_target_x = 0.0
        self.pp_target_y = 0.0
        self.max_iterations = 450
        self.step_size = 0.40
        self.neighborhood_radius = 1.8
        self.goal_threshold = self.step_size

        # ---- per-cycle planner 상태 ----
        self.trajectory_clear = True
        self.rrt_path_found = False
        self.final_node = None

        # ---- Stage 2 상태 ----
        self.rrt_mode = False               # 현재 RRT* 회피 모드 여부
        self.clear_count = 0                # 연속으로 '뚫림' 판정된 프레임 수
        self.blocked_count = 0              # 연속 막힘 프레임 수: 라이다 노이즈/RRT false entry 방지
        self.raw_blocked = False              # v3.6: RRT 진입 전 장애물 감지 상태를 속도 제어에도 사용
        self.last_target = None             # fallback용 직전 steering target (local)
        self.prev_path_global = None        # warm-start + 추종용 path (map frame)

        # ---- 차량 자세 ----
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_speed = 0.0            # (OPT 5) odom twist에서 갱신
        self.forward_clearance = None       # 라이다 전방 콘 최소 거리 (m)
        self.prev_steering = 0.0            # 조향 low-pass 필터 상태
        self._closest_idx = 0               # 최근접 waypoint (프레임당 1회 계산)
        self.grid_pose_x = 0.0
        self.grid_pose_y = 0.0
        self.grid_pose_yaw = 0.0

        # ---- 시각화 throttle 카운터 ----
        self._scan_count = 0
        self._plan_count = 0
        self._frame_count = 0

        # ---- frame_id 설정 ----
        self.rrt_tree_markers_frame_id = 'ego_racecar/base_link'
        self.occupancy_grid_markers_frame_id = 'ego_racecar/laser'

        # ---- ROS publisher / subscriber ----
        self.path_marker_pub = self.create_publisher(MarkerArray, '/path', 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.target_marker_pub = self.create_publisher(Marker, '/target_marker', 10)
        self.waypoints_marker_pub = self.create_publisher(Marker, '/waypoints_marker', 10)
        # /goal_marker : RViz Marker(SPHERE)로 goal 위치 표시
        self.goal_marker_pub = self.create_publisher(Marker, '/goal_marker', 10)
        # /goal_pose : RViz Pose display에서 초록 화살표로 goal 방향/위치 표시
        # 사용자가 보여준 영상처럼 Topic 목록에 /goal_pose가 뜨게 하기 위한 publisher.
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)
        self.rrt_tree_marker_pub = self.create_publisher(MarkerArray, '/rrt_star_tree', 10)
        self.odom_sub = self.create_subscription(Odometry, 'ego_racecar/odom', self.pose_callback, 10)

        # ---- 초기 occupancy grid (-1 = unknown) ----
        self.occupancy_grid = np.ones((self.grid_width, self.grid_height), dtype=np.int8) * -1
        self.nodes = [RRTStarNode(0, 0)]

        self.marker_lifetime = rclpy.duration.Duration(seconds=10.1).to_msg()

        # ---- waypoints 로드 ----
        self.waypoints_x = []
        self.waypoints_y = []
        self.load_waypoints(waypoint_filepath, WAYPOINTS_INTERVAL)

        if USE_FIXED_FRONT_RRT_GOAL:
            self.get_logger().warn(
                f"FIXED RRT GOAL DEBUG MODE ON: goal=({FIXED_RRT_GOAL_X:.1f}, {FIXED_RRT_GOAL_Y:.1f}) m in vehicle local frame, "
                f"stop={FIXED_RRT_GOAL_STOP}"
            )
            # RViz Add > By topic은 publisher가 살아 있어도 메시지가 아직 안 나오면 안 보이는 경우가 있다.
            # 그래서 디버그 모드에서는 /goal_marker와 /goal_pose를 10Hz로 계속 publish한다.
            self.fixed_goal_debug_timer = self.create_timer(0.1, self._fixed_goal_debug_timer)

    # =================================================================
    # Waypoint 로드 & 좌표 변환 (제공)
    # =================================================================

    def load_waypoints(self, filepath, interval=100):
        try:
            with open(filepath, 'r') as f:
                reader = csv.reader(f)
                for i, row in enumerate(reader):
                    if i % interval == 0:
                        x, y, _, _ = map(float, row)
                        self.waypoints_x.append(x)
                        self.waypoints_y.append(y)
            self.get_logger().info(
                f"Loaded {len(self.waypoints_x)} waypoints from {filepath}")
        except (FileNotFoundError, OSError) as e:
            self.get_logger().error(
                f"Failed to load waypoints from {filepath}: {e}")
        self._precompute_curvature()

    def _precompute_curvature(self):
        """waypoint마다 곡률 κ(연속 3점 외접원 반지름의 역수)와 다음 점까지의
        거리를 미리 계산. 속도 프로파일(_path_speed_limit)에서 사용."""
        n = len(self.waypoints_x)
        self.waypoints_kappa = [0.0] * n
        self.waypoints_seg = [1.0] * n
        if n < 3:
            return
        raw = [0.0] * n
        for i in range(n):
            x1, y1 = self.waypoints_x[i - 1], self.waypoints_y[i - 1]
            x2, y2 = self.waypoints_x[i], self.waypoints_y[i]
            x3, y3 = self.waypoints_x[(i + 1) % n], self.waypoints_y[(i + 1) % n]
            a = math.hypot(x2 - x1, y2 - y1)
            b = math.hypot(x3 - x2, y3 - y2)
            c = math.hypot(x3 - x1, y3 - y1)
            area2 = abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1))
            raw[i] = 2.0 * area2 / (a * b * c) if a * b * c > 1e-9 else 0.0
            self.waypoints_seg[i] = max(b, 1e-3)
        # 이웃 max 스무딩: 샘플링 위상 때문에 코너 곡률이 과소평가되어
        # 과속 진입하는 것을 방지 (보수적 방향으로만 스무딩)
        self.waypoints_kappa = [
            max(raw[i - 1], raw[i], raw[(i + 1) % n]) for i in range(n)]

    def _closest_waypoint_idx(self):
        min_d2 = float('inf')
        best = 0
        for i in range(len(self.waypoints_x)):
            dx = self.waypoints_x[i] - self.current_x
            dy = self.waypoints_y[i] - self.current_y
            d2 = dx * dx + dy * dy
            if d2 < min_d2:
                min_d2 = d2
                best = i
        return best

    def _path_speed_limit(self):
        """(속도 프로파일) 전방 SPEED_HORIZON 호 길이 안 waypoint들의
        곡률 한계 속도에 제동 거리를 역산해 현재 허용 속도를 계산.

        v_allow = min_i sqrt( min(V_MAX, sqrt(LAT/κ_i))² + 2·BRAKE·d_i )
        -> 코너 진입 전 정확히 필요한 만큼만 미리 감속하고,
           코너 정점을 지나면 horizon에서 코너가 빠지며 자동 재가속."""
        n = len(self.waypoints_x)
        if n < 3 or not self.waypoints_kappa:
            return V_MAX
        v_allow = V_MAX
        d = 0.0
        idx = self._closest_idx
        for _ in range(n):
            k = self.waypoints_kappa[idx]
            if k > 1e-3:
                v_curve = min(V_MAX, math.sqrt(LAT_ACCEL_MAX / k))
            else:
                v_curve = V_MAX
            v_here = math.sqrt(v_curve * v_curve + 2.0 * BRAKE_ACCEL * d)
            if v_here < v_allow:
                v_allow = v_here
            d += self.waypoints_seg[idx]
            if d >= SPEED_HORIZON:
                break
            idx = (idx + 1) % n
        return max(V_MIN, v_allow)

    def local_to_global(self, x_local, y_local):
        x_global = self.current_x + (x_local * math.cos(self.current_yaw)
                                     - y_local * math.sin(self.current_yaw))
        y_global = self.current_y + (x_local * math.sin(self.current_yaw)
                                     + y_local * math.cos(self.current_yaw))
        return x_global, y_global

    def global_to_local(self, x_global, y_global):
        dx = x_global - self.current_x
        dy = y_global - self.current_y
        x_local = dx * math.cos(-self.current_yaw) - dy * math.sin(-self.current_yaw)
        y_local = dx * math.sin(-self.current_yaw) + dy * math.cos(-self.current_yaw)
        return x_local, y_local

    def convert_to_grid(self, x_local, y_local):
        cos_p = math.cos(self.current_yaw)
        sin_p = math.sin(self.current_yaw)
        x_map = cos_p * x_local - sin_p * y_local + self.current_x
        y_map = sin_p * x_local + cos_p * y_local + self.current_y
        dx = x_map - self.grid_pose_x
        dy = y_map - self.grid_pose_y
        cos_s = math.cos(self.grid_pose_yaw)
        sin_s = math.sin(self.grid_pose_yaw)
        x_grid_local = cos_s * dx + sin_s * dy
        y_grid_local = -sin_s * dx + cos_s * dy
        x_grid = int(round((x_grid_local + self.x_offset) / self.grid_resolution))
        y_grid = int(round((y_grid_local + self.y_offset) / self.grid_resolution))
        return x_grid, y_grid

    def _grid_to_local(self, gx, gy):
        """grid index -> 현재 차량 local 좌표 (convert_to_grid의 역변환)."""
        x_g = gx * self.grid_resolution - self.x_offset
        y_g = gy * self.grid_resolution - self.y_offset
        cos_s = math.cos(self.grid_pose_yaw)
        sin_s = math.sin(self.grid_pose_yaw)
        x_map = self.grid_pose_x + cos_s * x_g - sin_s * y_g
        y_map = self.grid_pose_y + sin_s * x_g + cos_s * y_g
        return self.global_to_local(x_map, y_map)

    # =================================================================
    # Pure pursuit baseline
    # =================================================================

    def _lookahead_dist(self):
        """(OPT 5) 속도 비례 lookahead."""
        return min(MAX_LOOKAHEAD,
                   max(MIN_LOOKAHEAD, LOOKAHEAD_SPEED_GAIN * self.current_speed))

    def _lookahead_on_polyline(self, points_local, lookahead):
        L2 = lookahead * lookahead
        for i in range(len(points_local) - 1):
            ax, ay = points_local[i]
            bx, by = points_local[i + 1]
            dx = bx - ax
            dy = by - ay
            a = dx * dx + dy * dy
            if a < 1e-12:
                continue
            b = 2.0 * (ax * dx + ay * dy)
            c = ax * ax + ay * ay - L2
            disc = b * b - 4.0 * a * c
            if disc < 0.0:
                continue
            sqrt_disc = math.sqrt(disc)
            t1 = (-b - sqrt_disc) / (2.0 * a)
            t2 = (-b + sqrt_disc) / (2.0 * a)
            for t in (t1, t2):
                if 0.0 <= t <= 1.0:
                    return ax + t * dx, ay + t * dy
        return None

    def _find_pp_target_global(self):
        if not self.waypoints_x:
            return self.current_x, self.current_y

        lookahead = self._lookahead_dist()
        n = len(self.waypoints_x)
        closest_idx = self._closest_idx  # pose_callback에서 프레임당 1회 계산

        polyline_local = []
        idx = closest_idx
        steps = 0
        while True:
            wx, wy = self.waypoints_x[idx], self.waypoints_y[idx]
            lx, ly = self.global_to_local(wx, wy)
            polyline_local.append((lx, ly))
            if (math.hypot(lx, ly) >= lookahead
                    and len(polyline_local) >= 2):
                break
            idx = (idx + 1) % n
            steps += 1
            if steps >= n:
                break

        intersect_local = self._lookahead_on_polyline(polyline_local, lookahead)
        if intersect_local is None:
            return (self.waypoints_x[closest_idx],
                    self.waypoints_y[closest_idx])
        return self.local_to_global(*intersect_local)

    def _find_rrt_goal_global(self):
        """(OPT 7) RRT* goal 선택 강화.

        - 전방(lx>0) waypoint 중 grid 내부(여유 0.3m)이고 free 셀인 후보만 사용.
        - 후보 중 RRT_GOAL_TARGET_RADIUS에 가장 가까운 것을 선택.
        - free 후보가 없으면: grid 내부 후보 중 radius에 가장 가까운 것
          (perform_rrt_star에서 인근 free 셀로 자동 이동시킴).
        - grid 내부 후보조차 없으면: 거리 기준 fallback.
        코너 직후처럼 goal이 grid 밖/벽 위로 떨어져 플래너가 통째로
        포기하던 케이스를 제거한다.
        """
        if not self.waypoints_x:
            return self.current_x, self.current_y

        x_max = self.grid_length_x - self.x_offset - 0.3
        y_max = self.y_offset - 0.3

        best_free = None
        best_free_diff = float('inf')
        best_ingrid = None
        best_ingrid_diff = float('inf')
        fallback = None
        fb_diff = float('inf')

        for i in range(len(self.waypoints_x)):
            wp_x, wp_y = self.waypoints_x[i], self.waypoints_y[i]
            lx, ly = self.global_to_local(wp_x, wp_y)
            if lx <= 0:
                continue  # 후방 waypoint 무시
            dist = math.hypot(lx, ly)
            diff = abs(dist - RRT_GOAL_TARGET_RADIUS)

            if diff < fb_diff:
                fb_diff = diff
                fallback = (wp_x, wp_y)

            # grid 내부 여부
            if not (0.3 <= lx <= x_max and abs(ly) <= y_max):
                continue
            if diff < best_ingrid_diff:
                best_ingrid_diff = diff
                best_ingrid = (wp_x, wp_y)

            # free 셀 여부
            gx, gy = self.convert_to_grid(lx, ly)
            if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                if self.occupancy_grid[gx, gy] > 0:
                    continue
            if diff < best_free_diff:
                best_free_diff = diff
                best_free = (wp_x, wp_y)

        if best_free is not None:
            return best_free
        if best_ingrid is not None:
            return best_ingrid
        if fallback is not None:
            return fallback
        return self.current_x, self.current_y

    def _compute_steering(self, steering_target_x, steering_target_y, smooth=True):
        d2 = steering_target_x ** 2 + steering_target_y ** 2
        if d2 < 1e-6:
            return 0.0
        angle = KP * 2.0 * steering_target_y / d2
        angle = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, angle))
        # 고속 안정용 low-pass. 회피(RRT) 중엔 즉각 반응이 중요해 끔.
        if smooth:
            angle = 0.90 * angle + 0.10 * self.prev_steering
        self.prev_steering = angle
        return angle

    def _publish_drive(self, angle, target_x, target_y, rrt_mode=False):
        """주행 명령 publish. ENABLE_DRIVE=False면 정지.

        속도 = min( 1) steering target 즉시 곡률의 횡가속 한계,
                    2) 전방 경로 곡률 + 제동거리 프로파일,
                    3) 라이다 전방 콘 제동 캡 )
        밴드 없이 연속이라 코너 탈출 즉시 재가속 → 랩타임 단축.
        """
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.acceleration = 15.0  # v4.0: 재가속 반응 소폭 강화(드라이버가 지원하면 적용)

        # 고정 goal 디버그 모드에서 차량 정지 강제.
        # ENABLE_DRIVE=True여도 FIXED_RRT_GOAL_STOP=True이면 speed=0만 publish한다.
        # 즉, RViz에서 차량은 멈춘 상태로 전방 4m RRT* 트리/경로만 확인 가능.
        if USE_FIXED_FRONT_RRT_GOAL and FIXED_RRT_GOAL_STOP:
            drive_msg.drive.speed = 0.0
            self.drive_pub.publish(drive_msg)
            return

        if not ENABLE_DRIVE:
            drive_msg.drive.speed = 0.0
            self.drive_pub.publish(drive_msg)
            return

        # 1) 현재 steering target의 기하 곡률 |2y/d²| → 횡가속 한계
        d2 = target_x ** 2 + target_y ** 2
        curvature = abs(2.0 * target_y / d2) if d2 > 1e-6 else 0.0
        v_now = math.sqrt(LAT_ACCEL_MAX / curvature) if curvature > 1e-3 else V_MAX

        # 2) 전방 waypoint 곡률 + 제동 프로파일
        v_path = self._path_speed_limit()

        # 곡률 기반 한계는 V_MIN으로 하한 (가장 급한 코너에서도 전진 유지)
        speed = max(V_MIN, min(v_now, v_path, V_MAX))

        # 직선 구간 boost:
        # PP target 곡률이 작고, waypoint 기반 속도 프로파일도 충분히 빠르면
        # 라이다가 직선 끝의 벽/코너를 조금 본다는 이유로 속도가 죽지 않게 한다.
        straight_like = (curvature < STRAIGHT_CURVATURE_TH
                         and v_path > STRAIGHT_MIN_SPEED)
        # v3.6: 장애물이 아직 raw_blocked로 감지된 상태면 직선 boost를 강제로 걸지 않음.
        # 이렇게 해야 속도감은 유지하면서도 장애물 직전까지 밀고 들어가는 현상을 줄일 수 있다.
        # v3.8: raw_blocked가 떠도 정면 clearance가 충분히 멀면 직선 boost 유지.
        # 장애물 감지 자체보다 실제 정면 거리로 감속 여부를 판단해서 전체 속도를 올린다.
        front_far = (self.forward_clearance is None
                     or self.forward_clearance > LASER_CAP_DISTANCE + 1.00)
        obstacle_near = (self.forward_clearance is not None
                         and self.forward_clearance < LASER_CAP_DISTANCE + 0.35)

        # v3.9: 직선 속도감은 유지하되, raw_blocked가 켜지고 정면 여유가 충분하지 않으면
        # 직선 boost를 끊어서 장애물 직전 과속 진입을 방지한다.
        if not rrt_mode and straight_like and (not self.raw_blocked or front_far):
            speed = max(speed, STRAIGHT_MIN_SPEED)
        elif (not rrt_mode
              and curvature < STRAIGHT_CURVATURE_TH * 0.65
              and front_far
              and not obstacle_near):
            speed = max(speed, STRAIGHT_MIN_SPEED * 0.94)

        # 라이다 전방 제동 cap:
        # 직선에서는 너무 쉽게 감속하지 않지만, raw_blocked + 근접 clearance이면 즉시 cap을 건다.
        # 이렇게 해야 평소 직선 속도는 유지하면서 장애물 회피 진입 때만 확실히 속도를 낮출 수 있다.
        if self.forward_clearance is not None:
            if (not rrt_mode) and straight_like and (not self.raw_blocked or front_far):
                use_laser_cap = self.forward_clearance < STRAIGHT_LASER_CAP_DISTANCE
            else:
                use_laser_cap = (
                    self.forward_clearance < LASER_CAP_DISTANCE
                    or curvature >= STRAIGHT_CURVATURE_TH * 0.95
                    or rrt_mode
                    or (self.raw_blocked and self.forward_clearance < LASER_CAP_DISTANCE + 0.65)
                )

            if use_laser_cap:
                gap = self.forward_clearance - FWD_SAFETY_MARGIN
                if gap > 0.0:
                    v_laser = max(2.8, math.sqrt(2.0 * BRAKE_ACCEL * gap))
                else:
                    v_laser = 1.0
                speed = min(speed, v_laser)

        # RRT 모드에서는 장애물 주변 속도를 낮추고, 완전히 빠져나오는 직선 복귀에서만 빠른 cap 허용.
        if rrt_mode:
            rrt_cap = SPEED_RRT_CAP
            if ((self.forward_clearance is None or self.forward_clearance > LASER_CAP_DISTANCE + 0.75)
                    and curvature < STRAIGHT_CURVATURE_TH * 0.80
                    and (not self.raw_blocked or self.forward_clearance is None or self.forward_clearance > LASER_CAP_DISTANCE + 1.05)):
                rrt_cap = RRT_FAST_CAP
            speed = min(speed, rrt_cap)

        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

    # =================================================================
    # Marker 시각화
    # =================================================================

    def _fixed_goal_debug_timer(self):
        """고정 전방 4m goal 디버그 모드에서 RViz 토픽이 항상 보이도록 계속 publish."""
        if not USE_FIXED_FRONT_RRT_GOAL:
            return
        self.rrt_goal_x = FIXED_RRT_GOAL_X
        self.rrt_goal_y = FIXED_RRT_GOAL_Y
        self._publish_goal_marker()

    def _publish_goal_marker(self):
        gx, gy = self.local_to_global(self.rrt_goal_x, self.rrt_goal_y)
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.30
        marker.scale.y = 0.30
        marker.scale.z = 0.30
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.pose.position.x = gx
        marker.pose.position.y = gy
        marker.pose.position.z = 0.0
        marker.lifetime = self.marker_lifetime
        self.goal_marker_pub.publish(marker)

        # /goal_pose도 같이 publish해야 RViz Add > By topic에 /goal_pose가 보임.
        # 중요: RViz Pose display는 geometry_msgs/Pose보다 PoseStamped가 확실하게 잡힌다.
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = gx
        goal_pose.pose.position.y = gy
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.z = math.sin(self.current_yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(self.current_yaw / 2.0)
        self.goal_pose_pub.publish(goal_pose)

    def _publish_waypoints_marker(self):
        if not self.waypoints_x:
            return
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.a = 1.0
        marker.color.b = 1.0
        marker.points = [Point(x=x, y=y, z=0.0)
                         for x, y in zip(self.waypoints_x, self.waypoints_y)]
        self.waypoints_marker_pub.publish(marker)

    def _publish_target_marker(self, steering_target_x, steering_target_y):
        gx, gy = self.local_to_global(steering_target_x, steering_target_y)
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.position.x = gx
        marker.pose.position.y = gy
        marker.pose.position.z = 0.0
        marker.lifetime = self.marker_lifetime
        self.target_marker_pub.publish(marker)

    def add_marker(self, marker_array, node, idx, frame_id, is_rrt_goal=False):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.lifetime = self.marker_lifetime
        if is_rrt_goal is True:
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        else:
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        marker.pose.position.x = node.x
        marker.pose.position.y = node.y
        marker.pose.position.z = 0.0
        marker.id = idx
        marker_array.markers.append(marker)

    def add_edge_marker(self, marker_array, start_node, end_node, idx, frame_id,
                        is_final=False):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.lifetime = self.marker_lifetime

        marker.color.a = 1.0
        if is_final is True:
            marker.scale.x = 0.1
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.scale.x = 0.05
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        start_point = Point(x=start_node.x, y=start_node.y, z=0.0)
        end_point = Point(x=end_node.x, y=end_node.y, z=0.0)
        marker.points = [start_point, end_point]
        marker.id = idx + self.max_iterations
        marker_array.markers.append(marker)

    def _publish_tree_markers(self):
        """트리를 map frame 좌표로 변환 후 마커 2개(POINTS + LINE_LIST)로 publish.
        - local -> global 변환으로 TF 없이도 RViz map frame에서 바로 표시됨.
        - 마커를 노드당 2개 대신 전체 2개로 줄여 RViz가 죽지 않음."""
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # ---- 노드 점 (POINTS, 초록) ----
        node_marker = Marker()
        node_marker.header.frame_id = 'map'
        node_marker.header.stamp = stamp
        node_marker.id = 0
        node_marker.type = Marker.POINTS
        node_marker.action = Marker.ADD
        node_marker.lifetime = self.marker_lifetime
        node_marker.scale.x = 0.08
        node_marker.scale.y = 0.08
        node_marker.color.a = 1.0
        node_marker.color.r = 0.0
        node_marker.color.g = 1.0
        node_marker.color.b = 0.0

        # ---- 엣지 선 (LINE_LIST, 빨강) ----
        edge_marker = Marker()
        edge_marker.header.frame_id = 'map'
        edge_marker.header.stamp = stamp
        edge_marker.id = 1
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.lifetime = self.marker_lifetime
        edge_marker.scale.x = 0.03
        edge_marker.color.a = 1.0
        edge_marker.color.r = 1.0
        edge_marker.color.g = 0.0
        edge_marker.color.b = 0.0

        for node in self.nodes:
            gx, gy = self.local_to_global(node.x, node.y)
            node_marker.points.append(Point(x=gx, y=gy, z=0.0))
            if node.parent is not None:
                pgx, pgy = self.local_to_global(node.parent.x, node.parent.y)
                edge_marker.points.append(Point(x=pgx, y=pgy, z=0.0))
                edge_marker.points.append(Point(x=gx, y=gy, z=0.0))

        marker_array.markers = [node_marker, edge_marker]
        self.rrt_tree_marker_pub.publish(marker_array)

    def _publish_rrt_path_markers(self):
        """(OPT 4) prev_path_global(shortcut된 최종 path)을 그대로 표시."""
        path_marker_array = MarkerArray()

        delete_all = Marker()
        delete_all.header.frame_id = 'map'
        delete_all.action = Marker.DELETEALL
        delete_all.id = -1
        path_marker_array.markers.append(delete_all)

        if self.prev_path_global:
            prev_node = None
            for marker_id, (gx, gy) in enumerate(self.prev_path_global):
                node = RRTStarNode(gx, gy)
                is_goal = (marker_id == len(self.prev_path_global) - 1)
                self.add_marker(path_marker_array, node, marker_id, 'map',
                                is_rrt_goal=is_goal)
                if prev_node is not None:
                    self.add_edge_marker(path_marker_array, prev_node, node,
                                         marker_id, 'map', is_final=True)
                prev_node = node

        self.path_marker_pub.publish(path_marker_array)

    # =================================================================
    # Occupancy grid
    # =================================================================

    def scan_callback(self, scan_msg):
        """(OPT 1) NumPy 벡터화. 기존: 빔당 11x11 파이썬 루프(~13만 회/스캔)
        -> 전체를 배열 연산 몇 번으로 처리. 제어 루프 lag의 최대 원인 제거."""
        self.grid_pose_x = self.current_x
        self.grid_pose_y = self.current_y
        self.grid_pose_yaw = self.current_yaw

        grid = np.zeros((self.grid_width, self.grid_height), dtype=np.int8)

        ranges = np.asarray(scan_msg.ranges, dtype=np.float32)
        n = ranges.shape[0]
        angles = scan_msg.angle_min + np.arange(n, dtype=np.float32) * scan_msg.angle_increment

        # 빔 1/2 다운샘플 (inflation 25cm >> 인접 빔 간격이라 안전)
        ranges = ranges[::2]
        angles = angles[::2]

        # ---- 라이다 전방 콘 최소 거리 (원거리 제동 캡용) ----
        # grid(전방 6m) 너머의 장애물/벽까지의 거리. 9m 필터 적용 전 원본 사용.
        finite = np.isfinite(ranges) & (ranges > scan_msg.range_min)
        cone = finite & (np.abs(angles) < FWD_CONE_HALF_ANGLE)
        if np.any(cone):
            # v3.5: min은 라이다 한두 빔 노이즈/벽 모서리 때문에 직선에서도 과감속을 유발함.
            # 8th percentile을 사용해 노이즈는 줄이되, 가까운 장애물은 더 민감하게 잡는다.
            self.forward_clearance = float(np.percentile(ranges[cone], 8))
        else:
            self.forward_clearance = None

        # grid 대각(~9m)보다 먼 빔은 어차피 grid 밖
        valid = (np.isfinite(ranges)
                 & (ranges > scan_msg.range_min)
                 & (ranges < scan_msg.range_max)
                 & (ranges < 9.0))
        r = ranges[valid]
        a = angles[valid]

        if r.shape[0] > 0:
            x_local = r * np.cos(a)
            y_local = r * np.sin(a)
            gx = np.round((x_local + self.x_offset) / self.grid_resolution).astype(np.int32)
            gy = np.round((y_local + self.y_offset) / self.grid_resolution).astype(np.int32)

            t = self.occupancy_thickness
            keep = ((gx >= -t) & (gx < self.grid_width + t)
                    & (gy >= -t) & (gy < self.grid_height + t))
            gx = gx[keep]
            gy = gy[keep]

            if gx.shape[0] > 0:
                ix = (gx[:, None] + self._disk_dx[None, :]).ravel()
                iy = (gy[:, None] + self._disk_dy[None, :]).ravel()
                m = ((ix >= 0) & (ix < self.grid_width)
                     & (iy >= 0) & (iy < self.grid_height))
                grid[ix[m], iy[m]] = 100

        self.occupancy_grid = grid

        # (OPT 2) grid publish throttle (시각화 전용이라 5Hz 정도면 충분)
        self._scan_count += 1
        if self._scan_count % GRID_PUBLISH_EVERY == 0:
            self.publish_occupancy_grid(scan_msg.header.stamp)

    def publish_occupancy_grid(self, stamp):
        occupancy_msg = OccupancyGrid()
        occupancy_msg.header.stamp = stamp
        occupancy_msg.header.frame_id = 'map'

        occupancy_msg.info.resolution = self.grid_resolution
        occupancy_msg.info.width = self.grid_width
        occupancy_msg.info.height = self.grid_height

        cos_y = math.cos(self.grid_pose_yaw)
        sin_y = math.sin(self.grid_pose_yaw)
        occupancy_msg.info.origin.position.x = (
            self.grid_pose_x
            + cos_y * (-self.x_offset)
            - sin_y * (-self.y_offset))
        occupancy_msg.info.origin.position.y = (
            self.grid_pose_y
            + sin_y * (-self.x_offset)
            + cos_y * (-self.y_offset))
        occupancy_msg.info.origin.position.z = 0.0
        occupancy_msg.info.origin.orientation.z = math.sin(self.grid_pose_yaw / 2)
        occupancy_msg.info.origin.orientation.w = math.cos(self.grid_pose_yaw / 2)

        occupancy_msg.data = np.transpose(self.occupancy_grid).flatten().tolist()
        self.grid_pub.publish(occupancy_msg)

    # =================================================================
    # Pose / RRT* / follow_path 루프
    # =================================================================

    def pose_callback(self, odom_msg):
        self.current_x = odom_msg.pose.pose.position.x
        self.current_y = odom_msg.pose.pose.position.y
        q = odom_msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        # (OPT 5) 현재 속도 (lookahead / 막힘 검사 거리 산정용)
        self.current_speed = abs(odom_msg.twist.twist.linear.x)

        self._frame_count += 1

        # 최근접 waypoint는 프레임당 1회만 계산해 공유 (PP/막힘검사/속도프로파일)
        if self.waypoints_x:
            self._closest_idx = self._closest_waypoint_idx()

        # ---- Pure pursuit target ----
        pp_xg, pp_yg = self._find_pp_target_global()
        self.pp_target_x, self.pp_target_y = self.global_to_local(pp_xg, pp_yg)

        # ---- (a) 막힘 판정 + (b) hysteresis ----
        # raw_blocked가 1프레임만 튀어도 바로 RRT로 들어가면 직선에서 속도가 죽는다.
        # 그래서 연속 BLOCKED_FRAMES_TO_ENTER 프레임 이상 막힐 때만 RRT 진입.
        raw_blocked = self._pp_path_blocked()
        self.raw_blocked = raw_blocked
        if raw_blocked:
            self.blocked_count += 1
        else:
            self.blocked_count = 0
        blocked = self.blocked_count >= BLOCKED_FRAMES_TO_ENTER
        self.trajectory_clear = not blocked

        if blocked:
            self.rrt_mode = True
            self.clear_count = 0
        elif self.rrt_mode:
            # (OPT 8) PP target까지 직선 LOS가 확보된 프레임만 카운트.
            # 장애물 옆을 스치는 중에 성급히 PP로 복귀해 장애물 쪽으로
            # 꺾어버리는 케이스 방지.
            los_free = not self._segment_blocked_xy(
                0.0, 0.0, self.pp_target_x, self.pp_target_y)
            if los_free:
                self.clear_count += 1
            else:
                self.clear_count = 0
            if self.clear_count >= CLEAR_FRAMES_TO_EXIT:
                self.rrt_mode = False
                self.clear_count = 0
                self.prev_path_global = None
                self.last_target = None

        # ---- (OPT 2) RRT*는 필요할 때만 실행 ----
        # 기존엔 매 odom 프레임마다 무조건 500 iteration + 마커 500개 publish.
        # 주행 중에는 회피 모드일 때만 planning. (ENABLE_DRIVE=False면
        # 시각화 목적이므로 기존처럼 항상 planning.)
        # ---- RRT* planning 실행 조건 ----
        # 기본 주행: 장애물 회피 모드(rrt_mode)일 때만 RRT* 실행.
        # 디버그 모드: USE_FIXED_FRONT_RRT_GOAL=True이고 FIXED_RRT_GOAL_ALWAYS_PLAN=True이면
        #              차량이 정지해 있어도 매 odom 프레임마다 전방 4m goal로 RRT* 실행.
        need_rrt_plan = (
            self.rrt_mode
            or not ENABLE_DRIVE
            or (USE_FIXED_FRONT_RRT_GOAL and FIXED_RRT_GOAL_ALWAYS_PLAN)
        )

        if need_rrt_plan:
            self.nodes = [RRTStarNode(0.0, 0.0)]

            if USE_FIXED_FRONT_RRT_GOAL:
                # 디버그 모드: 차량 local frame 기준 전방 4m를 RRT* goal로 고정.
                # RViz에서 차량이 멈춘 상태로 RRT* 트리/경로가 생성되는지 확인할 때 사용.
                self.rrt_goal_x = FIXED_RRT_GOAL_X
                self.rrt_goal_y = FIXED_RRT_GOAL_Y
            else:
                # 일반 주행 모드: waypoint 중 차량 앞 RRT_GOAL_TARGET_RADIUS 근처를 goal로 선택.
                rrt_goal_xg, rrt_goal_yg = self._find_rrt_goal_global()
                self.rrt_goal_x, self.rrt_goal_y = self.global_to_local(
                    rrt_goal_xg, rrt_goal_yg)

            self.perform_rrt_star()
            self._plan_count += 1

            # 디버그 모드에서는 path marker를 매번 갱신해서 바로 확인 가능하게 함.
            if USE_FIXED_FRONT_RRT_GOAL or self._plan_count % PATH_VIZ_EVERY == 0:
                self._publish_rrt_path_markers()
        else:
            self.final_node = None
            self.rrt_path_found = False

        # ---- 주행 명령 publish ----
        self.follow_path()

    # ---- Stage 2 helper: grid 점유 검사 ----

    def _cell_occupied(self, x_local, y_local):
        """local (x, y)가 grid 안의 점유 셀이면 True.
        grid 밖이면 unknown으로 보고 False (모드 판정용)."""
        gx, gy = self.convert_to_grid(x_local, y_local)
        if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
            return self.occupancy_grid[gx, gy] > 0
        return False

    def _segment_blocked_xy(self, x1, y1, x2, y2):
        """두 점을 잇는 선분 위를 grid_resolution 간격으로 샘플링해 점유 검사."""
        dist = math.hypot(x2 - x1, y2 - y1)
        steps = max(2, int(math.ceil(dist / self.grid_resolution)))
        for i in range(steps + 1):
            t = i / steps
            if self._cell_occupied(x1 + t * (x2 - x1), y1 + t * (y2 - y1)):
                return True
        return False

    def _segment_blocked(self, n1, n2):
        return self._segment_blocked_xy(n1.x, n1.y, n2.x, n2.y)

    def _block_check_distance(self):
        """(OPT 6) 속도 비례 막힘 검사 거리. 7m/s에서 5.5m 전방까지 검사해
        고속일수록 일찍 회피 모드 진입(=일찍 감속)."""
        return min(BLOCK_CHECK_MAX,
                   max(BLOCK_CHECK_MIN, self.current_speed * BLOCK_CHECK_TIME))

    def _pp_path_blocked(self):
        """전방 waypoint polyline이 occupancy grid에서 막혀 있는지 판정."""
        if not self.waypoints_x:
            return False
        n = len(self.waypoints_x)
        check_dist = self._block_check_distance()
        closest = self._closest_idx  # pose_callback에서 프레임당 1회 계산

        px, py = 0.0, 0.0  # 차량 원점
        idx = closest
        for _ in range(n):
            lx, ly = self.global_to_local(self.waypoints_x[idx],
                                          self.waypoints_y[idx])
            if lx > 0.0:  # 전방 waypoint만
                if self._segment_blocked_xy(px, py, lx, ly):
                    return True
                px, py = lx, ly
                if math.hypot(lx, ly) >= check_dist:
                    break
            idx = (idx + 1) % n
        return False

    # ---- RRT* path 추종 target ----

    @staticmethod
    def _densify(pts, step=0.2):
        out = [pts[0]]
        for i in range(len(pts) - 1):
            ax, ay = pts[i]
            bx, by = pts[i + 1]
            d = math.hypot(bx - ax, by - ay)
            k = max(1, int(math.ceil(d / step)))
            for j in range(1, k + 1):
                out.append((ax + (bx - ax) * j / k, ay + (by - ay) * j / k))
        return out

    def _rrt_target(self):
        """(OPT 4) RRT* path 추종 target 선택.

        기존: lookahead 원과 raw path의 교점 -> 들쭉날쭉한 path에서
        교점이 벽 쪽으로 튀어 코너에서 벽에 박는 원인.
        변경: path를 0.2m 간격으로 densify, RRT_LOOKAHEAD 호 길이까지
        자른 뒤, 차량에서 '직선 LOS가 비어 있는' 가장 먼 점을 target으로.
        target까지의 직선이 항상 충돌 없음이 보장된다.
        """
        if not self.prev_path_global:
            return None
        pts = [(0.0, 0.0)]
        for gxy in self.prev_path_global:
            pts.append(self.global_to_local(*gxy))

        dense = self._densify(pts, 0.2)

        # 호 길이 기준 RRT_LOOKAHEAD까지 자름
        cut = [dense[0]]
        acc = 0.0
        for i in range(1, len(dense)):
            acc += math.hypot(dense[i][0] - dense[i - 1][0],
                              dense[i][1] - dense[i - 1][1])
            cut.append(dense[i])
            if acc >= RRT_LOOKAHEAD:
                break

        for p in reversed(cut):
            if math.hypot(p[0], p[1]) < 0.4:
                break  # 너무 가까운 점은 target으로 무의미
            if not self._segment_blocked_xy(0.0, 0.0, p[0], p[1]):
                return p
        return None

    def follow_path(self):
        """PURE_PURSUIT / RRT* 모드 분기 (모드 판정은 pose_callback에서 수행)."""
        target = None
        if self.rrt_mode or USE_FIXED_FRONT_RRT_GOAL:
            # 일반 회피 모드 또는 고정 goal 디버그 모드에서는 RRT* path 기준 target 사용.
            # 디버그 모드에서는 차량은 정지하지만 target/path marker 확인을 위해 target을 계산한다.
            target = self._rrt_target()        # (OPT 4) LOS 보장 target
            if target is None and self.last_target is not None:
                target = self.last_target      # fallback
        if target is None:
            target = (self.pp_target_x, self.pp_target_y)

        self.last_target = target

        tx, ty = target
        angle = self._compute_steering(tx, ty, smooth=not self.rrt_mode)
        # 회피 모드 중이면 (planning 실패로 PP fallback 중이어도) 속도 cap 유지
        self._publish_drive(angle, tx, ty, rrt_mode=self.rrt_mode)
        self._publish_target_marker(tx, ty)
        if self._frame_count % WAYPOINT_VIZ_EVERY == 0:
            self._publish_waypoints_marker()
        if self.rrt_mode or not ENABLE_DRIVE or USE_FIXED_FRONT_RRT_GOAL:
            self._publish_goal_marker()

    # =================================================================
    # RRT* 핵심 알고리즘
    # =================================================================

    def _nearest_free_grid(self, gx0, gy0, max_r=24):
        """(OPT 7) (gx0, gy0) 주변 ring 탐색으로 가장 가까운 free 셀 검색.
        goal waypoint가 inflation 셀 위에 떨어졌을 때 goal을 살짝 옮겨
        planning 자체를 포기하지 않게 함. max_r=24셀 ≈ 1.2m."""
        if (0 <= gx0 < self.grid_width and 0 <= gy0 < self.grid_height
                and self.occupancy_grid[gx0, gy0] <= 0):
            return gx0, gy0
        for r in range(1, max_r + 1):
            for dx in range(-r, r + 1):
                for dy in (-r, r):
                    gx, gy = gx0 + dx, gy0 + dy
                    if (0 <= gx < self.grid_width and 0 <= gy < self.grid_height
                            and self.occupancy_grid[gx, gy] <= 0):
                        return gx, gy
            for dy in range(-r + 1, r):
                for dx in (-r, r):
                    gx, gy = gx0 + dx, gy0 + dy
                    if (0 <= gx < self.grid_width and 0 <= gy < self.grid_height
                            and self.occupancy_grid[gx, gy] <= 0):
                        return gx, gy
        return None

    def perform_rrt_star(self):
        self.final_node = None
        self.rrt_path_found = False

        # (OPT 7) goal을 grid 내부로 클램프
        self.rrt_goal_x = min(max(self.rrt_goal_x, -self.x_offset + 0.2),
                              self.grid_length_x - self.x_offset - 0.2)
        self.rrt_goal_y = min(max(self.rrt_goal_y, -self.y_offset + 0.2),
                              self.y_offset - 0.2)

        gx, gy = self.convert_to_grid(self.rrt_goal_x, self.rrt_goal_y)
        # (OPT 7) goal 셀이 점유돼 있으면 인근 free 셀로 이동
        free = self._nearest_free_grid(gx, gy)
        if free is None:
            self._publish_tree_markers_if_viz()
            return
        if free != (gx, gy):
            self.rrt_goal_x, self.rrt_goal_y = self._grid_to_local(*free)

        rrt_goal_node = RRTStarNode(self.rrt_goal_x, self.rrt_goal_y)

        # warm-start: 직전 cycle path를 트리에 seed
        self._seed_warm_start()

        # (OPT 3) seed된 path가 여전히 goal 근처까지 유효하면 즉시 성공
        # -> 회피 모드 중 매 프레임 planning이 사실상 공짜가 됨
        if len(self.nodes) > 1:
            last = self.nodes[-1]
            if (self.distance(last, rrt_goal_node) <= self.goal_threshold * 2.0
                    and self.is_collision_free(last, rrt_goal_node)):
                self.final_node = last
                self.rrt_path_found = True

        if not self.rrt_path_found:
            for _ in range(self.max_iterations):
                random_node = self.get_random_node()
                nearest_node = self.get_nearest_node(random_node)

                theta = math.atan2(random_node.y - nearest_node.y,
                                   random_node.x - nearest_node.x)
                new_x = nearest_node.x + self.step_size * math.cos(theta)
                new_y = nearest_node.y + self.step_size * math.sin(theta)
                new_node = RRTStarNode(new_x, new_y)

                if not self.is_collision_free(nearest_node, new_node):
                    continue

                neighbors = self.get_neighbors(new_node)

                best_parent = nearest_node
                best_cost = nearest_node.cost + self.distance(nearest_node, new_node)
                for nbr in neighbors:
                    c = nbr.cost + self.distance(nbr, new_node)
                    if c < best_cost and self.is_collision_free(nbr, new_node):
                        best_parent = nbr
                        best_cost = c

                new_node.parent = best_parent
                new_node.cost = best_cost
                self.nodes.append(new_node)

                self.rewire(new_node, neighbors)

                if self.distance(new_node, rrt_goal_node) <= self.goal_threshold:
                    if self.is_collision_free(new_node, rrt_goal_node):
                        self.final_node = new_node
                        self.rrt_path_found = True
                        break

        # 성공 시: (OPT 4) shortcut smoothing 후 map frame으로 저장
        if self.rrt_path_found and self.final_node is not None:
            pts = []
            node = self.final_node
            while node is not None:
                pts.append((node.x, node.y))
                node = node.parent
            pts.reverse()                               # root -> final
            pts.append((self.rrt_goal_x, self.rrt_goal_y))
            pts = self._shortcut_path(pts)
            # 원점(root)은 제외하고 저장 (추종 시 매 프레임 원점을 prepend)
            self.prev_path_global = [self.local_to_global(x, y)
                                     for x, y in pts[1:]]
        # 실패 시 prev_path_global을 지우지 않음: _rrt_target의 LOS 검사가
        # 유효성 보장 -> 일시적 planning 실패에도 직전 path로 계속 추종 가능

        self._publish_tree_markers_if_viz()

        if not self.rrt_path_found:
            self.get_logger().warn(
                f"RRT* could not reach rrt_goal "
                f"({self.rrt_goal_x:.2f}, {self.rrt_goal_y:.2f}) in "
                f"{self.max_iterations} iterations.",
                throttle_duration_sec=2.0,
            )

    def _publish_tree_markers_if_viz(self):
        """트리 전체 마커 publish 조건.

        - ENABLE_DRIVE=False: 기존 시각화 모드이므로 publish.
        - USE_FIXED_FRONT_RRT_GOAL=True: 차량 정지 + 전방 4m goal 디버그 확인용이므로 publish.
        - 일반 주행 중에는 마커 publish 부하를 줄이기 위해 생략.
        """
        if (not ENABLE_DRIVE) or USE_FIXED_FRONT_RRT_GOAL:
            self._publish_tree_markers()

    def _shortcut_path(self, pts):
        """(OPT 4) greedy shortcut: 각 점에서 직선 LOS가 비는 가장 먼 점으로
        바로 연결. RRT*의 지그재그/과도한 우회를 제거해 코너에서 불필요하게
        크게 도는 경로를 곧게 편다."""
        if len(pts) <= 2:
            return pts
        out = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and self._segment_blocked_xy(*pts[i], *pts[j]):
                j -= 1
            out.append(pts[j])
            i = j
        return out

    def _seed_warm_start(self):
        """직전 cycle에서 찾은 path(map frame)를 현재 local frame으로 환원해
        트리에 순차 seed. 충돌 셀을 만나면 거기서 중단."""
        if not self.prev_path_global:
            return
        parent = self.nodes[0]  # root
        for gx, gy in self.prev_path_global:
            lx, ly = self.global_to_local(gx, gy)
            node = RRTStarNode(lx, ly)
            if not self.is_collision_free(parent, node):
                break
            node.parent = parent
            node.cost = parent.cost + self.distance(parent, node)
            self.nodes.append(node)
            parent = node

    def get_random_node(self):
        # 10% 확률로 goal을 직접 샘플링 (biased sampling)
        if random.random() < 0.18:
            return RRTStarNode(self.rrt_goal_x, self.rrt_goal_y)

        x_min = -self.x_offset
        x_max = self.grid_length_x - self.x_offset
        y_min = -self.y_offset
        y_max = self.grid_length_y - self.y_offset

        for _ in range(100):
            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)
            gx, gy = self.convert_to_grid(x, y)
            if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                if self.occupancy_grid[gx, gy] <= 0:
                    return RRTStarNode(x, y)

        return RRTStarNode(random.uniform(x_min, x_max), random.uniform(y_min, y_max))

    def get_nearest_node(self, random_node):
        return min(self.nodes, key=lambda n: self.distance(n, random_node))

    def get_neighbors(self, node):
        return [n for n in self.nodes if self.distance(n, node) <= self.neighborhood_radius]

    def rewire(self, new_node, neighbors):
        for neighbor in neighbors:
            if neighbor is new_node or neighbor is new_node.parent:
                continue
            new_cost = new_node.cost + self.distance(new_node, neighbor)
            if new_cost < neighbor.cost and self.is_collision_free(new_node, neighbor):
                neighbor.parent = new_node
                neighbor.cost = new_cost

    def is_collision_free(self, nearest_node, new_node):
        dist = self.distance(nearest_node, new_node)
        steps = max(2, int(math.ceil(dist / (self.grid_resolution * 0.5))))

        for i in range(steps + 1):
            t = i / steps
            x = nearest_node.x + t * (new_node.x - nearest_node.x)
            y = nearest_node.y + t * (new_node.y - nearest_node.y)
            gx, gy = self.convert_to_grid(x, y)

            if not (0 <= gx < self.grid_width and 0 <= gy < self.grid_height):
                return False
            if self.occupancy_grid[gx, gy] > 0:
                return False

        return True

    def distance(self, node1, node2):
        return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

    # =================================================================
    # Shutdown 정리 (제공)
    # =================================================================

    def clear_visualization(self):
        tree_clear = MarkerArray()
        tree_delete = Marker()
        tree_delete.header.frame_id = self.rrt_tree_markers_frame_id
        tree_delete.action = Marker.DELETEALL
        tree_delete.id = -1
        tree_clear.markers.append(tree_delete)
        self.rrt_tree_marker_pub.publish(tree_clear)

        path_clear = MarkerArray()
        path_delete = Marker()
        path_delete.header.frame_id = 'map'
        path_delete.action = Marker.DELETEALL
        path_delete.id = -1
        path_clear.markers.append(path_delete)
        self.path_marker_pub.publish(path_clear)

        for pub, frame in (
            (self.waypoints_marker_pub, 'map'),
            (self.target_marker_pub, 'map'),
            (self.goal_marker_pub, 'map'),
        ):
            m = Marker()
            m.header.frame_id = frame
            m.action = Marker.DELETE
            m.id = 0
            pub.publish(m)

    def stop_vehicle(self):
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = 0.0
        msg.drive.speed = 0.0
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    def _sigterm_handler(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sigterm_handler)

    rrt_star_node = RRTStar()
    try:
        rclpy.spin(rrt_star_node)
    except KeyboardInterrupt:
        pass
    finally:
        rrt_star_node.stop_vehicle()
        rrt_star_node.clear_visualization()
        time.sleep(0.2)
        rrt_star_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()