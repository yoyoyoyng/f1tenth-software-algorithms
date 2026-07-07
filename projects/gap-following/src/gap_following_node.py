import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from visualization_msgs.msg import Marker


class GapFollow(Node):
    def __init__(self):
        super().__init__('gap_follow_node')

        self.lidarscan_topic = '/scan'
        self.drive_topic = '/drive'
        self.best_point_marker_topic = '/best_point_marker'
        self.bubble_marker_topic = '/bubble_point_marker'

        self.lidar_sub = self.create_subscription(
            LaserScan, self.lidarscan_topic, self.scan_callback, 10
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 10
        )
        self.best_point_marker_pub = self.create_publisher(
            Marker, self.best_point_marker_topic, 10
        )
        self.bubble_marker_pub = self.create_publisher(
            Marker, self.bubble_marker_topic, 10
        )

        # =============================================================
        #  파라미터
        # =============================================================

        # max_range: LiDAR 최대 감지 거리 클리핑 [m]
        # ┌ 역할: 이 거리 이상의 LiDAR 값을 잘라냄. best_distance 최대값을 결정.
        # ├ 올리면: 더 멀리 봄 → best point가 멀리 잡힘 → calc_speed에서 고속 나옴
        # │         직선에서 빠르게 달릴 수 있음
        # ├ 내리면: 가까운 것만 봄 → best_distance가 항상 작음 → 전체적으로 느려짐
        # │         하지만 직선 끝 코너에서 감속이 빨라져서 안전해짐
        # ├ 직선 끝 정면 벽에 박히면: 내리기 (7.2 → 5.0)
        # ├ 직선에서 너무 느리면: 올리기 (7.2 → 9.0)
        # └ 범위: 3.0 ~ 10.0
        self.max_range = 11.0

        # front_deg: 앞쪽 FOV 범위 [deg] ([-front_deg, +front_deg] 사용)
        # ┌ 역할: LiDAR 데이터 중 이 각도 범위만 gap 탐색에 사용.
        # ├ 올리면: 옆쪽 gap도 감지 → 커브를 일찍 발견해서 빨리 꺾음
        # │         하지만 직선에서 옆벽 gap을 잡아 옆으로 쏠릴 수 있음
        # ├ 내리면: 정면만 봄 → 직진 안정적, 장애물 직진 회피 잘 됨
        # │         하지만 커브를 늦게 감지해서 돌지 못하고 정면 벽에 박힐 수 있음
        # ├ 커브를 너무 일찍 돌면: 내리기 (90 → 70)
        # ├ 커브를 너무 늦게 돌면: 올리기 (90 → 120)
        # ├ 직선에서 사이드벽을 긁으면: 내리기 (옆벽 gap 감지 차단)
        # └ 범위: 60 ~ 160
        self.front_deg = 90

        # smoothing_window: LiDAR 이동평균 필터 크기 [beams]
        # ┌ 역할: LiDAR ranges를 이 크기로 평균내서 노이즈 제거.
        # ├ 올리면: 노이즈 줄어듦 → 조향이 안정적, gap score 변동 줄어듦
        # │         하지만 장애물 경계가 뭉개져서 disparity 감지가 늦어짐
        # ├ 내리면: 장애물 경계 선명 → disparity 정확
        # │         하지만 노이즈로 조향이 떨리고 best point가 불안정
        # ├ 직선에서 좌우로 흔들리면: 올리기 (7 → 9)
        # ├ 장애물을 못 피하면: 내리기 (7 → 3)
        # └ 범위: 1(끔) ~ 11. 홀수 권장
        self.smoothing_window = 1

        # disparity_threshold: disparity 판단 임계값 [m]
        # ┌ 역할: 인접 LiDAR 빔 간 거리 차이가 이 값 이상이면 disparity로 판단.
        # │       disparity 지점에서 bubble을 씌워 장애물 뒤 공간을 차단.
        # ├ 올리면: 큰 차이만 disparity로 감지 → bubble 수 감소 → 좁은 통로 통과 가능
        # │         하지만 작은 장애물을 놓쳐서 모서리에 박힐 수 있음
        # ├ 내리면: 작은 차이도 감지 → bubble 많아짐 → 안전
        # │         하지만 bubble이 과다해서 gap이 전부 사라지고 좁은 통로 못 지남
        # ├ 장애물 모서리에 박히면: 내리기 (0.3 → 0.15)
        # ├ 좁은 통로를 못 지나가면: 올리기 (0.3 → 0.8)
        # └ 범위: 0.1 ~ 1.0
        self.disparity_threshold = 0.35

        # car_width: 차량 폭 [m]
        # ┌ 역할: disparity bubble 크기 계산 시 차폭으로 사용.
        # │       (car_width/2 + extra_safety_margin)만큼 bubble을 씌움.
        # ├ 올리면: bubble 넓어짐 → 장애물 회피 마진 증가
        # │         하지만 좁은 통로에서 bubble이 통로를 다 막아버림
        # ├ 내리면: bubble 좁아짐 → 좁은 통로 통과 가능
        # │         하지만 장애물 모서리를 스치며 지나감
        # ├ 장애물에 긁히면: 올리기 (0.35 → 0.40)
        # ├ 좁은 통로 못 지나가면: 내리기 (0.35 → 0.28)
        # └ 실제 차량 폭에 맞추기. F1TENTH: 0.28 ~ 0.40
        self.car_width = 0.30

        # extra_safety_margin: 차폭 위에 추가하는 여유 마진 [m]
        # ┌ 역할: car_width에 더해서 bubble을 넓힘. 고속 미끄러짐 대비용.
        # ├ 올리면: bubble 더 넓어짐 → 고속에서 미끄러져도 안전
        # │         하지만 좁은 통로에서 bubble이 통로를 막음
        # ├ 내리면: bubble 축소 → 좁은 통로 통과 가능
        # │         하지만 고속에서 장애물 모서리를 스침
        # ├ 고속에서 장애물에 긁히면: 올리기 (0.15 → 0.25)
        # ├ 좁은 통로 못 지나가면: 내리기 (0.15 → 0.05)
        # └ 범위: 0.02 ~ 0.35
        self.extra_safety_margin = 0.05

        # max_distance_threshold: center-following 활성화 거리 [m]
        # ┌ 역할: gap 내 최대 거리가 이 값 이상이면 deepest point 대신
        # │       deep region의 '중심'을 따라감 (Tweak 4: wiggling 방지).
        # ├ 올리면: deepest point 위주 → 커브 반응 빠름
        # │         하지만 장애물 양쪽에서 best point가 좌↔우 왔다갔다 (wiggling)
        # ├ 내리면: 빨리 center-following 모드 진입 → 안정적
        # │         하지만 커브 진입이 느려질 수 있음
        # ├ 장애물 앞에서 좌우 왔다갔다하면: 내리기 (3.0 → 2.0)
        # ├ 커브 반응이 늦으면: 올리기 (3.0 → 5.0)
        # └ 범위: 1.5 ~ 6.0. max_range의 40~70%가 적당
        self.max_distance_threshold = 3.18
        # speed_straight: 최대 속도 [m/s]
        # ┌ 역할: best_distance가 max_range일 때 이 속도까지 올라감.
        # │       거리 비례 속도: speed_corner + (dist/max_range) × (speed_straight - speed_corner)
        # ├ 올리면: 직선에서 더 빠름 → 랩타임 감소
        # │         하지만 감속이 늦으면 코너 벽에 박힘
        # ├ 내리면: 전체적으로 느려짐 → 안전
        # ├ 여기저기 박히면: 내리기 (8.5 → 5.0)
        # ├ 직선이 느리면: 올리기 (8.5 → 10.0)
        # └ 범위: 2.0 ~ 12.0
        self.speed_straight = 11.85

        # speed_corner: 최소 속도 [m/s]
        # ┌ 역할: best_distance가 0일 때(벽 바로 앞) 이 속도까지 내려감.
        # │       어떤 상황에서도 이 속도 아래로는 안 떨어짐.
        # ├ 올리면: 벽 가까울 때도 빠름 → 랩타임 감소
        # │         하지만 장애물 근처에서 감속이 부족해 충돌
        # ├ 내리면: 벽 가까울 때 충분히 감속 → 안전
        # │         하지만 좁은 구간에서 너무 느려짐
        # ├ 커브/장애물에서 박히면: 내리기 (3.5 → 2.0)
        # ├ 전체적으로 느리면: 올리기 (3.5 → 4.5)
        # └ 범위: 1.0 ~ 5.0
        self.speed_corner = 4.82

        # max_steer_for_speed: 이 조향각(deg)에서 최대 감속 적용
        # ┌ 역할: 조향각이 이 값일 때 angle_speed_penalty만큼 속도 감소.
        # │       이 각도 이상은 더 감속 안 함 (이미 최대 감속).
        # ├ 올리면: 큰 조향까지 고속 허용 → 공격적
        # ├ 내리면: 작은 조향에서도 감속 시작 → 보수적
        # ├ 커브 바깥벽에 박히면: 내리기 (30 → 20)
        # ├ 커브에서 너무 느리면: 올리기 (30 → 45)
        # └ 범위: 15 ~ 50
        self.max_steer_for_speed = 21

        # angle_speed_penalty: 최대 조향 시 속도 감소 비율 (0~1)
        # ┌ 역할: 조향각이 max_steer_for_speed일 때 속도를 이 비율만큼 깎음.
        # │       예) 0.6이면 최대 조향 시 속도가 40%로 줄어듦.
        # ├ 올리면: 커브에서 더 감속 → 원심력 줄어듦 → 안전
        # ├ 내리면: 커브에서 덜 감속 → 빠름 → 바깥벽 충돌 위험
        # ├ 커브 바깥벽에 박히면: 올리기 (0.6 → 0.8)
        # ├ 커브가 너무 느리면: 내리기 (0.6 → 0.4)
        # └ 범위: 0.2 ~ 0.9
        self.angle_speed_penalty = 0.8

        # emergency_distance: 비상 브레이크 발동 거리 [m]
        # ┌ 역할: 정면 ±emergency_fov_deg 범위에 이 거리 이내 장애물이면 급감속.
        # ├ 올리면: 더 일찍 비상 발동 → 정면 충돌 방지
        # │         하지만 커브 벽이 보일 때마다 멈칫거림
        # ├ 내리면: 늦게 발동 → 자연스러움
        # │         하지만 정면 장애물에 대응 못함
        # ├ 정면 벽에 박히면: 올리기 (1.0 → 1.5)
        # ├ 자꾸 멈칫거리면: 내리기 (1.0 → 0.5)
        # └ 범위: 0.3 ~ 2.0
        self.emergency_distance = 1.25

        # emergency_speed: 비상 브레이크 시 속도 [m/s]
        # ┌ 역할: 비상 발동 시 이 속도로 강제 설정.
        # ├ 올리면: 비상 시에도 어느 정도 속도 유지 (부드러운 감속)
        # ├ 내리면: 비상 시 급정거에 가까움
        # └ 범위: 0.0 ~ speed_corner
        self.emergency_speed = 2.97

        # emergency_fov_deg: 비상 체크 범위 [deg] (정면 ±이 각도)
        # ┌ 역할: 이 각도 범위 내에서 emergency_distance 이내 장애물을 체크.
        # ├ 올리면: 넓은 범위 체크 → 측면 벽에도 반응 → 자주 발동
        # ├ 내리면: 정면 좁은 범위만 → 진짜 정면 장애물에만 반응
        # ├ 비상이 자주 걸리면: 내리기 (8 → 3)
        # ├ 정면 장애물에 못 반응하면: 올리기 (8 → 20)
        # └ 범위: 3 ~ 30
        self.emergency_fov_deg = 6

        # steering_smooth_alpha: 조향 스무딩 계수 (0~1)
        # ┌ 역할: EMA(지수이동평균)으로 조향을 부드럽게.
        # │       smoothed = alpha × 새값 + (1-alpha) × 이전값
        # ├ 올리면 (→1.0): 새 조향에 즉시 반응 → 커브 반응 빠름
        # │         하지만 노이즈에 민감해서 조향 떨림
        # ├ 내리면 (→0.0): 이전 조향 유지 → 부드러움, 직진 안정
        # │         하지만 커브 반응이 느려서 돌지 못함
        # ├ 커브를 못 돌면: 올리기 (0.8 → 0.9)
        # ├ 직선에서 흔들리면: 내리기 (0.8 → 0.4)
        # ├ 장애물 앞에서 좌우 왔다갔다: 내리기 (0.8 → 0.5)
        # └ 범위: 0.2 ~ 1.0
        self.prev_steering = 0.2
        self.steering_smooth_alpha = 0.2

        # corner_min_steer: 옆벽 체크 최소 조향각 [rad]
        # ┌ 역할: 조향각이 이 값 이상일 때만 corner_safety_override 작동.
        # │       직선에서 미세 조향 시 옆벽 체크가 오발동하는 것을 방지.
        # ├ 올리면: 큰 조향에서만 체크 → 직선 오발동 방지
        # │         하지만 완만한 커브에서 옆벽 보호 안 됨
        # ├ 내리면: 작은 조향에서도 체크 → 옆벽 보호 강화
        # │         하지만 직선에서도 발동해서 커브 못 돎
        # ├ 직선에서 갑자기 꺾이면: 올리기 (8 → 12도)
        # ├ 커브에서 옆벽에 박히면: 내리기 (8 → 5도)
        # └ 범위: 3 ~ 15도
        self.corner_min_steer = np.radians(10.0)

        # corner_side_dist: 옆벽 조향 제한 거리 [m]
        # ┌ 역할: 회전 방향 옆벽(±90도 바깥)이 이 거리 이내면
        # │       조향을 비례적으로 줄임 (완전 차단이 아님).
        # │       벽 0.5m → 100% 유지, 벽 0.25m → 50%, 벽 0.05m → 10%
        # ├ 올리면: 일찍 조향 제한 → 옆벽 충돌 방지
        # │         하지만 커브를 충분히 못 돌 수 있음
        # ├ 내리면: 가까울 때만 제한 → 자유로운 회전z 
        # │         하지만 옆벽에 차 옆면이 닿을 위험
        # ├ 커브에서 옆벽에 박히면: 올리기 (0.5 → 0.7)
        # ├ 커브를 못 돌면: 내리기 (0.5 → 0.3)
        # └ 범위: 0.2 ~ 1.0
        self.corner_side_dist = 0.1

    # =============================================================
    #  마커 퍼블리시
    # =============================================================

    def publish_best_point_marker(self, best_point_idx, best_point_distance, angle_min, angle_increment):
        best_point_angle = angle_min + best_point_idx * angle_increment
        x = best_point_distance * np.cos(best_point_angle)
        y = best_point_distance * np.sin(best_point_angle)

        marker = Marker()
        marker.header.frame_id = "ego_racecar/laser"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "best_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.best_point_marker_pub.publish(marker)

    def publish_closest_bubble_marker(self, bubble_idx, bubble_distance, bubble_radius, angle_min, angle_increment):
        bubble_point_angle = angle_min + bubble_idx * angle_increment
        x = bubble_distance * np.cos(bubble_point_angle)
        y = bubble_distance * np.sin(bubble_point_angle)

        marker = Marker()
        marker.header.frame_id = "ego_racecar/laser"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "bubble_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = bubble_radius
        marker.scale.y = bubble_radius
        marker.scale.z = bubble_radius
        marker.color.a = 0.5
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.bubble_marker_pub.publish(marker)

    # =============================================================
    #  Helper 함수들
    # =============================================================

    def preprocess_lidar(self, ranges):
        proc = np.array(ranges, dtype=np.float32)
        proc[np.isnan(proc)] = 0.0
        proc[np.isinf(proc)] = self.max_range
        proc = np.clip(proc, 0.0, self.max_range)

        if self.smoothing_window > 1:
            kernel = np.ones(self.smoothing_window) / self.smoothing_window
            proc = np.convolve(proc, kernel, mode='same')

        return proc

    def get_front_indices(self, data):
        angle_min = data.angle_min
        angle_inc = data.angle_increment
        n = len(data.ranges)

        start_angle = -np.deg2rad(self.front_deg)
        end_angle = np.deg2rad(self.front_deg)

        start_idx = int(np.floor((start_angle - angle_min) / angle_inc))
        end_idx = int(np.ceil((end_angle - angle_min) / angle_inc))

        start_idx = max(0, start_idx)
        end_idx = min(n - 1, end_idx)

        return start_idx, end_idx

    def get_side_indices(self, data):
        angle_min = data.angle_min
        angle_inc = data.angle_increment
        n = len(data.ranges)

        angles = angle_min + np.arange(n) * angle_inc
        left_side = np.where(angles > np.deg2rad(90.0))[0]
        right_side = np.where(angles < np.deg2rad(-90.0))[0]

        return left_side, right_side

    def find_disparities(self, ranges):
        disparities = []
        for i in range(len(ranges) - 1):
            if abs(ranges[i + 1] - ranges[i]) > self.disparity_threshold:
                disparities.append(i)
        return disparities

    def get_num_points_to_cover(self, dist, angle_increment):
        width_to_cover = (self.car_width / 2.0) + self.extra_safety_margin
        if dist <= 1e-3:
            return 0

        arc_angle = 2.0 * np.arcsin(np.clip(width_to_cover / (2.0 * dist), 0.0, 1.0))
        num_points = int(np.ceil(arc_angle / angle_increment))
        return max(1, num_points)

    def extend_disparities(self, ranges, angle_increment):
        new_ranges = np.copy(ranges)
        disparities = self.find_disparities(ranges)

        for i in disparities:
            left = new_ranges[i]
            right = new_ranges[i + 1]

            if left < right:
                close_dist = left
                num_points = self.get_num_points_to_cover(close_dist, angle_increment)
                start = i + 1
                end = min(len(new_ranges), i + 1 + num_points)
                new_ranges[start:end] = np.minimum(new_ranges[start:end], close_dist)
            else:
                close_dist = right
                num_points = self.get_num_points_to_cover(close_dist, angle_increment)
                start = max(0, i - num_points + 1)
                end = i + 1
                new_ranges[start:end] = np.minimum(new_ranges[start:end], close_dist)

        return new_ranges, disparities

    def find_gaps(self, ranges):
        gaps = []
        in_gap = False
        gap_start = 0

        for i in range(len(ranges)):
            if ranges[i] > 1e-3 and not in_gap:
                gap_start = i
                in_gap = True
            elif ranges[i] <= 1e-3 and in_gap:
                gaps.append((gap_start, i - 1))
                in_gap = False

        if in_gap:
            gaps.append((gap_start, len(ranges) - 1))

        return gaps

    def choose_gap(self, ranges):
        gaps = self.find_gaps(ranges)
        if len(gaps) == 0:
            return None

        best_gap = None
        best_score = -1.0

        for start, end in gaps:
            gap_len = end - start + 1
            gap_depth = np.max(ranges[start:end + 1])
            score = gap_len * gap_depth
            if score > best_score:
                best_score = score
                best_gap = (start, end)

        return best_gap

    def choose_best_point(self, ranges, gap_start, gap_end):
        gap_ranges = ranges[gap_start:gap_end + 1]
        if len(gap_ranges) == 0:
            return (gap_start + gap_end) // 2

        max_in_gap = np.max(gap_ranges)

        if max_in_gap >= self.max_distance_threshold:
            deep_indices = np.where(gap_ranges >= self.max_distance_threshold)[0]
            if len(deep_indices) > 0:
                center_local = int((deep_indices[0] + deep_indices[-1]) / 2)
                return gap_start + center_local

        local_best = int(np.argmax(gap_ranges))
        return gap_start + local_best

    def smooth_steering(self, raw_steering):
        smoothed = (self.steering_smooth_alpha * raw_steering +
                    (1.0 - self.steering_smooth_alpha) * self.prev_steering)
        self.prev_steering = smoothed
        return smoothed

    def check_emergency(self, data, proc_ranges):
        angle_min = data.angle_min
        angle_inc = data.angle_increment
        n = len(proc_ranges)

        start_angle = -np.deg2rad(self.emergency_fov_deg)
        end_angle = np.deg2rad(self.emergency_fov_deg)

        start_idx = int(np.floor((start_angle - angle_min) / angle_inc))
        end_idx = int(np.ceil((end_angle - angle_min) / angle_inc))
        start_idx = max(0, start_idx)
        end_idx = min(n - 1, end_idx)

        front_narrow = proc_ranges[start_idx:end_idx + 1]
        valid = front_narrow[front_narrow > 0.01]

        if len(valid) > 0 and np.min(valid) < self.emergency_distance:
            return True
        return False

    def corner_safety_override(self, data, steering_angle):
        proc = self.preprocess_lidar(data.ranges)
        left_side, right_side = self.get_side_indices(data)

        if steering_angle > self.corner_min_steer:
            if len(left_side) > 0:
                min_dist = np.min(proc[left_side])
                if min_dist < self.corner_side_dist:
                    ratio = np.clip(min_dist / self.corner_side_dist, 0.1, 1.0)
                    return steering_angle * ratio

        if steering_angle < -self.corner_min_steer:
            if len(right_side) > 0:
                min_dist = np.min(proc[right_side])
                if min_dist < self.corner_side_dist:
                    ratio = np.clip(min_dist / self.corner_side_dist, 0.1, 1.0)
                    return steering_angle * ratio

        return steering_angle

    def calc_speed(self, steering_angle, best_distance, is_emergency):
        if is_emergency:
            return self.emergency_speed

        dist_ratio = np.clip(best_distance / self.max_range, 0.0, 1.0)
        distance_speed = self.speed_corner + dist_ratio * (self.speed_straight - self.speed_corner)

        abs_deg = abs(np.degrees(steering_angle))
        angle_ratio = np.clip(abs_deg / self.max_steer_for_speed, 0.0, 1.0)
        angle_factor = 1.0 - self.angle_speed_penalty * angle_ratio

        speed = distance_speed * angle_factor
        speed = np.clip(speed, self.speed_corner, self.speed_straight)

        return speed

    def scan_callback(self, data):
        raw_ranges = np.array(data.ranges)

        proc_ranges = self.preprocess_lidar(raw_ranges)

        is_emergency = self.check_emergency(data, proc_ranges)

        start_idx, end_idx = self.get_front_indices(data)
        front_ranges = np.copy(proc_ranges[start_idx:end_idx + 1])

        virtual_front_ranges, disparities = self.extend_disparities(front_ranges, data.angle_increment)
        virtual_front_ranges[virtual_front_ranges < 0.05] = 0.0

        nonzero = np.where(front_ranges > 1e-3)[0]
        if len(nonzero) > 0:
            nearest_local = nonzero[np.argmin(front_ranges[nonzero])]
            nearest_global = start_idx + nearest_local
            nearest_dist = front_ranges[nearest_local]
        else:
            nearest_local = len(front_ranges) // 2
            nearest_global = start_idx + nearest_local
            nearest_dist = 0.0

        bubble_radius = (self.car_width / 2.0) + self.extra_safety_margin

        chosen_gap = self.choose_gap(virtual_front_ranges)

        if chosen_gap is None:
            best_local = len(virtual_front_ranges) // 2
            best_global = start_idx + best_local
            best_distance = 0.0
            steering_angle = 0.0
        else:
            gap_start, gap_end = chosen_gap
            best_local = self.choose_best_point(virtual_front_ranges, gap_start, gap_end)
            best_global = start_idx + best_local
            best_distance = virtual_front_ranges[best_local]
            steering_angle = data.angle_min + best_global * data.angle_increment

        steering_angle = self.corner_safety_override(data, steering_angle)
        steering_angle = self.smooth_steering(steering_angle)
        speed = self.calc_speed(steering_angle, best_distance, is_emergency)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(speed)
        self.drive_pub.publish(drive_msg)

        self.publish_best_point_marker(
            best_global, best_distance, data.angle_min, data.angle_increment
        )
        self.publish_closest_bubble_marker(
            nearest_global, nearest_dist, bubble_radius, data.angle_min, data.angle_increment
        )


def main(args=None):
    rclpy.init(args=args)
    print("Reactive Gap Follower Node Initialized")
    reactive_node = GapFollow()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()