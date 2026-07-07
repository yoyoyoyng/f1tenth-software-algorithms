#!/usr/bin/env python3
"""
Pure Pursuit - tuning parameters gathered at top

waypoints.csv 형식:
x, y, heading, speed

튜닝은 아래 TUNING PARAMETERS 블록만 수정하면 됨.
"""

import rclpy
from rclpy.node import Node

import numpy as np
import math
import os
import csv

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


# ============================================================
# TUNING PARAMETERS - 여기만 수정하면 됨
# ============================================================

# ------------------------------
# 1. 파일 / 패키지 설정
# ------------------------------
WAYPOINTS_FILENAME = "waypoints.csv"
WAYPOINTS_INTERVAL = 1
PACKAGE_NAME = "lab5"


# ------------------------------
# 2. 차량 기본 파라미터
# ------------------------------

# 직선 최고속.
# 직선이 너무 느리면 올림: 10.0 -> 11.0 -> 12.0
# 코너에서 계속 박으면 먼저 이걸 올리지 말고 코너 감속/waypoint부터 조정.
MAX_SPEED = 12.0

# 최저속.
# 코너에서 거의 멈추면 올림: 1.2 -> 1.5
# 코너를 못 돌면 낮춤: 1.2 -> 0.8
MIN_SPEED = 1.2

# 최대 조향각.
# 0.4189 rad는 약 24도.
# 커브에서 물리적으로 조향이 부족하면 0.45~0.50까지 테스트 가능.
MAX_STEER = 0.4189

# 차량 축간거리. 보통 건드리지 말 것.
WHEELBASE = 0.33


# ------------------------------
# 3. Lookahead 설정
# ------------------------------
# speed <= threshold 이면 해당 lookahead 사용.
#
# 직선에서 좌우로 흔들림:
#   고속 lookahead 증가. 예: 2.50 -> 2.80
#
# 코너에서 바깥벽에 박음:
#   저속/중속 lookahead 감소. 예: 0.95 -> 0.80
#
# 코너에서 안쪽벽에 박음:
#   저속/중속 lookahead 증가. 예: 0.95 -> 1.10
LOOKAHEAD_TABLE = [
    (3.0, 0.75),
    (4.5, 0.95),
    (5.5, 1.20),
    (6.5, 1.60),
    (8.0, 2.05),
    (10.0, 2.50),
]

# 곡률이 커질 때 lookahead를 줄이기 시작하는 기준.
# 코너에서 너무 늦게 꺾으면 낮춤: 0.08 -> 0.06
# 코너에서 너무 안쪽으로 말리면 올림: 0.08 -> 0.10
LOOKAHEAD_CURV_REDUCE_THRESHOLD = 0.08

# 곡률이 큰 구간에서 lookahead를 최소 몇 배까지 줄일지.
# 낮을수록 코너에서 더 가까운 점을 보고 빨리 꺾음.
# 바깥벽에 박으면 낮춤: 0.85 -> 0.70
# 안쪽벽에 박으면 올림: 0.85 -> 0.95
LOOKAHEAD_CURV_MIN_FACTOR = 0.85


# ------------------------------
# 4. 조향 설정
# ------------------------------

# 조향 증폭 계수.
# 커브에서 못 꺾고 바깥벽에 박으면 올림: 1.00 -> 1.10 -> 1.20
# 코너에서 안쪽벽으로 말리면 낮춤: 1.00 -> 0.90
STEERING_GAIN = 1.00

# 조향 smoothing.
# 1에 가까울수록 조향 명령을 바로 반영.
# 코너 반응이 늦으면 올림: 0.65 -> 0.80
# 직선에서 좌우로 흔들리면 낮춤: 0.65 -> 0.50
STEER_ALPHA = 0.65


# ------------------------------
# 5. 속도 smoothing 설정
# ------------------------------

# 가속 smoothing.
# 작을수록 가속이 천천히 됨.
# 직선에서 속도 올라가는 게 너무 느리면 올림: 0.35 -> 0.50
# 출발/직선에서 흔들리면 낮춤: 0.35 -> 0.25
# 감속은 smoothing 없이 즉시 적용됨.
SPEED_ALPHA_ACCEL = 0.35


# ------------------------------
# 6. 커브 사전 감지 설정
# ------------------------------

# 앞으로 몇 m 안의 커브를 보고 미리 감속할지.
# 커브 진입 전에 감속이 늦으면 증가: 5.0 -> 7.0 -> 10.0
# 너무 일찍 감속해서 전체가 느리면 감소: 5.0 -> 3.5
FUTURE_CURVE_DISTANCE = 5.0

# 곡률 계산 smoothing window.
# 커브를 더 일찍 감지하고 싶으면 증가: 8 -> 12
# 너무 일찍 감속해서 느리면 감소: 8 -> 5
CURVATURE_SMOOTH_WINDOW = 8

# 곡률 계산 기준 gap.
# 너무 민감하면 증가: 3 -> 4 or 5
# 코너 감지가 둔하면 감소: 3 -> 2
CURVATURE_GAP = 3


# ------------------------------
# 7. 곡률 기반 속도 제한
# ------------------------------
# 값이 작을수록 더 쉽게 커브라고 판단해서 감속함.
# 너무 느리면 tier 값을 올림.
# 커브에서 바깥벽에 박으면 tier 값을 낮춤.
CURV_TIER_1 = 0.05
CURV_TIER_2 = 0.10
CURV_TIER_3 = 0.15
CURV_TIER_4 = 0.20

# 곡률별 속도 제한.
# 직선이 느리면 CURV_SPEED_1~2를 올림.
# 코너에서 박으면 CURV_SPEED_3~4를 낮춤.
CURV_SPEED_1 = 8.5
CURV_SPEED_2 = 6.8
CURV_SPEED_3 = 5.0
CURV_SPEED_4 = 3.5


# ------------------------------
# 8. 조향각 기반 속도 제한
# ------------------------------
# 조향각이 커질수록 속도를 줄임.
# 직선에서 조금만 조향해도 느려지면 각도 threshold를 올리거나 speed limit을 올림.
# 커브에서 박으면 speed limit을 낮춤.
STEER_DEG_1 = 6.0
STEER_DEG_2 = 10.0
STEER_DEG_3 = 15.0
STEER_DEG_4 = 20.0

STEER_SPEED_1 = 9.0
STEER_SPEED_2 = 7.5
STEER_SPEED_3 = 5.5
STEER_SPEED_4 = 3.8


# ------------------------------
# 9. nearest waypoint 탐색 범위
# ------------------------------
# 차량이 갑자기 엉뚱한 waypoint를 잡으면 100~160 사이에서 조정.
NEAREST_SEARCH_WINDOW = 120


# ============================================================
# 코드 본문 - 여기 아래는 보통 수정하지 말 것
# ============================================================

try:
    waypoint_dir = get_package_share_directory(PACKAGE_NAME)
    waypoint_filepath = os.path.join(waypoint_dir, WAYPOINTS_FILENAME)
except (PackageNotFoundError, Exception):
    waypoint_filepath = os.path.join(os.getcwd(), PACKAGE_NAME, WAYPOINTS_FILENAME)


class PurePursuit(Node):
    def __init__(self):
        super().__init__("pure_pursuit_node")

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            "/drive",
            10
        )

        self.waypoints_marker_pub = self.create_publisher(
            Marker,
            "/waypoints_marker",
            10
        )

        self.target_marker_pub = self.create_publisher(
            Marker,
            "/target_marker",
            10
        )

        self.create_subscription(
            Odometry,
            "/ego_racecar/odom",
            self.pose_callback,
            10
        )

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.heading_angle = 0.0

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_idx = 0

        self.prev_speed = 0.0
        self.prev_steer = 0.0
        self.prev_nearest_idx = 0
        self.initialized_nearest = False
        self.callback_count = 0

        self.waypoints_x = np.array([])
        self.waypoints_y = np.array([])
        self.waypoints_speed = np.array([])
        self.curvature = np.array([])
        self.waypoints_s = np.array([])
        self.total_length = 0.0

        self.load_waypoints(waypoint_filepath, WAYPOINTS_INTERVAL)

    def load_waypoints(self, filename, interval=1):
        self.get_logger().info(f"Waypoint file path: {filename}")

        if not os.path.exists(filename):
            self.get_logger().error(f"Waypoint file does not exist: {filename}")
            return

        xs, ys, speeds = [], [], []

        with open(filename, "r") as file:
            reader = csv.reader(file)

            for i, row in enumerate(reader):
                if len(row) == 0:
                    continue

                try:
                    values = list(map(float, row))
                except ValueError:
                    continue

                if len(values) < 2:
                    continue

                if i % interval != 0:
                    continue

                x = values[0]
                y = values[1]

                if len(values) >= 4:
                    speed = values[3]
                else:
                    speed = MAX_SPEED

                xs.append(x)
                ys.append(y)
                speeds.append(float(np.clip(speed, MIN_SPEED, MAX_SPEED)))

        self.waypoints_x = np.array(xs)
        self.waypoints_y = np.array(ys)
        self.waypoints_speed = np.array(speeds)

        self.compute_cumulative_s()
        self.compute_curvature()

        if len(self.waypoints_x) > 0:
            self.get_logger().info(
                f"Loaded {len(self.waypoints_x)} waypoints, "
                f"speed range: {self.waypoints_speed.min():.2f} ~ {self.waypoints_speed.max():.2f}, "
                f"curvature max: {self.curvature.max():.3f}"
            )

    def compute_cumulative_s(self):
        n = len(self.waypoints_x)

        if n == 0:
            self.waypoints_s = np.array([])
            self.total_length = 0.0
            return

        s = [0.0]

        for i in range(1, n):
            dx = self.waypoints_x[i] - self.waypoints_x[i - 1]
            dy = self.waypoints_y[i] - self.waypoints_y[i - 1]
            s.append(s[-1] + math.sqrt(dx * dx + dy * dy))

        dx = self.waypoints_x[0] - self.waypoints_x[-1]
        dy = self.waypoints_y[0] - self.waypoints_y[-1]
        self.total_length = s[-1] + math.sqrt(dx * dx + dy * dy)
        self.waypoints_s = np.array(s)

    def compute_curvature(self):
        n = len(self.waypoints_x)

        if n == 0:
            self.curvature = np.array([])
            return

        gap = CURVATURE_GAP
        curv = np.zeros(n)
        pts = np.vstack([self.waypoints_x, self.waypoints_y]).T

        for i in range(n):
            p0 = pts[(i - gap) % n]
            p1 = pts[i]
            p2 = pts[(i + gap) % n]

            v1 = p1 - p0
            v2 = p2 - p1

            n1 = np.linalg.norm(v1)
            n2 = np.linalg.norm(v2)

            if n1 < 1e-6 or n2 < 1e-6:
                continue

            cross = v1[0] * v2[1] - v1[1] * v2[0]
            dot = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
            angle = abs(math.atan2(cross, dot))

            curv[i] = angle / max(0.5 * (n1 + n2), 1e-6)

        smooth = np.zeros(n)
        window = CURVATURE_SMOOTH_WINDOW

        for i in range(n):
            vals = []
            for k in range(-window, window + 1):
                vals.append(curv[(i + k) % n])
            smooth[i] = max(vals)

        self.curvature = smooth

    def get_lookahead_from_speed(self, speed):
        for speed_threshold, lookahead in LOOKAHEAD_TABLE:
            if speed <= speed_threshold:
                return lookahead

        return LOOKAHEAD_TABLE[-1][1]

    def adjust_lookahead_for_curvature(self, lookahead, current_kappa, future_kappa):
        max_kappa = max(current_kappa, future_kappa)

        if max_kappa > LOOKAHEAD_CURV_REDUCE_THRESHOLD:
            ratio = (max_kappa - LOOKAHEAD_CURV_REDUCE_THRESHOLD) / (
                0.20 - LOOKAHEAD_CURV_REDUCE_THRESHOLD
            )
            ratio = min(1.0, max(0.0, ratio))

            factor = 1.0 - (1.0 - LOOKAHEAD_CURV_MIN_FACTOR) * ratio
            return lookahead * factor

        return lookahead

    def get_future_max_curvature(self, start_idx, distance_ahead):
        n = len(self.waypoints_x)

        if n == 0:
            return 0.0

        start_s = self.waypoints_s[start_idx]
        max_kappa = 0.0

        for offset in range(n):
            idx = (start_idx + offset) % n

            if self.waypoints_s[idx] >= start_s:
                ds = self.waypoints_s[idx] - start_s
            else:
                ds = (self.total_length - start_s) + self.waypoints_s[idx]

            if ds > distance_ahead:
                break

            max_kappa = max(max_kappa, float(self.curvature[idx]))

        return max_kappa

    def pose_callback(self, odometry_msg):
        if len(self.waypoints_x) == 0:
            self.publish_drive(0.0, 0.0)
            return

        self.callback_count += 1

        self.pos_x = odometry_msg.pose.pose.position.x
        self.pos_y = odometry_msg.pose.pose.position.y

        q = odometry_msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.heading_angle = math.atan2(siny_cosp, cosy_cosp)

        nearest_idx = self.find_nearest_index()

        current_kappa = float(self.curvature[nearest_idx])
        future_kappa = self.get_future_max_curvature(
            nearest_idx,
            FUTURE_CURVE_DISTANCE
        )
        max_kappa = max(current_kappa, future_kappa)

        target_speed = float(self.waypoints_speed[nearest_idx])

        if max_kappa > CURV_TIER_4:
            target_speed = min(target_speed, CURV_SPEED_4)
        elif max_kappa > CURV_TIER_3:
            target_speed = min(target_speed, CURV_SPEED_3)
        elif max_kappa > CURV_TIER_2:
            target_speed = min(target_speed, CURV_SPEED_2)
        elif max_kappa > CURV_TIER_1:
            target_speed = min(target_speed, CURV_SPEED_1)

        lookahead = self.get_lookahead_from_speed(target_speed)
        lookahead = self.adjust_lookahead_for_curvature(
            lookahead,
            current_kappa,
            future_kappa
        )

        target_idx = self.find_forward_target_index(nearest_idx, lookahead)

        self.target_idx = target_idx
        self.target_x = float(self.waypoints_x[target_idx])
        self.target_y = float(self.waypoints_y[target_idx])

        steering_angle = self.compute_steering(self.target_x, self.target_y)

        speed = self.compute_speed(target_speed, steering_angle)

        steering_angle = STEER_ALPHA * steering_angle + (1.0 - STEER_ALPHA) * self.prev_steer

        if speed >= self.prev_speed:
            speed = SPEED_ALPHA_ACCEL * speed + (1.0 - SPEED_ALPHA_ACCEL) * self.prev_speed

        self.prev_steer = steering_angle
        self.prev_speed = speed

        self.publish_drive(speed, steering_angle)
        self.publish_markers()

    def find_nearest_index(self):
        n = len(self.waypoints_x)

        if n == 0:
            return 0

        if not self.initialized_nearest:
            dx = self.waypoints_x - self.pos_x
            dy = self.waypoints_y - self.pos_y
            dists = dx * dx + dy * dy

            nearest_idx = int(np.argmin(dists))
            self.prev_nearest_idx = nearest_idx
            self.initialized_nearest = True

            return nearest_idx

        candidates = np.array([
            (self.prev_nearest_idx + k) % n
            for k in range(-NEAREST_SEARCH_WINDOW // 2, NEAREST_SEARCH_WINDOW // 2 + 1)
        ])

        dx = self.waypoints_x[candidates] - self.pos_x
        dy = self.waypoints_y[candidates] - self.pos_y
        dists = dx * dx + dy * dy

        nearest_local = int(np.argmin(dists))
        nearest_idx = int(candidates[nearest_local])

        self.prev_nearest_idx = nearest_idx

        return nearest_idx

    def find_forward_target_index(self, nearest_idx, lookahead):
        n = len(self.waypoints_x)

        if n == 0:
            return 0

        start_s = self.waypoints_s[nearest_idx]

        for offset in range(n):
            idx = (nearest_idx + offset) % n

            if self.waypoints_s[idx] >= start_s:
                cur_s = self.waypoints_s[idx]
            else:
                cur_s = self.waypoints_s[idx] + self.total_length

            if cur_s - start_s >= lookahead:
                return idx

        return nearest_idx

    def compute_steering(self, tx, ty):
        dx = tx - self.pos_x
        dy = ty - self.pos_y

        local_x = dx * math.cos(self.heading_angle) + dy * math.sin(self.heading_angle)
        local_y = -dx * math.sin(self.heading_angle) + dy * math.cos(self.heading_angle)

        L = math.sqrt(local_x * local_x + local_y * local_y)

        if L < 1e-6:
            return 0.0

        curvature = 2.0 * local_y / (L * L)
        steering = math.atan(WHEELBASE * curvature * STEERING_GAIN)

        return float(np.clip(steering, -MAX_STEER, MAX_STEER))

    def compute_speed(self, target_speed, steering_angle):
        abs_steer_deg = abs(steering_angle) * 180.0 / math.pi

        speed = target_speed

        if abs_steer_deg > STEER_DEG_4:
            speed = min(speed, STEER_SPEED_4)
        elif abs_steer_deg > STEER_DEG_3:
            speed = min(speed, STEER_SPEED_3)
        elif abs_steer_deg > STEER_DEG_2:
            speed = min(speed, STEER_SPEED_2)
        elif abs_steer_deg > STEER_DEG_1:
            speed = min(speed, STEER_SPEED_1)

        return float(np.clip(speed, MIN_SPEED, MAX_SPEED))

    def publish_drive(self, speed, steering_angle):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(speed)

        self.drive_pub.publish(drive_msg)

        if self.callback_count % 20 == 0:
            self.get_logger().info(
                f"idx={self.target_idx}, "
                f"steer={steering_angle * 180.0 / np.pi:.1f} deg, "
                f"speed={speed:.2f} m/s, "
                f"kappa={self.curvature[self.prev_nearest_idx]:.3f}"
            )

    def publish_markers(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.color.a = 1.0
        marker.color.b = 1.0

        marker.points = [
            Point(x=float(x), y=float(y), z=0.0)
            for x, y in zip(self.waypoints_x, self.waypoints_y)
        ]

        self.waypoints_marker_pub.publish(marker)

        target_marker = Marker()
        target_marker.header.frame_id = "map"
        target_marker.header.stamp = self.get_clock().now().to_msg()
        target_marker.ns = "target"
        target_marker.id = 1
        target_marker.type = Marker.POINTS
        target_marker.action = Marker.ADD
        target_marker.scale.x = 0.25
        target_marker.scale.y = 0.25
        target_marker.color.a = 1.0
        target_marker.color.r = 1.0

        target_marker.points = [
            Point(x=float(self.target_x), y=float(self.target_y), z=0.0)
        ]

        self.target_marker_pub.publish(target_marker)


def main(args=None):
    rclpy.init(args=args)

    print("Pure Pursuit - parameters gathered at top")

    node = PurePursuit()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()