import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
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
        # 파라미터
        # =============================================================

        self.max_range = 6.0
        self.front_deg = 95
        self.smoothing_window = 3

        self.disparity_threshold = 0.20
        self.car_width = 0.35
        self.extra_safety_margin = 0.10

        self.speed_straight = 4.0
        self.speed_corner = 1.4
        self.max_steer_for_speed = 22.0
        self.angle_speed_penalty = 0.95

        self.emergency_distance = 0.85
        self.emergency_speed = 0.8
        self.emergency_fov_deg = 5.0

        self.prev_steering = 0.0
        self.steering_smooth_alpha = 0.35

        self.corner_min_steer = np.radians(6.0)
        self.corner_side_dist = 0.24

        # =============================================================
        # 좌측 L자 막다른길 패턴 감지
        # "좌측이 가깝고 / 중간도 막혀 있고 / 우측이 멀다"면
        # 우측으로 bias
        # =============================================================
        self.l_deadend_left_close_thresh = 0.80
        self.l_deadend_mid_close_thresh = 1.20
        self.l_deadend_right_far_thresh = 1.50
        self.l_deadend_diff_thresh = 0.60

        self.l_deadend_counter = 0
        self.l_deadend_counter_trigger = 2

        self.l_deadend_right_bias_deg = 12.0
        self.l_deadend_speed_limit = 1.8

        self.debug_log = True

    # =============================================================
    # 마커
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
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
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
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
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
    # 기본 유틸
    # =============================================================

    def preprocess_lidar(self, ranges):
        proc = np.array(ranges, dtype=np.float32)
        proc[np.isnan(proc)] = 0.0
        proc[np.isinf(proc)] = self.max_range
        proc = np.clip(proc, 0.0, self.max_range)

        if self.smoothing_window > 1:
            kernel = np.ones(self.smoothing_window, dtype=np.float32) / float(self.smoothing_window)
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

    def angle_to_index(self, data, angle_deg):
        angle_rad = np.deg2rad(angle_deg)
        idx = int(round((angle_rad - data.angle_min) / data.angle_increment))
        return int(np.clip(idx, 0, len(data.ranges) - 1))

    def get_mean_range_window(self, proc_ranges, data, center_deg, half_width_deg):
        c_idx = self.angle_to_index(data, center_deg)
        hw = int(round(np.deg2rad(half_width_deg) / data.angle_increment))
        s = max(0, c_idx - hw)
        e = min(len(proc_ranges) - 1, c_idx + hw)

        vals = proc_ranges[s:e + 1]
        vals = vals[vals > 0.01]
        if len(vals) == 0:
            return 0.0
        return float(np.mean(vals))

    def get_num_points_to_cover(self, dist, angle_increment):
        width_to_cover = (self.car_width / 2.0) + self.extra_safety_margin
        if dist <= 1e-3:
            return 0

        arc_angle = 2.0 * np.arcsin(np.clip(width_to_cover / (2.0 * dist), 0.0, 1.0))
        num_points = int(np.ceil(arc_angle / angle_increment))
        return max(1, num_points)

    # =============================================================
    # Disparity Extender
    # =============================================================

    def extend_disparities(self, ranges, angle_increment):
        new_ranges = np.copy(ranges)

        for i in range(len(ranges) - 1):
            diff = abs(ranges[i + 1] - ranges[i])
            if diff > self.disparity_threshold:
                if ranges[i] < ranges[i + 1]:
                    close_dist = ranges[i]
                    num_points = self.get_num_points_to_cover(close_dist, angle_increment)
                    start = i + 1
                    end = min(len(new_ranges), i + 1 + num_points)
                    new_ranges[start:end] = np.minimum(new_ranges[start:end], close_dist)
                else:
                    close_dist = ranges[i + 1]
                    num_points = self.get_num_points_to_cover(close_dist, angle_increment)
                    start = max(0, i - num_points + 1)
                    end = i + 1
                    new_ranges[start:end] = np.minimum(new_ranges[start:end], close_dist)

        return new_ranges

    def find_best_point(self, ranges):
        return int(np.argmax(ranges))

    # =============================================================
    # L자 패턴 감지
    # =============================================================

    def detect_left_L_deadend(self, proc_ranges, data):
        """
        반시계 방향 기준:
        좌측이 가깝고, 중간이 튀어나와 있고, 우측이 멀면
        좌측 L자 막다른길 패턴으로 보고 우측 bias를 넣는다.
        """

        left_close = self.get_mean_range_window(proc_ranges, data, 55.0, 10.0)
        mid_close = self.get_mean_range_window(proc_ranges, data, 15.0, 12.0)
        right_far = self.get_mean_range_window(proc_ranges, data, -40.0, 12.0)

        cond = (
            left_close < self.l_deadend_left_close_thresh and
            mid_close < self.l_deadend_mid_close_thresh and
            right_far > self.l_deadend_right_far_thresh and
            (right_far - left_close) > self.l_deadend_diff_thresh
        )

        if cond:
            self.l_deadend_counter += 1
        else:
            self.l_deadend_counter = 0

        detected = self.l_deadend_counter >= self.l_deadend_counter_trigger
        return detected, left_close, mid_close, right_far

    # =============================================================
    # 안전 / 조향 / 속도
    # =============================================================

    def smooth_steering(self, raw_steering):
        smoothed = (
            self.steering_smooth_alpha * raw_steering +
            (1.0 - self.steering_smooth_alpha) * self.prev_steering
        )
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
                valid = proc[left_side][proc[left_side] > 0.01]
                if len(valid) > 0:
                    min_dist = np.min(valid)
                    if min_dist < self.corner_side_dist:
                        ratio = np.clip(min_dist / self.corner_side_dist, 0.1, 1.0)
                        return steering_angle * ratio

        if steering_angle < -self.corner_min_steer:
            if len(right_side) > 0:
                valid = proc[right_side][proc[right_side] > 0.01]
                if len(valid) > 0:
                    min_dist = np.min(valid)
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

    # =============================================================
    # 메인
    # =============================================================

    def scan_callback(self, data):
        raw_ranges = np.array(data.ranges, dtype=np.float32)

        # 1) 전처리
        proc_ranges = self.preprocess_lidar(raw_ranges)

        # 2) emergency 체크
        is_emergency = self.check_emergency(data, proc_ranges)

        # 3) 앞쪽 FOV 추출
        start_idx, end_idx = self.get_front_indices(data)
        front_ranges = np.copy(proc_ranges[start_idx:end_idx + 1])

        # 4) safe range 생성
        safe_ranges = self.extend_disparities(front_ranges, data.angle_increment)
        safe_ranges[safe_ranges < 0.05] = 0.0

        # 5) 가장 먼 점 선택
        best_local = self.find_best_point(safe_ranges)
        best_global = start_idx + best_local
        best_distance = float(safe_ranges[best_local])
        steering_angle = data.angle_min + best_global * data.angle_increment

        mode = "GAP"

        # 6) 좌측 L자 막다른길 패턴 감지 -> 우측 bias
        left_L_detected, left_l_close, left_l_mid, left_l_right_far = self.detect_left_L_deadend(proc_ranges, data)

        if left_L_detected:
            steering_angle -= np.radians(self.l_deadend_right_bias_deg)
            mode = "LEFT_L_DEADEND_RIGHT_BIAS"

        # 7) 옆벽 안전
        steering_angle = self.corner_safety_override(data, steering_angle)

        # 8) 조향 스무딩
        steering_angle = self.smooth_steering(steering_angle)

        # 9) 속도
        speed = self.calc_speed(steering_angle, best_distance, is_emergency)

        if left_L_detected:
            speed = min(speed, self.l_deadend_speed_limit)

        # 10) publish
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(speed)
        self.drive_pub.publish(drive_msg)

        # 11) 로그
        if self.debug_log:
            status = " [EMERGENCY]" if is_emergency else ""
            self.get_logger().info(
                f"mode={mode}, "
                f"steer_deg={np.degrees(steering_angle):.1f}, "
                f"speed={speed:.2f}, "
                f"best_dist={best_distance:.2f}, "
                f"left_L_detected={left_L_detected}, "
                f"leftL_close={left_l_close:.2f}, "
                f"leftL_mid={left_l_mid:.2f}, "
                f"leftL_right_far={left_l_right_far:.2f}, "
                f"counter={self.l_deadend_counter}"
                f"{status}"
            )

        # 12) 마커
        self.publish_best_point_marker(
            best_global, best_distance, data.angle_min, data.angle_increment
        )

        nonzero = np.where(front_ranges > 1e-3)[0]
        if len(nonzero) > 0:
            nearest_local = nonzero[np.argmin(front_ranges[nonzero])]
            nearest_global = start_idx + nearest_local
            nearest_dist = float(front_ranges[nearest_local])
        else:
            nearest_global = start_idx + len(front_ranges) // 2
            nearest_dist = 0.0

        bubble_radius = (self.car_width / 2.0) + self.extra_safety_margin
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