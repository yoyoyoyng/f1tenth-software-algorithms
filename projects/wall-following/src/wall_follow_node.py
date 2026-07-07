#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32
import math

class WallFollow(Node):
    def __init__(self):
        super().__init__('wall_follow_node')

        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(Float32, '/threshold/front', self.cb_front, 10)

        self.kp = 1.0
        self.kd = 0.4
        self.ki = 0.0

        self.desired_distance_right = 1.2
        self.lookahead_distance = 1.0

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = self.get_clock().now()

        self.FRONT_THRESHOLD = 3.0  # 전방이 이것보다 멀면 교차로 후보
        self.in_intersection = False

    def cb_front(self, msg):
        self.FRONT_THRESHOLD = msg.data

    def get_range(self, range_data, angle):
        scan = range_data
        angle = max(scan.angle_min, min(angle, scan.angle_max))
        i = int(round((angle - scan.angle_min) / scan.angle_increment))
        i = max(0, min(i, len(scan.ranges) - 1))
        r = scan.ranges[i]
        if r is None or not math.isfinite(r) or r <= 0.0:
            return scan.range_max if scan.range_max > 0.0 else 100.0
        if scan.range_min > 0.0 and r < scan.range_min:
            r = scan.range_min
        if scan.range_max > 0.0 and r > scan.range_max:
            r = scan.range_max
        return float(r)

    def get_error(self, range_data, dist):
        angle_b = math.radians(-80.0)
        angle_a = math.radians(-35.0)

        b = self.get_range(range_data, angle_b)
        a = self.get_range(range_data, angle_a)

        theta = angle_a - angle_b
        k = a * math.sin(theta)

        alpha = 0.0 if abs(k) < 1e-6 else math.atan((a * math.cos(theta) - b) / k)

        D_t = b * math.cos(alpha)
        D_t_1 = D_t + self.lookahead_distance * math.sin(alpha)

        return float(dist - D_t_1)

    def pid_control(self, error):
        current_time = self.get_clock().now()
        delta_time = (current_time - self.prev_time).nanoseconds / 1e9
        if delta_time <= 0.0:
            delta_time = 1e-3

        d_error = (error - self.prev_error) / delta_time
        self.integral += error * delta_time

        steering_angle = self.kp * error + self.ki * self.integral + self.kd * d_error
        max_steer = math.radians(35.0)
        steering_angle = max(-max_steer, min(max_steer, steering_angle))

        abs_steer = abs(steering_angle)
        if abs_steer > math.radians(15):
            velocity = 2.6
        elif abs_steer > math.radians(7):
            velocity = 3.6
        else:
            velocity = 6.95

        self.publish_drive(steering_angle, velocity)
        self.prev_error = error
        self.prev_time = current_time

    def publish_drive(self, steer, speed):
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = float(steer)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)

    def scan_callback(self, msg):
        right_85 = self.get_range(msg, math.radians(-85.0))
        front    = self.get_range(msg, math.radians(0.0))

        self.get_logger().info(f'right_85={right_85:.2f} front={front:.2f}')

        # ── 1순위: 교차로 (앞 멀고 오른쪽 85도가 2.0 이하) ──────
        if front > self.FRONT_THRESHOLD and 1.3< right_85 < 1.8:
            self.get_logger().info(f'[INTERSECTION] right_85={right_85:.2f} front={front:.2f}')
            self.publish_drive(+0.06, 6.5)
            return

        # ── 2순위: 그냥 PID ───────────────────────────────────────
        self.pid_control(self.get_error(msg, self.desired_distance_right))


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)
    wall_follow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
