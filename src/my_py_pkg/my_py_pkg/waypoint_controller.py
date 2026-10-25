import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

WAYPOINT_MAP = {
    'HOME': (0.0, 0.0),
    'A': (1.0, 0.0),
    'B': (1.0, 1.5),
    'C': (0.0, 1.5),
}

# 設備のGazebo上の中心座標。設備前面への視線を生成する。
EQUIPMENT_MAP = {
    'A': (2.35, 0.0),
    'B': (2.35, 1.5),
}


class WaypointController(Node):
    def __init__(self):
        super().__init__('waypoint_controller')
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False
        self.last_odom_time = None
        self.state = 'IDLE'  # IDLE, RUNNING, COMPLETED, ABORTED
        self.phase = 'NAVIGATING'  # NAVIGATING, ALIGNING, WAITING_CAPTURE
        self.waypoints = []
        self.current_index = 0
        self.start_time = None
        self.waypoint_start_time = None
        self.phase_start_time = None
        self.aligned_since = None
        self.goal_tolerance = 0.10
        self.max_linear_speed = 0.15
        self.max_angular_speed = 0.5
        self.odom_timeout = 3.0
        self.waypoint_timeout = 120.0
        self.align_timeout = 20.0
        self.yaw_tolerance = math.radians(5.0)
        self.settle_seconds = 0.5
        self.capture_timeout = 5.0
        self.max_capture_attempts = 3
        self.capture_attempts = 0
        self.capture_sent_at = None

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.plan_sub = self.create_subscription(
            String, '/inspection/plan', self.plan_callback, 10)
        self.capture_done_sub = self.create_subscription(
            String, '/inspection/capture_done', self.capture_done_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/inspection/status', 10)
        # Step 7-2の撮影ノードがこのイベントを受信する。
        self.capture_pub = self.create_publisher(
            String, '/inspection/capture_request', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('AI Waypoint Controller Step 7 started')
        self.get_logger().info('Waiting for AI plan...')
        self.publish_status('IDLE')

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        self.odom_received = True
        self.last_odom_time = time.monotonic()

    def plan_callback(self, msg):
        if self.state == 'RUNNING':
            self.get_logger().warning('Plan rejected: task already running')
            return
        if self.state != 'IDLE':
            self.get_logger().warning(
                f'Plan rejected: state={self.state}. Restart controller for a new task.')
            return
        now = time.monotonic()
        if (not self.odom_received or self.last_odom_time is None
                or now - self.last_odom_time > self.odom_timeout):
            self.get_logger().error('Plan rejected: fresh odometry unavailable')
            self.publish_status('REJECTED: NO_ODOMETRY')
            return
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or set(data.keys()) != {'waypoints'}:
                raise ValueError('Plan must contain only waypoints')
            names = data['waypoints']
            if not isinstance(names, list) or not 1 <= len(names) <= 4:
                raise ValueError('Invalid waypoint list')
            for name in names:
                if not isinstance(name, str) or name not in WAYPOINT_MAP:
                    raise ValueError(f'Unknown waypoint: {name}')
            self.waypoints = [(name, *WAYPOINT_MAP[name]) for name in names]
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().error(f'Invalid AI plan: {exc}')
            self.publish_status('REJECTED: INVALID_PLAN')
            self.stop_robot()
            return

        self.current_index = 0
        self.start_time = now
        self.waypoint_start_time = now
        self.phase_start_time = now
        self.aligned_since = None
        self.phase = 'NAVIGATING'
        self.capture_attempts = 0
        self.capture_sent_at = None
        self.state = 'RUNNING'
        self.get_logger().info(f'AI plan accepted: {names}')
        self.publish_status(f'RUNNING: {" -> ".join(names)}')

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def abort_task(self, reason):
        self.stop_robot()
        self.state = 'ABORTED'
        self.get_logger().error(f'Task aborted: {reason}')
        self.publish_status(f'ABORTED: {reason}')

    @staticmethod
    def wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def request_capture(self, name):
        # 撮影結果を待つ間は必ず停止する。
        self.stop_robot()
        self.capture_attempts += 1
        self.phase = 'WAITING_CAPTURE'
        self.capture_sent_at = time.monotonic()
        request = String()
        request.data = json.dumps({
            'equipment': name,
            'frame_id': 'camera_link_optical',
        })
        self.capture_pub.publish(request)
        self.get_logger().info(
            f'Capture requested: equipment {name} '
            f'(attempt {self.capture_attempts}/{self.max_capture_attempts})')
        self.publish_status(f'WAITING_CAPTURE: {name}')

    def capture_done_callback(self, msg):
        # 無関係な通知や、移動中の古い通知は無視する。
        if self.state != 'RUNNING' or self.phase != 'WAITING_CAPTURE':
            return
        name = self.waypoints[self.current_index][0]
        try:
            result = json.loads(msg.data)
            if not isinstance(result, dict):
                raise ValueError('result must be an object')
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f'Ignoring invalid capture_done: {exc}')
            return
        if result.get('equipment') != name:
            self.get_logger().warning(
                f'Ignoring capture_done for {result.get("equipment")!r}; '
                f'waiting for {name}')
            return
        if result.get('success') is True:
            image_path = result.get('image_path')
            if not isinstance(image_path, str) or not image_path:
                self.get_logger().warning('Capture result missing image_path')
                return
            self.get_logger().info(f'Capture completed: equipment {name}: {image_path}')
            self.publish_status(f'CAPTURED: {name}')
            self.advance_waypoint(name)
        elif result.get('success') is False:
            reason = str(result.get('error', 'UNKNOWN'))
            self.get_logger().warning(f'Capture failed: equipment {name}: {reason}')
            self.retry_capture_or_abort(name, reason)
        else:
            self.get_logger().warning('Ignoring capture_done without boolean success')

    def retry_capture_or_abort(self, name, reason):
        if self.capture_attempts >= self.max_capture_attempts:
            self.abort_task(f'CAPTURE_FAILED: {name}: {reason}')
            return
        self.request_capture(name)

    def advance_waypoint(self, name):
        self.stop_robot()
        self.current_index += 1
        self.capture_attempts = 0
        self.capture_sent_at = None
        if self.current_index >= len(self.waypoints):
            self.state = 'COMPLETED'
            elapsed = time.monotonic() - self.start_time
            self.get_logger().info('ALL WAYPOINTS REACHED!')
            self.get_logger().info(f'Elapsed time: {elapsed:.2f}s')
            self.publish_status(f'COMPLETED: {elapsed:.2f}s')
            return
        self.waypoint_start_time = time.monotonic()
        self.phase_start_time = self.waypoint_start_time
        self.aligned_since = None
        self.phase = 'NAVIGATING'
        next_name = self.waypoints[self.current_index][0]
        self.get_logger().info(f'Moving to next waypoint: {next_name}')
        self.publish_status(f'RUNNING: {next_name}')

    def control_loop(self):
        if self.state != 'RUNNING':
            self.stop_robot()
            return
        now = time.monotonic()
        if (not self.odom_received or self.last_odom_time is None
                or now - self.last_odom_time > self.odom_timeout):
            self.abort_task('ODOMETRY_TIMEOUT')
            return

        name, target_x, target_y = self.waypoints[self.current_index]
        if self.phase == 'WAITING_CAPTURE':
            self.stop_robot()
            if (self.capture_sent_at is not None
                    and now - self.capture_sent_at > self.capture_timeout):
                self.get_logger().warning(f'Capture timeout: equipment {name}')
                self.retry_capture_or_abort(name, 'TIMEOUT')
            return
        if self.phase == 'NAVIGATING':
            if now - self.waypoint_start_time > self.waypoint_timeout:
                self.abort_task('WAYPOINT_TIMEOUT')
                return
            dx = target_x - self.x
            dy = target_y - self.y
            distance = math.hypot(dx, dy)
            if distance < self.goal_tolerance:
                self.stop_robot()
                self.get_logger().info(
                    f'Waypoint {name} reached: x={self.x:.3f}, '
                    f'y={self.y:.3f}, error={distance:.3f}m')
                self.publish_status(f'REACHED: {name}')
                if name not in EQUIPMENT_MAP:
                    self.advance_waypoint(name)
                else:
                    self.phase = 'ALIGNING'
                    self.phase_start_time = now
                    self.aligned_since = None
                    self.get_logger().info(f'Aligning camera toward equipment {name}')
                    self.publish_status(f'ALIGNING: {name}')
                return
            target_angle = math.atan2(dy, dx)
            angle_error = self.wrap_angle(target_angle - self.yaw)
            cmd = Twist()
            if abs(angle_error) <= 0.2:
                cmd.linear.x = min(self.max_linear_speed, 0.5 * distance)
            cmd.angular.z = max(
                -self.max_angular_speed,
                min(self.max_angular_speed, 1.2 * angle_error))
            self.cmd_pub.publish(cmd)
            return

        if self.phase == 'ALIGNING':
            if now - self.phase_start_time > self.align_timeout:
                self.abort_task(f'ALIGN_TIMEOUT: {name}')
                return
            equip_x, equip_y = EQUIPMENT_MAP[name]
            desired_yaw = math.atan2(equip_y - self.y, equip_x - self.x)
            error = self.wrap_angle(desired_yaw - self.yaw)
            cmd = Twist()
            if abs(error) <= self.yaw_tolerance:
                self.stop_robot()
                if self.aligned_since is None:
                    self.aligned_since = now
                if now - self.aligned_since >= self.settle_seconds:
                    self.get_logger().info(
                        f'Equipment {name} aligned: yaw_error={math.degrees(error):.2f}deg')
                    self.request_capture(name)
                return
            self.aligned_since = None
            cmd.angular.z = max(
                -self.max_angular_speed,
                min(self.max_angular_speed, 1.2 * error))
            # 最小旋回速度で動摩擦・コントローラ不感帯を超える
            if abs(cmd.angular.z) < 0.08:
                cmd.angular.z = math.copysign(0.08, error)
            self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
