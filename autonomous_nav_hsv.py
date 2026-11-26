"""Autonomous navigation to a coloured target panel.

State machine:  SEARCH -> ALIGN -> APPROACH -> AVOID -> ARRIVED

  camera  : HSV threshold finds the panel; its apparent width gives the distance
  LiDAR   : gap finding, picks the free direction closest to the target
  pose    : /odometry/filtered from the robot_localization EKF (wheel odometry + IMU)
"""
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

# ── Target detection (HSV) ───────────────────────────────────
HSV_LOW = (2, 120, 90)
HSV_HIGH = (22, 255, 255)
MIN_AREA_PX = 60
PANEL_WIDTH = 0.32                      # m
IMG_WIDTH = 640
FOCAL_PX = (IMG_WIDTH / 2) / math.tan(1.3962634 / 2)
CAMERA_OFFSET = 0.20                    # camera is 20 cm ahead of the chassis centre

# ── Motion ───────────────────────────────────────────────────
LINEAR_SPEED = 0.30
SEARCH_SPEED = 0.5
KP_PIXEL = 0.004
KP_HEADING = 1.6
MAX_ANGULAR = 0.9
CENTER_TOLERANCE = 25                   # px
ARRIVE_DISTANCE = 0.75                  # m, chassis centre to panel

# ── Obstacle avoidance (gap finding) ─────────────────────────
ROBOT_HALF_WIDTH = 0.24                 # m, with margin
LOOKAHEAD_APPROACH = 0.60               # m, corridor that must stay free while approaching
LOOKAHEAD_GAP = 1.00                    # m, a direction is a "gap" if free this far
LOOKAHEAD_RESUME = 1.30                 # m, free this far towards the target: resume
CANDIDATES = np.radians(np.arange(-130, 131, 5))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class AutonomousNav(Node):

    def __init__(self):
        super().__init__('autonomous_nav')
        self.bridge = CvBridge()
        self.state = 'SEARCH'
        self.pose = None                 # x, y, yaw from the EKF
        self.points = np.zeros((0, 2))   # LiDAR points in the robot frame
        self.pixel_error = None          # target centre minus image centre, px
        self.target = None               # estimated target position in the odom frame
        self.gap = 0.0

        self.cmd_pub = self.create_publisher(Twist, '/model/my_robot/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/nav/state', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/nav/markers', 10)
        self.image_pub = self.create_publisher(Image, '/camera/annotated', 2)

        self.create_subscription(Image, '/camera/image', self.image_cb, 2)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 5)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('state: SEARCH')

    # ── Sensors ──────────────────────────────────────────────
    def odom_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def scan_cb(self, msg):
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = np.isfinite(r) & (r > msg.range_min) & (r < 4.0)
        self.points = np.column_stack((r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])))

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), HSV_LOW, HSV_HIGH)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = max(contours, key=cv2.contourArea) if contours else None
        if best is not None and cv2.contourArea(best) >= MIN_AREA_PX:
            x, y, w, h = cv2.boundingRect(best)
            self.pixel_error = x + w / 2 - IMG_WIDTH / 2
            touches_edge = x <= 1 or x + w >= IMG_WIDTH - 1
            if self.pose is not None and not touches_edge:
                distance = PANEL_WIDTH * FOCAL_PX / w + CAMERA_OFFSET
                bearing = -math.atan2(self.pixel_error, FOCAL_PX)
                px, py, yaw = self.pose
                self.target = (px + distance * math.cos(yaw + bearing),
                               py + distance * math.sin(yaw + bearing))
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        else:
            self.pixel_error = None
        cv2.putText(frame, self.state, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
        cv2.putText(frame, self.state, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out.header = msg.header
        self.image_pub.publish(out)

    # ── Geometry helpers ─────────────────────────────────────
    def is_free(self, direction, distance):
        """True if the robot can drive `distance` metres in `direction` (robot frame)."""
        if len(self.points) == 0:
            return True
        ux, uy = math.cos(direction), math.sin(direction)
        along = self.points[:, 0] * ux + self.points[:, 1] * uy
        lateral = np.abs(self.points[:, 0] * uy - self.points[:, 1] * ux)
        return not np.any((along > 0.0) & (along < distance) & (lateral < ROBOT_HALF_WIDTH))

    def target_polar(self):
        """Distance and bearing of the estimated target in the robot frame."""
        x, y, yaw = self.pose
        dx, dy = self.target[0] - x, self.target[1] - y
        return math.hypot(dx, dy), wrap(math.atan2(dy, dx) - yaw)

    def find_gap(self, wanted):
        """Free direction closest to the wanted one (with a little hysteresis)."""
        free = [c for c in CANDIDATES if self.is_free(c, LOOKAHEAD_GAP)]
        if not free:
            return None
        return min(free, key=lambda c: abs(wrap(c - wanted)) + 0.3 * abs(wrap(c - self.gap)))

    # ── Control loop, 10 Hz ──────────────────────────────────
    def control_loop(self):
        cmd = Twist()
        if self.pose is None:
            return
        err = self.pixel_error
        dist, bearing = self.target_polar() if self.target else (None, None)

        # Steering error: from the camera while the panel is visible, otherwise from
        # the remembered target position and the EKF pose (low walls hide the panel up close).
        if err is not None:
            steer = -KP_PIXEL * err
            centred = abs(err) < CENTER_TOLERANCE
        elif bearing is not None:
            steer = KP_HEADING * bearing
            centred = abs(bearing) < 0.08
        else:
            steer, centred = None, False

        if self.state == 'SEARCH':
            if steer is not None:
                self.set_state('ALIGN')
            else:
                cmd.angular.z = SEARCH_SPEED

        elif self.state == 'ALIGN':
            if centred:
                self.set_state('APPROACH')
            else:
                cmd.angular.z = float(np.clip(steer, -MAX_ANGULAR, MAX_ANGULAR))

        elif self.state == 'APPROACH':
            if dist is None:
                cmd.angular.z = float(np.clip(steer, -MAX_ANGULAR, MAX_ANGULAR))
            elif dist < ARRIVE_DISTANCE:
                self.set_state('ARRIVED')
            elif not self.is_free(bearing, min(LOOKAHEAD_APPROACH, dist - 0.3)):
                self.gap = bearing
                self.set_state('AVOID')
            else:
                cmd.angular.z = float(np.clip(steer, -MAX_ANGULAR, MAX_ANGULAR))
                cmd.linear.x = LINEAR_SPEED * max(0.4, 1.0 - abs(steer) / 0.8)

        elif self.state == 'AVOID':
            if self.is_free(bearing, min(LOOKAHEAD_RESUME, dist - 0.3)):
                self.set_state('ALIGN')
            else:
                gap = self.find_gap(bearing)
                if gap is None:
                    cmd.angular.z = SEARCH_SPEED          # boxed in: turn on the spot
                else:
                    self.gap = gap
                    cmd.angular.z = float(np.clip(KP_HEADING * gap, -MAX_ANGULAR, MAX_ANGULAR))
                    cmd.linear.x = 0.25 if abs(gap) < 0.35 else 0.0

        # ARRIVED: stay put
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(String(data=self.state))
        self.publish_markers()

    def set_state(self, new_state):
        if new_state != self.state:
            self.state = new_state
            self.get_logger().info(f'state: {new_state}')

    # ── RViz markers ─────────────────────────────────────────
    def publish_markers(self):
        arr = MarkerArray()
        text = Marker()
        text.header.frame_id = 'my_robot/odom'
        text.ns, text.id, text.type = 'state', 0, Marker.TEXT_VIEW_FACING
        text.pose.position.x, text.pose.position.y, text.pose.position.z = 2.5, 0.95, 0.5
        text.pose.orientation.w = 1.0
        text.scale.z = 0.32
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = self.state
        arr.markers.append(text)
        if self.target:
            ball = Marker()
            ball.header.frame_id = 'my_robot/odom'
            ball.ns, ball.id, ball.type = 'target', 1, Marker.SPHERE
            ball.pose.position.x, ball.pose.position.y = self.target
            ball.pose.orientation.w = 1.0
            ball.scale.x = ball.scale.y = ball.scale.z = 0.22
            ball.color.r, ball.color.g, ball.color.b, ball.color.a = 1.0, 0.85, 0.1, 1.0
            arr.markers.append(ball)
        arrow = Marker()
        arrow.header.frame_id = 'my_robot/chassis'
        arrow.ns, arrow.id, arrow.type = 'gap', 2, Marker.ARROW
        arrow.pose.orientation.w = 1.0
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.04, 0.10, 0.12
        arrow.color.r, arrow.color.g, arrow.color.b = 1.0, 1.0, 1.0
        arrow.color.a = 1.0 if self.state == 'AVOID' else 0.0
        arrow.points = [Point(x=0.0, y=0.0, z=0.1),
                        Point(x=0.7 * math.cos(self.gap), y=0.7 * math.sin(self.gap), z=0.1)]
        arr.markers.append(arrow)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
