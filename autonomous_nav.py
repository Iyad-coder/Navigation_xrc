import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading

# ── Paramètres ──────────────────────────────────────────────
LINEAR_SPEED      = 0.3    # vitesse d'avance max (m/s)
ANGULAR_SPEED     = 0.4    # vitesse de rotation max (rad/s)
KP_ANGULAR        = 0.005  # gain proportionnel angulaire
KP_LINEAR         = 0.8    # gain proportionnel linéaire
TARGET_AREA       = 8000   # aire du QR (px²) quand on est assez proche
OBSTACLE_DIST     = 0.2    # distance obstacle (m) pour s'arrêter
SEARCH_SPEED      = 0.3    # vitesse rotation en mode recherche
IMG_WIDTH         = 640
IMG_HEIGHT        = 480
IMG_CENTER_X      = IMG_WIDTH // 2
CENTER_TOLERANCE  = 30     # pixels d'erreur acceptables
# ────────────────────────────────────────────────────────────

class AutonomousNav(Node):

    def __init__(self):
        super().__init__('autonomous_nav')

        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()

        # État de la machine
        self.state = 'SEARCH'
        self.qr_center_x = None
        self.qr_area = None
        self.obstacle_front = float('inf')

        # Locks
        self.image_lock = threading.Lock()
        self.scan_lock = threading.Lock()

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/model/my_robot/cmd_vel', 10)

        # Subscribers
        self.create_subscription(Image, '/camera/image', self.image_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Boucle de contrôle à 10 Hz
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Navigation autonome démarrée — état: SEARCH')

    # ── Callback caméra ─────────────────────────────────────
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        data, points, _ = self.qr_detector.detectAndDecode(frame)

        with self.image_lock:
            if points is not None and len(points) > 0:
                pts = points[0].astype(int)
                # Centre du QR code
                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))
                # Aire du QR code
                x, y, w, h = cv2.boundingRect(pts)
                area = w * h
                self.qr_center_x = cx
                self.qr_area = area
            else:
                self.qr_center_x = None
                self.qr_area = None

    # ── Callback LiDAR ──────────────────────────────────────
    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        # Secteur frontal : ±20° autour de 0
        front_indices = list(range(0, 20)) + list(range(340, 360))
        front_ranges = [ranges[i] for i in front_indices
                        if i < len(ranges) and not np.isnan(ranges[i])
                        and not np.isinf(ranges[i])]
        with self.scan_lock:
            self.obstacle_front = min(front_ranges) if front_ranges else float('inf')

    # ── Boucle de contrôle fermée ────────────────────────────
    def control_loop(self):
        with self.image_lock:
            qr_x = self.qr_center_x
            qr_area = self.qr_area
        with self.scan_lock:
            obstacle = self.obstacle_front

        cmd = Twist()

        # ── OBSTACLE : priorité absolue ──
        if obstacle < OBSTACLE_DIST:
            self.set_state('STOP_OBSTACLE')
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().warn(f'Obstacle à {obstacle:.2f}m — arrêt')
            return

        # ── MACHINE D'ÉTATS ──────────────────────────────────
        if self.state == 'SEARCH':
            if qr_x is not None:
                self.set_state('ALIGN')
            else:
                # Rotation lente pour chercher le QR
                cmd.angular.z = SEARCH_SPEED

        elif self.state == 'ALIGN':
            if qr_x is None:
                self.set_state('SEARCH')
            else:
                # Erreur de centrage (positive = QR à droite)
                error = qr_x - IMG_CENTER_X
                cmd.angular.z = -KP_ANGULAR * error

                if abs(error) < CENTER_TOLERANCE:
                    self.set_state('APPROACH')

        elif self.state == 'APPROACH':
            if qr_x is None:
                self.set_state('SEARCH')
            else:
                error = qr_x - IMG_CENTER_X

                # Correction angulaire en avançant
                cmd.angular.z = -KP_ANGULAR * error

                if qr_area is not None and qr_area >= TARGET_AREA:
                    self.set_state('ARRIVED')
                else:
                    # Vitesse linéaire proportionnelle à la distance estimée
                    if qr_area and qr_area > 0:
                        distance_factor = 1.0 - min(qr_area / TARGET_AREA, 1.0)
                        cmd.linear.x = LINEAR_SPEED * distance_factor
                    else:
                        cmd.linear.x = LINEAR_SPEED

        elif self.state == 'ARRIVED':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('QR code atteint — mission accomplie !')

        elif self.state == 'STOP_OBSTACLE':
            # Reprend si obstacle dégagé
            if obstacle > OBSTACLE_DIST + 0.1:
                self.set_state('SEARCH')

        self.cmd_pub.publish(cmd)

    def set_state(self, new_state):
        if self.state != new_state:
            self.state = new_state
            self.get_logger().info(f'État → {new_state}')


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()