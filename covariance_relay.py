"""Gives the simulator's odometry and IMU messages a covariance.

Gazebo publishes both with all-zero covariance matrices, and robot_localization
does not fuse a measurement it is told is perfect. Same fix as on the real rover.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class CovarianceRelay(Node):
    def __init__(self):
        super().__init__('covariance_relay')
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odometry', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.create_subscription(Odometry, '/model/my_robot/odometry', self.odom_cb, 10)
        self.create_subscription(Imu, '/imu', self.imu_cb, 10)

    def odom_cb(self, msg):
        msg.header.frame_id = 'my_robot/odom'
        msg.child_frame_id = 'my_robot/chassis'
        cov = [0.0] * 36
        cov[0] = 0.02    # vx: wheels slip a little
        cov[7] = 0.02    # vy
        cov[35] = 0.10   # yaw rate from the wheels is the weak point of a skid-steer base
        msg.twist.covariance = cov
        self.odom_pub.publish(msg)

    def imu_cb(self, msg):
        msg.header.frame_id = 'my_robot/chassis'
        msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.005]
        msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.002]
        msg.linear_acceleration_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
        self.imu_pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(CovarianceRelay())


if __name__ == '__main__':
    main()
