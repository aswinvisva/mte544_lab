import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist, Pose
# from turtlesim.msg import Pose
from irobot_create_msgs.msg import WheelTicks
from math import pow, atan2, sqrt
from std_msgs.msg import String
from rclpy.qos import ReliabilityPolicy, QoSProfile
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import matplotlib.pyplot as plt

WHEEL_RADIUS = 0.03575
WHEEL_SEPARATION = 0.16
TICK2RAD = 0.001533981 + 0.005

def quaternion_from_yaw(yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(0.0)
    sp = math.sin(0.0)
    cr = math.cos(0.0)
    sr = math.sin(0.0)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr
    return q

def yaw_from_quaternion(q):
    return atan2(2.0 * q[3]*q[2] + q[0]*q[1], 1.0 - 2.0 * q[1]*q[1] - q[2]*q[2])

class WheelEncoderOdom(Node):

    def __init__(self):
        super().__init__('wheel_encoder_odom')

        self.last_message_ts = None
        self.initialize_wheel_encoder_odom = True
        self.imu_yaw = None
        self.use_imu = True
        
        self.robot_pose = [0, 0, 0]
        self.gt_pose = [0, 0, 0]
        self.last_tick = {
            "left": None,
            "right": None
        }

        self.subscription = self.create_subscription(
            WheelTicks,
            '/wheel_ticks',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.odom_publisher = self.create_publisher(
            Pose, '/wheel_encoder_odom',
            10
        )

    def imu_callback(self, msg):
        q = msg.orientation
        q = [q.w, q.x, q.y, q.z]
        self.imu_yaw = yaw_from_quaternion(q)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = msg.twist.twist.angular.z
        self.gt_pose = [x, y, yaw]

        if self.initialize_wheel_encoder_odom:
            self.robot_pose[0] = x
            self.robot_pose[1] = y
            self.initialize_wheel_encoder_odom = False

    def listener_callback(self, msg):
        ts = msg.header.stamp.nanosec
        current_wheel_l = msg.ticks_left
        current_wheel_r = msg.ticks_right
        if self.last_message_ts is not None and self.imu_yaw is not None:
            step_time = (ts - self.last_message_ts) * 1e-9

            wheel_l = TICK2RAD * (current_wheel_l - self.last_tick["left"])
            wheel_r = TICK2RAD * (current_wheel_r - self.last_tick["right"])

            delta_s = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0

            # if self.use_imu:
            #     theta = self.imu_yaw
            #     delta_theta = self.gt_pose[2] - self.robot_pose[2]
            # else:
            delta_theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION

            self.robot_pose[0] += delta_s * math.cos(self.robot_pose[2] + (delta_theta / 2.0))
            self.robot_pose[1] += delta_s * math.sin(self.robot_pose[2] + (delta_theta / 2.0))
            self.robot_pose[2] += delta_theta

            q = quaternion_from_yaw(self.robot_pose[2])

            pose = Pose()
            pose.position.x = self.robot_pose[0]
            pose.position.y = self.robot_pose[1]
            pose.position.z = 0.0
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]

            plt.plot(self.gt_pose[0], self.gt_pose[1], 'ro')
            plt.plot(self.robot_pose[0], self.robot_pose[1], 'go')

            plt.savefig('/home/aswin/trajectory.png')

            self.odom_publisher.publish(pose)

            print(f"Wheel Encoder - x: {self.robot_pose[0]}, y: {self.robot_pose[1]}, yaw: {math.degrees(self.robot_pose[2])}, delta yaw: {delta_theta}")
            # print(f"GT Odom - x: {self.gt_pose[0]}, y: {self.gt_pose[1]}, yaw: {math.degrees(self.gt_pose[2])}")

            error = ((self.gt_pose[0] - self.robot_pose[0]) ** 2 + (self.gt_pose[1] - self.robot_pose[1]) ** 2) ** 0.5
            # print(f"Error: {error}")

        self.last_message_ts = ts
        self.last_tick["left"] = current_wheel_l
        self.last_tick["right"] = current_wheel_r

def main(args=None):
    rclpy.init(args=args)
    wheel_encoder_odom = WheelEncoderOdom()

    rclpy.spin(wheel_encoder_odom)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wheel_encoder_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()