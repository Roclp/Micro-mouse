# 激光雷达、位置、偏航角 获取

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf_transformations


class Drive(object):
    def __init__(self) -> None:
        self.ros_register()


    def ros_register(self):
        self.msg = Twist()
        self.node = Node('mynode')
        self.node.create_subscription(LaserScan, '/scan', self.laser, 10)
        self.node.create_subscription(Odometry, '/odom', self.odom, 10)
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.node.create_timer(0.1, self.ros_pub)

    def ros_spin(self):
        rclpy.spin(self.node)

    def laser(self, msg):
        # 激光雷达数据获取
        region = msg.ranges
        self.l_dis = region[340]
        self.fl_dis = region[270]
        self.f_dis = region[180]
        self.fr_dis = region[90]
        self.r_dis = region[20]
        # print(self.get_laser())

    def get_laser(self):
        return self.l_dis, self.fl_dis, self.f_dis, self.fr_dis, self.r_dis

    def odom(self, msg):
        # 位置数据获取
        position = msg.pose.pose.position
        self.position_x = position.x
        self.position_y = position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = tf_transformations.euler_from_quaternion(quaternion)
        # 偏航角数据获取
        self.yaw = euler[2]+3.1415926
        print(self.get_odom())

    def get_odom(self):
        return self.position_x, self.position_y, self.yaw

    def ros_pub(self):
        self.msg.linear.x = 0.01
        self.msg.angular.z = 0.0
        self.pub.publish(self.msg)


def main():
    rclpy.init(args=None)
    drive = Drive()
    drive.ros_spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
