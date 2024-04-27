# 循迹走直线 示例程序

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf_transformations
import threading


class Drive:
    def __init__(self) -> None:
        self.valueinit()
        self.ros_register()

    def valueinit(self):
        self.l_dis = 0						        # 为各个属性赋初值
        self.fl_dis = 0
        self.f_dis = 0
        self.fr_dis = 0
        self.r_dis = 0
        self.kp = 10						        # 校正车姿的强度
        self.speed_x = 0.1

    def ros_register(self):
        self.node = Node('mynode')
        self.msg = Twist()
        self.node.create_subscription(LaserScan, '/scan', self.laser, 10)
        self.node.create_subscription(Odometry, '/odom', self.odom, 10)
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.node.create_rate(50.0)     # 创建Rate对象，固定速率

    def laser(self, msg):					       # /scan回调
        region = msg.ranges
        self.l_dis = region[355]
        self.fl_dis = region[270]
        self.f_dis = region[180]
        self.fr_dis = region[90]
        self.r_dis = region[5]

    def get_laser(self):
        return self.l_dis, self.fl_dis, self.f_dis, self.fr_dis, self.r_dis

    def odom(self, msg):
        position = msg.pose.pose.position
        self.position_x = position.x
        self.position_y = position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = tf_transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]+3.1415926
        # print(self.get_odom())

    def get_odom(self):
        return self.position_x, self.position_y, self.yaw

    def ros_spin(self):					        # rclpy.spin()，在子线程调用
        rclpy.spin(self.node)

    def posture_adjust(self):			        # 判断是否偏移，然后校正
        if self.l_dis<0.085:
            speed_z_temp = 0.5*(self.l_dis-0.085)*self.kp
        elif self.r_dis<0.085:
            speed_z_temp = -0.5*(self.r_dis-0.085)*self.kp
        else:
            speed_z_temp = 0.0
        return speed_z_temp 

    def move(self):						        # 迷宫机器人运行
        while rclpy.ok():
            self.msg.linear.x = self.speed_x
            self.msg.angular.z = self.posture_adjust()
            self.pub.publish(self.msg)
            self.rate.sleep()					# 实现固定速率发布
            print(self.f_dis)
            if 0.05<self.f_dis<0.08:			# 检测到前方挡板时停车
                self.msg.linear.x = 0.0
                self.msg.angular.z = 0.0
                self.pub.publish(self.msg)
                self.rate.sleep()
                break

def main():
    rclpy.init(args=None)
    track = Drive()
    t = threading.Thread(None, target=track.ros_spin, daemon=True)	# 在子线程调用rclpy.spin()
    t.start()
    track.move()


if __name__ == '__main__':
    main()