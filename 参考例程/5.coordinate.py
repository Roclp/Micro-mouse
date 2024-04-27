# 运行指定数量的单元格 示例程序

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf_transformations
import threading


class Drive(object):
    def __init__(self) -> None:
        self.valueinit()
        self.ros_register()

    def valueinit(self):
        self.l_dis = 0
        self.fl_dis = 0
        self.f_dis = 0
        self.fr_dis = 0
        self.r_dis = 0
        self.position_x = 0
        self.position_y = 0
        self.yaw = 1.5708

        self.blocksize = 0.178				# 迷宫单元格预设大小
        self.kp = 10

        self.speed_x = 0.10
        self.speed_z = 0.35

    def ros_register(self):
        self.msg = Twist()
        self.string = String()
        self.node = Node('mynode')
        self.node.create_subscription(LaserScan, '/scan', self.laser, 10) # 创建订阅者，/scan
        self.node.create_subscription(Odometry, '/odom', self.odom, 10) # 创建订阅者，/odom
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)	# 创建发布者，/cmd_vel
        self.rate = self.node.create_rate(50.0)

    def ros_spin(self):			# rclpy.spin()，在子线程调用
        rclpy.spin(self.node)

    def laser(self, msg):			# /scan回调
        region = msg.ranges
        self.l_dis = region[340]
        self.fl_dis = region[270]
        self.f_dis = region[180]
        self.fr_dis = region[90]
        self.r_dis = region[20]

    def get_laser(self):
        return self.l_dis, self.fl_dis, self.f_dis, self.fr_dis, self.r_dis

    def odom(self, msg):					# /odom回调
        position = msg.pose.pose.position
        self.position_x = position.x
        self.position_y = position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = tf_transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def get_odom(self):
        return self.position_x, self.position_y, self.yaw

    def posture_adjust(self):				# 判断是否偏移，然后校正
        if self.f_dis>0.09:
            if self.fl_dis<0.118:
                speed_z_temp = 0.5*(self.fl_dis-0.118)*self.kp
            elif self.fr_dis<0.118:
                speed_z_temp = -0.5*(self.fr_dis-0.118)*self.kp
            else:
                speed_z_temp = 0.0
        else:
            if self.l_dis<0.085:
                speed_z_temp = 0.5*(self.l_dis-0.085)*self.kp
            elif self.r_dis<0.085:
                speed_z_temp = -0.5*(self.r_dis-0.085)*self.kp
            else:
                speed_z_temp = 0.0
        return speed_z_temp 

    def move(self, numblock=1):			# 驱动迷宫机器人运行
        flag = 1
        while rclpy.ok():
            self.msg.linear.x = self.speed_x
            self.msg.angular.z = self.posture_adjust()
            self.pub.publish(self.msg)
            self.rate.sleep()
            if flag:		# 刚开始驱动时self.get_odom()返回的是0，因此添加此判断
                tempx, tempy, tempz = self.get_odom()
                if (not tempx) and (not tempy):
                    continue
                flag = 0
            if abs(self.get_odom()[0] - tempx)>=self.blocksize*numblock or abs(self.get_odom()[1] - tempy)>=self.blocksize*numblock:  # 当移动的距离超过设定值时break
                self.msg.linear.x = 0.0
                self.msg.angular.z = 0.0
                self.pub.publish(self.msg)
                self.rate.sleep()
                break


def main():
    rclpy.init(args=None)
    drive = Drive()
    t = threading.Thread(None, target=drive.ros_spin, daemon=True)
    t.start()
    drive.move(6)			# 移动6个单元格

if __name__ == '__main__':
    try:
        main()
    finally:
        rclpy.shutdown()