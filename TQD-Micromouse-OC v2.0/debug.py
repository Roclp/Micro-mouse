"""new 集成后的"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf_transformations
import threading
import os
from configparser import ConfigParser


class Drive(object):
    def __init__(self) -> None:
        self.valueinit()
        self.ros_register()

    def valueinit(self):

        if os.path.exists('config.ini'):
            config = ConfigParser()
            with open('config.ini') as f:
                config.read_file(f)

                self.threshold_l = float(config.get('threshold', 'threshold_l'))
                self.threshold_lf = float(config.get('threshold', 'threshold_lf'))
                self.threshold_f = float(config.get('threshold', 'threshold_f'))
                self.threshold_rf = float(config.get('threshold', 'threshold_rf'))
                self.threshold_r = float(config.get('threshold', 'threshold_r'))

                self.fyaw = float(config.get('threshold', 'fyaw'))
                self.ryaw = float(config.get('threshold', 'ryaw'))
                self.byaw = float(config.get('threshold', 'byaw'))
                self.lyaw0 = float(config.get('threshold', 'lyaw0'))
                self.lyaw1 = float(config.get('threshold', 'lyaw1'))

                self.blocksize = float(config.get('threshold', 'blocksize'))
                self.kp = float(config.get('threshold', 'kp'))

                self.speed_x = float(config.get('threshold', 'speed_x'))
                self.speed_z = float(config.get('threshold', 'speed_z'))

        else:
            self.threshold_l = 0.085
            self.threshold_lf = 0.12
            self.threshold_f = 0.078
            self.threshold_rf = 0.12
            self.threshold_r = 0.085         

            self.fyaw = 4.71
            self.ryaw = 3.14
            self.byaw = 1.57
            self.lyaw0 = 0
            self.lyaw1 = 6.28

            self.blocksize = 0.17
            self.kp = 10

            self.speed_x = 0.10
            self.speed_z = 0.35

        self.yaw = 0
        self.l_dis = 0
        self.fl_dis = 0
        self.f_dis = 0
        self.fr_dis = 0
        self.r_dis = 0
        self.position_x = 0
        self.position_y = 0

    def ros_register(self):
        self.msg = Twist()
        self.string = String()
        self.node = Node('mynode')
        self.node.create_subscription(LaserScan, '/scan', self.laser, 10)
        self.node.create_subscription(Odometry, '/odom', self.odom, 10)
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_custom = self.node.create_publisher(String, '/mycustom', 10)
        self.rate = self.node.create_rate(50.0)

    def ros_pub(self):
        self.msg.linear.x = self.speed_x
        self.msg.angular.z = self.speed_z
        self.pub.publish(self.msg)

    def ros_spin(self):
        rclpy.spin(self.node)

    def laser(self, msg):
        region = msg.ranges
        self.l_dis = region[340]
        self.fl_dis = region[270]
        self.f_dis = region[180]
        self.fr_dis = region[90]
        self.r_dis = region[20]

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

    def get_odom(self):
        return self.position_x, self.position_y, self.yaw

    def posture_adjust(self):
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

    def move(self, numblock=1):
        flag = 1
        flag_cps = 0
        compensate = 0
        while rclpy.ok():
            self.msg.linear.x = self.speed_x
            self.msg.angular.z = self.posture_adjust()
            self.pub.publish(self.msg)
            self.rate.sleep()

            if flag:
                tempx, tempy, tempz = self.get_odom()
                if (not tempx) and (not tempy):
                    continue
                flag = 0
                if self.l_dis<0.1 and self.r_dis<0.1:
                    flag_cps = 0
                elif self.l_dis<0.1 and self.r_dis>0.1:
                    flag_cps = 1
                if self.l_dis>0.1 and self.r_dis<0.1:
                    flag_cps = 2
                elif self.l_dis>0.1 and self.r_dis>0.1:
                    flag_cps = 3

            if 0.01<self.f_dis<0.075:
                self.msg.linear.x = 0.0
                self.msg.angular.z = 0.0
                self.pub.publish(self.msg)
                # rclpy.spin_once(self.node)
                self.rate.sleep()
                break

            if abs(self.get_odom()[0] - tempx)>=self.blocksize*numblock or abs(self.get_odom()[1] - tempy)>=self.blocksize*numblock:
                self.msg.linear.x = 0.0
                self.msg.angular.z = 0.0
                self.pub.publish(self.msg)
                # rclpy.spin_once(self.node)
                self.rate.sleep()
                break

            if flag_cps ==0 and (self.l_dis>0.1 or self.r_dis>0.1):
                flag = 1
                compensate = 1
                break

        """
            # 其他情况，暂时没想好怎么处理，所以注释掉
            if flag_cps ==1 and self.l_dis>0.1:
                flag = 1
                compensate = 0
                break

            if flag_cps ==2 and self.r_dis>0.1:
                flag = 1
                compensate = 0
                break
        """

        while compensate and rclpy.ok():
            self.msg.linear.x = self.speed_x
            self.msg.angular.z = self.posture_adjust()
            self.pub.publish(self.msg)
            self.rate.sleep()
            if flag:
                tempx, tempy, tempz = self.get_odom()
                if (not tempx) and (not tempy):
                    continue
                flag = 0

            if 0.01<self.f_dis<0.075:
                self.msg.linear.x = 0.0
                self.msg.angular.z = 0.0
                self.pub.publish(self.msg)
                # rclpy.spin_once(self.node)
                self.rate.sleep()
                break

            if abs(self.get_odom()[0] - tempx)>=self.blocksize/2+0.01 or abs(self.get_odom()[1] - tempy)>=self.blocksize/2+0.01:
                self.msg.linear.x = 0.0
                self.msg.angular.z = 0.0
                self.pub.publish(self.msg)
                # rclpy.spin_once(self.node)
                self.rate.sleep()
                break

    def turn(self, angle):
        while rclpy.ok():
            self.msg.linear.x = 0.0
            self.msg.angular.z = -self.speed_z
            self.pub.publish(self.msg)
            self.rate.sleep()
            if angle-0.1<self.yaw<angle+0.1:
                self.msg.linear.x = 0.0
                self.msg.angular.z = 0.0
                self.pub.publish(self.msg)
                # rclpy.spin_once(self.node)
                self.rate.sleep()
                break

    def turnright(self):
        flag = 1
        while rclpy.ok():
            self.msg.linear.x = 0.0
            self.msg.angular.z = -self.speed_z
            self.pub.publish(self.msg)
            self.rate.sleep()
            if flag:  # It is similar with move.
                oldyaw = self.get_odom()[2]
                if not oldyaw:
                    continue
                flag = 0
            # Considering the mutation of 0 and 6.28, so dividing the whole process into four parts.

            if 4.41<oldyaw<5.01:  # up
                if self.ryaw-0.1<self.yaw<self.ryaw+0.1:
                    self.msg.linear.x = 0.0
                    self.msg.angular.z = 0.0
                    self.pub.publish(self.msg)
                    # rclpy.spin_once(self.node)
                    self.rate.sleep()
                    break

            elif 2.84<oldyaw<3.44:  # right
                if self.byaw-0.1<self.yaw<self.byaw+0.1:
                    self.msg.linear.x = 0.0
                    self.msg.angular.z = 0.0
                    self.pub.publish(self.msg)
                    # rclpy.spin_once(self.node)
                    self.rate.sleep()
                    break

            elif 1.27<oldyaw<1.87:  # back
                if self.yaw<self.lyaw0+0.1:
                    self.msg.linear.x = 0.0
                    self.msg.angular.z = 0.0
                    self.pub.publish(self.msg)
                    # rclpy.spin_once(self.node)
                    self.rate.sleep()
                    break

            elif oldyaw<0.3 or oldyaw>5.98:  # left
                if self.fyaw-0.1<self.yaw<self.fyaw+0.1:
                    self.msg.linear.x = 0.0
                    self.msg.angular.z = 0.0
                    self.pub.publish(self.msg)
                    # rclpy.spin_once(self.node)
                    self.rate.sleep()
                    break

    def turnleft(self):
        flag = 1
        while rclpy.ok():
            self.msg.linear.x = 0.0
            self.msg.angular.z = self.speed_z
            self.pub.publish(self.msg)
            self.rate.sleep()
            if flag:
                oldyaw = self.yaw
                if not oldyaw:
                    continue
                flag = 0
            if 4.41<oldyaw<5.01:  # up
                if self.yaw>self.lyaw1-0.1:
                    self.msg.linear.x = 0.0
                    self.msg.angular.z = 0.0
                    self.pub.publish(self.msg)
                    # rclpy.spin_once(self.node)
                    self.rate.sleep()
                    break
            elif 2.84<oldyaw<3.44:  # right
                if self.fyaw-0.1<self.yaw<self.fyaw+0.1:
                    self.msg.linear.x = 0.0
                    self.msg.angular.z = 0.0
                    self.pub.publish(self.msg)
                    # rclpy.spin_once(self.node)
                    self.rate.sleep()
                    break
            elif 1.27<oldyaw<1.87:  # back
                if self.ryaw-0.1<self.yaw<self.ryaw+0.1:
                    self.msg.linear.x = 0.0
                    self.msg.angular.z = 0.0
                    self.pub.publish(self.msg)
                    # rclpy.spin_once(self.node)
                    self.rate.sleep()
                    break
            elif oldyaw<0.3 or oldyaw>5.98:  # left
                if self.byaw-0.1<self.yaw<self.byaw+0.1:
                    self.msg.linear.x = 0.0
                    self.msg.angular.z = 0.0
                    self.pub.publish(self.msg)
                    # rclpy.spin_once(self.node)
                    self.rate.sleep()
                    break

    def turnback(self):
        flag = 1
        while rclpy.ok():
            self.msg.linear.x = 0.0
            self.msg.angular.z = -self.speed_z
            self.pub.publish(self.msg)
            self.rate.sleep()
            if flag:
                oldyaw = self.get_odom()[2]
                if not oldyaw:
                    continue
                flag = 0
            if 4.41<oldyaw<5.01:  # up
                if self.byaw-0.1<self.yaw<self.byaw+0.1:
                    self.msg.linear.x = 0.0
                    self.msg.angular.z = 0.0
                    self.pub.publish(self.msg)
                    # rclpy.spin_once(self.node)
                    self.rate.sleep()
                    break
            elif 2.84<oldyaw<3.44:  # right
                if self.yaw<self.lyaw0+0.1:
                    self.msg.linear.x = 0.0
                    self.msg.angular.z = 0.0
                    self.pub.publish(self.msg)
                    # rclpy.spin_once(self.node)
                    self.rate.sleep()
                    break
            elif 1.27<oldyaw<1.87:  # back
                if self.fyaw-0.1<self.yaw<self.fyaw+0.1:
                    self.msg.linear.x = 0.0
                    self.msg.angular.z = 0.0
                    self.pub.publish(self.msg)
                    # rclpy.spin_once(self.node)
                    self.rate.sleep()
                    break
            elif oldyaw<0.3 or oldyaw>5.98:  # left
                if self.ryaw-0.1<self.yaw<self.ryaw+0.1:
                    self.msg.linear.x = 0.0
                    self.msg.angular.z = 0.0
                    self.pub.publish(self.msg)
                    # rclpy.spin_once(self.node)
                    self.rate.sleep()
                    break


class Control:
    def __init__(self) -> None:
        self.flag = 666
        self.loop = 1
        self.drive = Drive()
        self.drive.node.create_subscription(String, '/mycontrol', self.mycallback, 10)

    def mycallback(self, msg:String):
        self.commands = msg.data.split(',')
        command = self.commands[0]
        if command=='0':
            self.flag = 0
        elif command=='1':
            self.flag = 1
        elif command=='2':
            self.flag = 2

    def myspin(self):
        rclpy.spin(self.drive.node)

    def myrun(self):
        while self.loop:
            if self.flag==0:
                self.control_junior(self.commands[1:])
                self.flag = 666
            elif self.flag==1:
                self.control_senior1(self.commands[1:])
                self.flag = 666
            elif self.flag==2:
                self.control_senior2(self.commands[1:])
                self.flag = 666
            else:
                pass

    def setLoop(self, flag:bool):
        self.loop = flag

    def control_junior(self, commands):
        threshold_l, threshold_lf, threshold_f, threshold_rf, threshold_r, numblock, blocksize, speed_x, kp = commands
        self.drive.blocksize = float(blocksize)
        self.drive.speed_x = float(speed_x)
        self.drive.kp = int(kp)
        for i in range(int(numblock)):
            self.drive.move()

    def control_senior1(self, commands):
        speed_z, yaw = commands
        self.drive.speed_z = float(speed_z)
        self.drive.turn(float(yaw))

    def control_senior2(self, commands):
        speed_z, index, fyaw, ryaw, byaw, lyaw0, lyaw1 = commands
        self.drive.speed_z = float(speed_z)
        self.drive.fyaw = float(fyaw)
        self.drive.ryaw = float(ryaw)
        self.drive.byaw = float(byaw)
        self.drive.lyaw0 = float(lyaw0)
        self.drive.lyaw1 = float(lyaw1)
        index = int(index)
        if index == 0:
            self.drive.turnright()
        elif index == 1:
            self.drive.turnback()
        elif index == 2:
            self.drive.turnleft()


if __name__ == '__main__':
    rclpy.init(args=None)
    mycontrol = Control()
    t = threading.Thread(None, target=mycontrol.myspin, daemon=True)
    t.start()
    mycontrol.myrun()
    rclpy.shutdown()