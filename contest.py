import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf_transformations
import threading
import numpy as np
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

            self.speed_x = 0.1
            self.speed_z = 0.2

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
        self.rate = self.node.create_rate(250.0)

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
            # 其他情况，暂时注释掉
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


class Micromouse(Drive):
    def __init__(self):
        """继承父类，可以增加自己要添加的内容。"""
        super().__init__()
        self.valuesinit()

    def valuesinit(self):
        """Micromosue参数初始化"""
        self.mapblock = np.full((16, 16), '00000')  # 2D array,墙壁资料初始化, flag left down right up
                                                    # flag: 是否走过, 0 or 1
                                                    # left: 是否有路, 0 or 1
                                                    # down: 是否有路, 0 or 1
                                                    # right: 是否有路, 0 or 1
                                                    # up: 是否有路, 0 or 1
                                                    
        self.mapblock[0,0] = '10001'                # 起点墙壁初始化 
        self.crosswaystack = []
        self.mouse_x = 0                            # x,y起点坐标，这是在迷宫中的坐标
        self.mouse_y = 0
        self.dir = 0                                # Micromouse车头,0:up, 1:right, 2:down, 3:left

    def __coordinateupdate(self):
        """根据车头方向更新坐标"""
        if self.dir == 0:           # 车头向上
            self.mouse_y += 1
        elif self.dir == 1:         # 车头向右
            self.mouse_x += 1
        elif self.dir == 2:         # 车头向下
            self.mouse_y -= 1
        elif self.dir == 3:         # 车头向左
            self.mouse_x -= 1

    def get_coordinate(self):
        """获取坐标，方便调用"""
        return self.mouse_x, self.mouse_y

    def __savewallinfo(self):
        """墙壁信息保存"""
        frontpath = '1' if self.f_dis > 0.15 else '0'
        rightpath = '1' if self.r_dis > 0.15 else '0'
        backpath = '1'
        leftpath = '1' if self.l_dis > 0.15 else '0'
        # 墙壁资料赋值
        if self.dir == 0:
            self.mapblock[self.mouse_x, self.mouse_y] = ''.join(('1', leftpath, backpath, rightpath, frontpath))
        elif self.dir == 1:
            self.mapblock[self.mouse_x, self.mouse_y] = ''.join(('1', backpath, rightpath, frontpath, leftpath))
        elif self.dir == 2:
            self.mapblock[self.mouse_x, self.mouse_y] = ''.join(('1', rightpath, frontpath, leftpath, backpath))
        elif self.dir == 3:
            self.mapblock[self.mouse_x, self.mouse_y] = ''.join(('1', frontpath, leftpath, backpath, rightpath))

    def turnright(self):
        """右转弯，加入了方向转换"""
        self.dir = (self.dir+1)%4
        return super(Micromouse, self).turnright()

    def turnleft(self):
        """左转弯，加入了方向转换"""
        self.dir = (self.dir+3)%4
        return super(Micromouse, self).turnleft()

    def turnback(self):
        """向后转弯，加入了方向转换"""
        self.dir = (self.dir+2)%4
        return super(Micromouse, self).turnback()

    def moveoneblock(self):
        """移动一个单元格，然后更新坐标，保存墙壁信息的功能"""
        self.move()
        self.__coordinateupdate()
        if self.mapblock[self.mouse_x, self.mouse_y][0]=='0':
            self.__savewallinfo()

    def rightmethod(self):
        frontpath = '1' if self.f_dis > 0.2 else '0'
        rightpath = '1' if self.r_dis > 0.2 else '0'
        backpath = '1'
        leftpath = '1' if self.l_dis > 0.2 else '0'
        if rightpath == '1':        # 右侧有路
            if self.dir == 0:
                flag = self.mapblock[self.mouse_x+1, self.mouse_y][0]
            elif self.dir == 1:
                flag = self.mapblock[self.mouse_x, self.mouse_y-1][0]
            elif self.dir == 2:
                flag = self.mapblock[self.mouse_x-1, self.mouse_y][0]
            elif self.dir == 3:
                flag = self.mapblock[self.mouse_x, self.mouse_y+1][0]
            if flag == '0':
                self.turnright()
                return

        if frontpath == '1':        # 前方有路
            if self.dir == 0:
                flag = self.mapblock[self.mouse_x, self.mouse_y+1][0]
            elif self.dir == 1:
                flag = self.mapblock[self.mouse_x+1, self.mouse_y][0]
            elif self.dir == 2:
                flag = self.mapblock[self.mouse_x, self.mouse_y-1][0]
            elif self.dir == 3:
                flag = self.mapblock[self.mouse_x-1, self.mouse_y][0]
            if flag == '0':
                return

        if leftpath == '1':         # 左侧有路
            if self.dir == 0:
                flag = self.mapblock[self.mouse_x-1, self.mouse_y][0]
            elif self.dir == 1:
                flag = self.mapblock[self.mouse_x, self.mouse_y+1][0]
            elif self.dir == 2:
                flag = self.mapblock[self.mouse_x+1, self.mouse_y][0]
            elif self.dir == 3:
                flag = self.mapblock[self.mouse_x, self.mouse_y-1][0]
            if flag == '0':
                self.turnleft()
                return

    def leftmethod(self):
        frontpath = '1' if self.f_dis > 0.2 else '0'
        rightpath = '1' if self.r_dis > 0.2 else '0'
        backpath = '1'
        leftpath = '1' if self.l_dis > 0.2 else '0'
        if leftpath == '1':         # 左侧有路
            if self.dir == 0:
                flag = self.mapblock[self.mouse_x-1, self.mouse_y][0]
            elif self.dir == 1:
                flag = self.mapblock[self.mouse_x, self.mouse_y+1][0]
            elif self.dir == 2:
                flag = self.mapblock[self.mouse_x+1, self.mouse_y][0]
            elif self.dir == 3:
                flag = self.mapblock[self.mouse_x, self.mouse_y-1][0]
            if flag == '0':
                self.turnleft()
                return

        if frontpath == '1':        # 前方有路
            if self.dir == 0:
                flag = self.mapblock[self.mouse_x, self.mouse_y+1][0]
            elif self.dir == 1:
                flag = self.mapblock[self.mouse_x+1, self.mouse_y][0]
            elif self.dir == 2:
                flag = self.mapblock[self.mouse_x, self.mouse_y-1][0]
            elif self.dir == 3:
                flag = self.mapblock[self.mouse_x-1, self.mouse_y][0]
            if flag == '0':
                return

        if rightpath == '1':        # 右侧有路
            if self.dir == 0:
                flag = self.mapblock[self.mouse_x+1, self.mouse_y][0]
            elif self.dir == 1:
                flag = self.mapblock[self.mouse_x, self.mouse_y-1][0]
            elif self.dir == 2:
                flag = self.mapblock[self.mouse_x-1, self.mouse_y][0]
            elif self.dir == 3:
                flag = self.mapblock[self.mouse_x, self.mouse_y+1][0]
            if flag == '0':
                self.turnright()
                return

    def frontrightmethod(self):
        frontpath = '1' if self.f_dis > 0.2 else '0'
        rightpath = '1' if self.r_dis > 0.2 else '0'
        backpath = '1'
        leftpath = '1' if self.l_dis > 0.2 else '0'
        if frontpath == '1':        # 前方有路
            if self.dir == 0:
                flag = self.mapblock[self.mouse_x, self.mouse_y+1][0]
            elif self.dir == 1:
                flag = self.mapblock[self.mouse_x+1, self.mouse_y][0]
            elif self.dir == 2:
                flag = self.mapblock[self.mouse_x, self.mouse_y-1][0]
            elif self.dir == 3:
                flag = self.mapblock[self.mouse_x-1, self.mouse_y][0]
            if flag == '0':
                return

        if rightpath == '1':        # 右侧有路
            if self.dir == 0:
                flag = self.mapblock[self.mouse_x+1, self.mouse_y][0]
            elif self.dir == 1:
                flag = self.mapblock[self.mouse_x, self.mouse_y-1][0]
            elif self.dir == 2:
                flag = self.mapblock[self.mouse_x-1, self.mouse_y][0]
            elif self.dir == 3:
                flag = self.mapblock[self.mouse_x, self.mouse_y+1][0]
            if flag == '0':
                self.turnright()
                return

        if leftpath == '1':         # 左侧有路
            if self.dir == 0:
                flag = self.mapblock[self.mouse_x-1, self.mouse_y][0]
            elif self.dir == 1:
                flag = self.mapblock[self.mouse_x, self.mouse_y+1][0]
            elif self.dir == 2:
                flag = self.mapblock[self.mouse_x+1, self.mouse_y][0]
            elif self.dir == 3:
                flag = self.mapblock[self.mouse_x, self.mouse_y-1][0]
            if flag == '0':
                self.turnleft()
                return

    def frontleftmethod(self):
        frontpath = '1' if self.f_dis > 0.2 else '0'
        rightpath = '1' if self.r_dis > 0.2 else '0'
        backpath = '1'
        leftpath = '1' if self.l_dis > 0.2 else '0'
        if frontpath == '1':        # 前方有路
            if self.dir == 0:
                flag = self.mapblock[self.mouse_x, self.mouse_y+1][0]
            elif self.dir == 1:
                flag = self.mapblock[self.mouse_x+1, self.mouse_y][0]
            elif self.dir == 2:
                flag = self.mapblock[self.mouse_x, self.mouse_y-1][0]
            elif self.dir == 3:
                flag = self.mapblock[self.mouse_x-1, self.mouse_y][0]
            if flag == '0':
                return

        if leftpath == '1':         # 左侧有路
            if self.dir == 0:
                flag = self.mapblock[self.mouse_x-1, self.mouse_y][0]
            elif self.dir == 1:
                flag = self.mapblock[self.mouse_x, self.mouse_y+1][0]
            elif self.dir == 2:
                flag = self.mapblock[self.mouse_x+1, self.mouse_y][0]
            elif self.dir == 3:
                flag = self.mapblock[self.mouse_x, self.mouse_y-1][0]
            if flag == '0':
                self.turnleft()
                return

        if rightpath == '1':        # 右侧有路
            if self.dir == 0:
                flag = self.mapblock[self.mouse_x+1, self.mouse_y][0]
            elif self.dir == 1:
                flag = self.mapblock[self.mouse_x, self.mouse_y-1][0]
            elif self.dir == 2:
                flag = self.mapblock[self.mouse_x-1, self.mouse_y][0]
            elif self.dir == 3:
                flag = self.mapblock[self.mouse_x, self.mouse_y+1][0]
            if flag == '0':
                self.turnright()
                return

    def centralmethod(self):
        if self.mouse_x < 8:
            if self.mouse_y < 8:        # Micromouse位于左下角
                if self.dir == 0:
                    self.frontrightmethod()
                elif self.dir == 1:
                    self.frontleftmethod()
                elif self.dir == 2:
                    self.leftmethod()
                elif self.dir == 3:
                    self.rightmethod()
            else:                       # Micromouse位于左上角
                if self.dir == 0:
                    self.rightmethod()
                elif self.dir == 1:
                    self.frontrightmethod()
                elif self.dir == 2:
                    self.frontleftmethod()
                elif self.dir == 3:
                    self.leftmethod()
        else:
            if self.mouse_y < 8:        # Micromouse位于右下角
                if self.dir == 0:
                    self.frontleftmethod()
                elif self.dir == 1:
                    self.leftmethod()
                elif self.dir == 2:
                    self.rightmethod()
                elif self.dir == 3:
                    self.frontrightmethod()
            else:                       # Micromouse位于右下角
                if self.dir == 0:
                    self.leftmethod()
                elif self.dir == 1:
                    self.rightmethod()
                elif self.dir == 2:
                    self.frontrightmethod()
                elif self.dir == 3:
                    self.frontleftmethod()

    def destinationcheck(self):
        if (self.mouse_x==7 and self.mouse_y==7) or (self.mouse_x==7 and self.mouse_y==8) or (self.mouse_x==8 and self.mouse_y==7) or (self.mouse_x==8 and self.mouse_y==8):
            return True
        else:
            return False

    def crosswaychoice(self):
        self.centralmethod()

    def mapstepedit(self, dx, dy):
        """制作等高图"""
        self.mapstep = np.full((16,16), 255)
        step = 1
        n = 1
        stack = []
        stack.append((dx, dy))
        cx = dx
        cy = dy

        while n:
            self.mapstep[cx, cy] = step
            step += 1

            count = 0
            # 统计当前坐标有几个可前进的方向
            if (self.mapblock[cx, cy][-1] == '1') and (self.mapstep[cx, cy+1]>step):
                count += 1
            if (self.mapblock[cx, cy][-2] == '1') and (self.mapstep[cx+1, cy]>step):
                count += 1
            if (self.mapblock[cx, cy][-3] == '1') and (self.mapstep[cx, cy-1]>step):
                count += 1
            if (self.mapblock[cx, cy][-4] == '1') and (self.mapstep[cx-1, cy]>step):
                count += 1

            if count == 0:
                cx, cy = stack.pop()
                step = self.mapstep[cx, cy]
                n -= 1
            else:
                if count>1:
                    stack.append((cx, cy))
                    n += 1
                # 随便挑一个方向，不影响结果，因为会全部走一遍
                if (self.mapblock[cx, cy][-1] == '1') and (self.mapstep[cx, cy+1]>step):
                    cy += 1
                    continue
                if (self.mapblock[cx, cy][-2] == '1') and (self.mapstep[cx+1, cy]>step):
                    cx += 1
                    continue
                if (self.mapblock[cx, cy][-3] == '1') and (self.mapstep[cx, cy-1]>step):
                    cy -= 1
                    continue
                if (self.mapblock[cx, cy][-4] == '1') and (self.mapstep[cx-1, cy]>step):
                    cx -= 1
                    continue

    def objectgoto(self, dx, dy):
        """向目标点运动"""
        self.mapstepedit(dx, dy)
        temp_dir = 0 
        cx = self.mouse_x
        cy = self.mouse_y
        while (cx!=dx) or (cy!=dy):
            # 沿着等高值减小的方向运行，直至到达目标点
            step = self.mapstep[cx, cy]
            if (self.mapblock[cx, cy][-1] == '1') and (self.mapstep[cx, cy+1]<step):
                temp_dir = 0
            elif (self.mapblock[cx, cy][-2] == '1') and (self.mapstep[cx+1, cy]<step):
                temp_dir = 1
            elif (self.mapblock[cx, cy][-3] == '1') and (self.mapstep[cx, cy-1]<step):
                temp_dir = 2
            elif (self.mapblock[cx, cy][-4] == '1') and (self.mapstep[cx-1, cy]<step):
                temp_dir = 3
            d_dir = (temp_dir - self.dir + 4 )%4
            if d_dir == 1 :
                self.turnright()
            elif d_dir == 2 :
                self.turnback()
            elif d_dir == 3 :
                self.turnleft()
            self.moveoneblock()
            cx = self.mouse_x
            cy = self.mouse_y

    def crosswaycheck(self):
        temp = 0
        left, down, right, up = self.mapblock[self.mouse_x, self.mouse_y][1:]
        if up == '1':
            if self.mapblock[self.mouse_x, self.mouse_y+1][0] == '0':
                temp += 1
        if right == '1':
            if self.mapblock[self.mouse_x+1, self.mouse_y][0] == '0':
                temp += 1
        if down == '1':
            if self.mapblock[self.mouse_x, self.mouse_y-1][0] == '0':
                temp += 1
        if left == '1':
            if self.mapblock[self.mouse_x-1, self.mouse_y][0] == '0':
                temp += 1
        return temp

    def mazesearch(self):
        while 1:
            # print(self.mouse_x, self.mouse_y)
            if self.destinationcheck():                     # 如果到了终点，则返回起点
                destination = (self.mouse_x, self.mouse_y)
                self.turnback()
                self.objectgoto(0, 0)
                return (1, destination)
            else:                                           # 否则一直搜索
                crosswaycount = self.crosswaycheck()
                print(self.mouse_x, self.mouse_y, self.mapblock[self.mouse_x, self.mouse_y], crosswaycount)
                if crosswaycount:
                    if crosswaycount>1:
                        self.crosswaystack.append((self.mouse_x, self.mouse_y))
                        self.crosswaychoice()
                        self.moveoneblock()
                    if crosswaycount==1:
                        self.crosswaychoice()
                        self.moveoneblock()
                else:
                    self.turnback()
                    self.objectgoto(*self.crosswaystack.pop())

    def spurt(self, dest):
        self.objectgoto(*dest)
        self.turnback()
        self.objectgoto(0, 0)

def main():
    rclpy.init(args=None)
    micromouse = Micromouse()
    t = threading.Thread(None, target=micromouse.ros_spin, daemon=True)
    t.start()
    flag, destination = micromouse.mazesearch()
    micromouse.spurt(destination)


if __name__ == '__main__':
    try:
        main()
    finally:
        rclpy.shutdown()

