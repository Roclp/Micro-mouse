# ROS2节点示例程序

import rclpy
from rclpy.node import Node

class Demonode(Node):
    def __init__(self,name):
        super().__init__(name)
        self.count = 0
        self.get_logger().info("{}节点启动了".format(name))
        # 创建一个计时器回调，计时间隔0.5s
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        """ 计时器回调函数 """
        self.get_logger().info('这是第{}次回调'.format(self.count)) #打印一下发布的数据
        self.count += 1


def main(args=None):
    rclpy.init()		    # 初始化rclpy
    node = Demonode("demo") # 新建一个节点
    rclpy.spin(node)		# 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown()		# 关闭rclpy


if __name__ == '__main__':
    main()