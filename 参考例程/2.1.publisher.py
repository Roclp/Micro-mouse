# ROS2 Publisher示例程序

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Publishernode(Node):
    def __init__(self,name):
            super().__init__(name)
            self.count = 0
            self.get_logger().info("大家好，我是{}".format(name))
            self.publisher = self.create_publisher(String,"command", 10) 
            # 创建一个计时回调，计时间隔0.5s
            self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
            """ 计时器回调函数 """
            msg = String()
            msg.data = 'hello {}'.format(self.count)
            self.publisher.publish(msg)   		# 发布数据
            self.get_logger().info(f'发布了指令：{msg.data}')    #打印一下发布的数据
            self.count += 1


def main(args=None):
    rclpy.init(args=args)			        # 初始化rclpy
    node = Publishernode("publisher_node")	# 新建一个节点
    rclpy.spin(node)			            # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown()			            # 关闭rclpy


if __name__ == '__main__':
    main()