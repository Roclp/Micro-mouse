# ROS2 Subscription 示例程序

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Subscribernode(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是{}".format(name))
        # 订阅话题
        self.subscribe = self.create_subscription(String,"command",self.command_callback,10) 

    def command_callback(self, msg):
        self.get_logger().info('收到消息，内容是{}'.format(msg.data))

    
def main(args=None):
    rclpy.init(args=args)			            # 初始化rclpy
    node = Subscribernode("subscriber_node")	# 新建一个节点
    rclpy.spin(node)			                # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown()			                # 关闭rclpy


if __name__ == '__main__':
    main()