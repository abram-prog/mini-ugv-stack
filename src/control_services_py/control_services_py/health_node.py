import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

class HealthNode(Node):
    def __init__(self):
        super().__init__('health_node')
        self.pub = self.create_publisher(String, '/system/health', 10)
        self.timer = self.create_timer(1.0, self._tick)  # 1 Hz
        self.srv = self.create_service(Trigger, 'health_check', self._on_check)
        self.get_logger().info('health_node up: publishing /system/health and serving /health_check')

    def _tick(self):
        msg = String()
        msg.data = 'OK'
        self.pub.publish(msg)

    def _on_check(self, request, response):
        response.success = True
        response.message = 'alive'
        return response

def main():
    rclpy.init()
    node = HealthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
