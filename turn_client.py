import sys
import rclpy
from rclpy.node import Node
from custom_turtlebot_interface.srv import Turn

class TurnClient(Node):
    def __init__(self):
        super().__init__('turn_client')
        self.client = self.create_client(Turn, 'turn')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
    
    def send_request(self, movement):
        req = Turn.Request()
        req.movement = movement
        self.future = self.client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = TurnClient()
    node.send_request(sys.argv[1])
    
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().info('Service call failed %r' % (e,))
            else:
                node.get_logger().info('Success: %r' % (response.success,))
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
