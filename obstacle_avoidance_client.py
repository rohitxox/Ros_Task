#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from custom_turtlebot_interface.action import MoveToWall

class MoveToWallClient(Node):
    def __init__(self):
        super().__init__('move_to_wall_client')
        self.client = ActionClient(self, MoveToWall, 'move_to_wall')


    def send_goal(self, meters_to_stop):
        goal_msg = MoveToWall.Goal()
        goal_msg.meters_to_stop = meters_to_stop
        self.client.wait_for_server()
        self._send_goal_future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Seconds moving: {feedback_msg.feedback.seconds_moving}')


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Robot stopped {result.distance} meters from the obstacle')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    client = MoveToWallClient()
    client.send_goal(1.0)
    rclpy.spin(client)
    

if __name__ == '__main__':
    main()
