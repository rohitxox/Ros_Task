#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from custom_turtlebot_interface.action import MoveToWall
import time

class MoveToWallServer(Node):
    def __init__(self):
        super().__init__('move_to_wall_server')
        self.action_server = ActionServer(
            self,
            MoveToWall,
            'move_to_wall',
            self.execute_callback)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
        self.cmd = Twist()
        self.cmd.linear.x = 0.0  # Prevent automatic forward movement
        self.cmd.angular.z = 0.0
        self.current_goal = None
        self.goal_active = False


    def execute_callback(self, goal_handle):
        self.current_goal = goal_handle.request.meters_to_stop
        self.goal_active = True
        self.get_logger().info(f'Executing goal with stopping threshold: {self.current_goal} meters.')
        feedback = MoveToWall.Feedback()
        result = MoveToWall.Result()
        seconds_moving = 0

        while rclpy.ok() and self.goal_active:
            feedback.seconds_moving = seconds_moving
            goal_handle.publish_feedback(feedback)
            time.sleep(1)
            if self.cmd.linear.x == 0.0 and self.cmd.angular.z == 0.0:
                self.goal_active = False
                result.distance = round(self.current_goal, 2)
                goal_handle.succeed()
                return result
            seconds_moving += 1


    def laser_callback(self, msg):
        distance = min(msg.ranges)
        distance = round(distance, 2)  # Round LIDAR data to 2 decimal places
        if self.current_goal is not None and distance <= self.current_goal:
            self.get_logger().info(f'Obstacle detected at {distance} meters, stopping.')
            self.stop_robot()
        elif self.current_goal is not None:
            self.move_based_on_distance(distance)


    def stop_robot(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher.publish(self.cmd)


    def move_based_on_distance(self, distance):
        if distance > 5:
            self.cmd.linear.x = 0.5
            self.cmd.angular.z = 0.5
        elif distance >= 0.5:
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = 0.0
        else:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
        self.publisher.publish(self.cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MoveToWallServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()
