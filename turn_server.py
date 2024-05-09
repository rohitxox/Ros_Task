import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time
from custom_turtlebot_interface.srv import Turn

class TurnService(Node):
    def __init__(self):
        super().__init__('turn_service')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.srv = self.create_service(Turn, 'turn', self.turn_callback)

    def turn_callback(self, request, response):
        self.get_logger().info('Received movement request: %s' % request.movement)

        if request.movement == "Turn Right":
            self.turn_right()
            response.success = True
        elif request.movement == "Turn Left":
            self.turn_left()
            response.success = True
        elif request.movement == "Stop":
            self.stop_moving()
            response.success = True
        else:
            response.success = False
            self.get_logger().info('Invalid command.')

        return response

    def turn_right(self):
        angular_speed = 0.5  #radians per second
        turn_duration = math.pi / 2 / angular_speed  #duration for a 90 degree turn
        turn_command = Twist()
        turn_command.angular.z = -angular_speed
        self.get_logger().info('Turning right 90 degrees.')
        self.publisher_.publish(turn_command)
        time.sleep(turn_duration)
        self.stop_moving()

    def turn_left(self):
        angular_speed = 0.5  #radians per second
        turn_duration = math.pi / 2 / angular_speed
        turn_command = Twist()
        turn_command.angular.z = angular_speed
        self.get_logger().info('Turning left 90 degrees.')
        self.publisher_.publish(turn_command)
        time.sleep(turn_duration)
        self.stop_moving()

    def stop_moving(self):
        self.get_logger().info('Stopping movement.')
        self.publisher_.publish(Twist())  #Publish zero velocities to stop the robot

def main(args=None):
    rclpy.init(args=args)
    turn_service = TurnService()
    rclpy.spin(turn_service)
    turn_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
