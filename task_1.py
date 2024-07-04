#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from functools import partial
from turtlesim.msg import Pose
import math
import time


class CircleTurtle(Node):

    def __init__(self):
        super().__init__('circle_turtle')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.callback, 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.angular_speed = 0.5  
        self.linear_speed = 0.5   

        self.angular_speed_2 = -0.8
        self.linear_speed_2 = 1.2

        self.circle_duration = 6.28 / 0.5 + 0.4
        self.circle_completed = False
        self.turtle_spawn = False

        self.last_x = 0
        self.last_y = 0
        self.time = 0

    def callback(self, msg):
        self.last_x = msg.x
        self.last_y = msg.y

    def timer_callback(self):
        vel_msg = Twist()
        vel_msg.linear.x = self.linear_speed
        vel_msg.angular.z = self.angular_speed
        self.publisher.publish(vel_msg)

        if not self.circle_completed:
            self.timer_2 = self.create_timer(self.circle_duration, self.stop_turtle)
            self.timer_2 = self.create_timer(self.circle_duration + 6.28 / 0.8, self.stop_turtle_2)
            self.circle_completed = True

        if self.circle_completed:
            self.turtle2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
            vel_msg = Twist()
            vel_msg.linear.x = self.linear_speed_2
            vel_msg.angular.z = self.angular_speed_2
            self.turtle2.publish(vel_msg)

    def spawn_service(self, x, y, name):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for spawn service")
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_spawn))

    def callback_spawn(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))

        if self.circle_completed:
            pass


    def stop_turtle(self):
        vel_msg = Twist()
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.publisher.publish(vel_msg)
        self.get_logger().info('Full circle completed. Stopping the turtle.')
        self.spawn_service(self.last_x, self.last_y, 'turtle2')
        self.time = time.time()
        self.turtle_spawn = True

    def stop_turtle_2(self):
        vel_msg = Twist()
        self.linear_speed_2 = 0.0
        self.angular_speed_2 = 0.0
        self.turtle2.publish(vel_msg)
        self.get_logger().info('Full circle completed. Stopping the turtle.')

def main(args=None):
    rclpy.init(args=args)
    circle_turtle = CircleTurtle()
    rclpy.spin(circle_turtle)
    circle_turtle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

