#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from threading import Thread
from geometry_msgs.msg import Twist
import time


class ShapeController(Node):
    def __init__(self, bot_name, cmd_vel_topic):
        super().__init__(f'{bot_name}_Controller_node')
        self.bot_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.vel = Twist()
        
    def line(self, linear_x, linear_y):
        while rclpy.ok():
            self.vel.linear.y = -linear_x
            self.bot_pub.publish(self.vel)


    def square(self, linear_x, linear_y):
        #square motion logic 
        while rclpy.ok():
            delay = 3.0
            self.vel.linear.x = 0.0
            self.vel.linear.y = linear_y
            self.bot_pub.publish(self.vel)
            time.sleep(delay)#increase if needed
            
            self.vel.linear.x = 0.0
            self.vel.linear.y = 0.0
            self.bot_pub.publish(self.vel)
            

            self.vel.linear.x = linear_x
            self.vel.linear.y = 0.0
            self.bot_pub.publish(self.vel)
            time.sleep(delay)
            
            self.vel.linear.x = 0.0
            self.vel.linear.y = 0.0
            self.bot_pub.publish(self.vel)
            

            self.vel.linear.y = -linear_y
            self.vel.linear.x = 0.0
            self.bot_pub.publish(self.vel)
            time.sleep(delay)
            
            self.vel.linear.x = 0.0
            self.vel.linear.y = 0.0
            self.bot_pub.publish(self.vel)

            self.vel.linear.x = -linear_x
            self.vel.linear.y = 0.0
            self.bot_pub.publish(self.vel)
            time.sleep(delay)
            
            self.vel.linear.x = 0.0
            self.vel.linear.y = 0.0
            self.bot_pub.publish(self.vel)


    def triangle(self, linear_x, linear_y):
        # triangle motion logic
        while rclpy.ok():
            self.vel.linear.x = linear_x
            self.vel.linear.y = linear_y
            self.bot_pub.publish(self.vel)
            time.sleep(3.0)

            self.vel.linear.x = linear_x
            self.vel.linear.y = -linear_y
            self.bot_pub.publish(self.vel)
            time.sleep(3.0)

            self.vel.linear.x = -linear_x
            self.vel.linear.y = 0.0
            self.bot_pub.publish(self.vel)
            time.sleep(3.0)

    def circle(self, linear_x, angular_z):
        #circle motion logic
        while rclpy.ok():
            self.vel.linear.x = linear_x
            self.vel.angular.z = angular_z
            self.bot_pub.publish(self.vel)

def main(args=None):
    rclpy.init(args=args)
    
    bot1_controller = ShapeController('Bot1', '/cmd_vel/bot1')
    bot2_controller = ShapeController('Bot2', '/cmd_vel/bot2')
    bot3_controller = ShapeController('Bot3', '/cmd_vel/bot3')
    
    executor = MultiThreadedExecutor()
    executor.add_node(bot1_controller)
    executor.add_node(bot2_controller)
    executor.add_node(bot3_controller)

    try:
        # Run each node's motion logic in a separate thread
        thread1 = Thread(target=bot1_controller.square, args=(25.0, 25.0,))
        thread2 = Thread(target=bot2_controller.triangle, args=(43.8, 25.0))
        thread3 = Thread(target=bot3_controller.circle, args=(20.0, 20.0))
        
        thread1.start()
        thread2.start()
        thread3.start()
        
        executor.spin()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
