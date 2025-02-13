########################################################################################################################
########################################## eYRC 23-24 Hologlyph Bots Task 1A ###########################################
# Team ID: 1476
# Team Leader Name: Mohit K
# Team Members Name: Puran Y,Sufiyan Ahmed,ViswaRaja S
# College: Hindusthan College Of Engineering and Technology
########################################################################################################################

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Spawn,SetPen
from geometry_msgs.msg import Twist
from functools import partial
import math

class DrawSnowManNode(Node):
    
    def __init__(self):
        super().__init__('draw_snowman')
        #creating publisher
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        #creating subscriber
        self.subscription_ = self.create_subscription(Pose,'turtle1/pose',self.pose_callback,10)
        print(f"Drawing SnowMan :)..")
        self.circle_completed_ = False
        self.radius_ = 1.0  #radius 
        self.linear_speed_ = 0.8
        self.start_pose_ = None
        self.i=0

    def pose_callback(self, pose):
        if not self.circle_completed_:
            twist_msg = Twist()

            if self.start_pose_ is None:
                self.start_pose_ = pose

            current_pose = pose

            # Calculate the angle difference between the current and starting pose
            radian_diff = abs (math.atan2(
                
                current_pose.y - self.start_pose_.y,
                current_pose.x - self.start_pose_.x
            ))
            
            
            self.angular_speed = self.linear_speed_ / self.radius_
            # Calculating angular velocity from linear speed & radius
            twist_msg.angular.z = self.angular_speed


            # Assigning linear velocity to maintain forward motion
            twist_msg.linear.x = self.linear_speed_

            # Publishing the twist message
            self.publisher_.publish(twist_msg)
            
            
            pi = round(math.pi,1)
            
            if current_pose.y < 5.5 and self.start_pose_.y > 5.5:
                self.pen_width(179,184,255,5,0)
            #checks if circle is complete or not
            if radian_diff >= pi:
                self.circle_completed_ = True
                self.stop_func()
                print(f"Circle {self.i} Completed")
                #self.get_logger().info('Circle {self.i}completed')
                while self.i<1:
                    self.i+=1
                    self.spawn_turtle(x=5.544445,y=5.544445,theta=0.0,name="turtle2")
                    self.reset_circle(new_radius=-1.7) #calling pose_callback for drawing another circle
                self.stop_func()
                
                
            
    #pen service to change width of the second circle
    def pen_width(self,r,g,b,width,off):
        client = self.create_client(SetPen,"/turtle1/set_pen")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Pen Service")
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off
        
        future =client.call_async(request)
        future.add_done_callback(partial(self.callback_setpen)) 
        
    #callback for penservice    
    def callback_setpen(self,future):
        try:
            response = future.result()
        except Exception as e :
            self.get_logger().error("Service call failed: %r" %(e,))
            
    #spawn service                    
    def spawn_turtle(self,x,y,theta,name):
        client = self.create_client(Spawn,"/spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Spawn Service.....")
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name
        
        future = client.call_async(request)            
        future.add_done_callback(partial(self.callback_spawn))
        
        
    #spawn callback
    def callback_spawn(self, future):
        try:
            response = future.result()
        except Exception as e :
            self.get_logger().error("Service call failed : %r" % (e,))


#Stop function for stopping turtles
    
    def stop_func(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0       
        twist_msg.angular.z = 0.0
        self.publisher_.publish(twist_msg)
        
#resetting the circle to draw second circle

    def reset_circle(self, new_radius):
        self.circle_completed_ = False
        self.radius_ = new_radius
        self.start_pose_ = None
        
#main function
def main(args=None):
    rclpy.init(args=args)

    node = DrawSnowManNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()