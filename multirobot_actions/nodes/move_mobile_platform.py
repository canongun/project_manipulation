#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class movement:
    def __init__(self):
        self.speed = 0.1 # [m/s]
        self.distance = 0.5 # [m]

        rospy.init_node('mobile_linear', anonymous=False)
        self.velocity_publisher = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()
        
    def move_forward(self):
        self.vel_msg.linear.x = abs(self.speed)
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0

    def move_backward(self):
        self.vel_msg.linear.x = -abs(self.speed)
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0

    def stop(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0

        
    def publish_velocity(self):
        #Setting the current time for distance calculus
        self.t0 = rospy.Time.now().to_sec()
        self.current_distance = 0
        self.stop()
        
        #Loop to move the turtle in an specified distance
        while(self.current_distance < self.distance):
            #Publish the velocity
            self.velocity_publisher.publish(self.vel_msg)
            #Takes actual time to velocity calculus
            self.t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            self.current_distance= self.speed*(self.t1-self.t0)
            print("\ncurrent_distance\n",self.current_distance)
        
        self.t0 = rospy.Time.now().to_sec()
        self.current_distance = 0
        self.move_backward()
        
        while(self.current_distance < self.distance):
            print("HERE")
            #Publish the velocity
            self.velocity_publisher.publish(self.vel_msg)
            #Takes actual time to velocity calculus
            self.t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            self.current_distance= self.speed*(self.t1-self.t0)
            print("\ncurrent_distance\n",self.current_distance)
        
        
    
        

            
if __name__ == '__main__':
    move = movement()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        move.publish_velocity()
        rate.sleep()
    
    