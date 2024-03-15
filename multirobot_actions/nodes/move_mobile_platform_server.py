#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import actionlib
from multirobot_actions.msg import move_platformAction, move_platformResult

class MoveMobileServer():
    def __init__(self, name):
        self.speed = 0.1 # [m/s]

        self.velocity_publisher = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()

        # Action Server Initialization part
        self._action_name = name

        self._as = actionlib.SimpleActionServer(self._action_name,      # Server name string
                                    move_platformAction,                # Action message type
                                    self.publish_velocity,              # Action Function
                                    auto_start = False 
                                    )
        
        self._as.start()
        
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

        
    def publish_velocity(self, goal):

        if goal.start:
            _result = move_platformResult()

            self.t0 = rospy.Time.now().to_sec()
            self.current_distance = 0
            self.distance_x = goal.goal_x # [m]

            if self.distance_x < 0:
                self.move_backward()
            else:
                self.move_forward()
            
            #Loop to move the turtle in an specified distance
            while(self.current_distance < abs(self.distance_x)):
                #Publish the velocity
                self.velocity_publisher.publish(self.vel_msg)
                #Takes actual time to velocity calculus
                self.t1=rospy.Time.now().to_sec()
                #Calculates distancePoseStamped
                self.current_distance= self.speed*(self.t1-self.t0)

            if self.current_distance >= self.distance_x:
                _result.finish = True
                rospy.loginfo('%s: Operation Result: Succeeded' % self._action_name)
                self._as.set_succeeded(_result, "True")

            else:
                _result.finish = False
                rospy.loginfo('%s: Operation Result: Failed' % self._action_name)
                self._as.set_aborted(_result, "False")
        
        
    
        

            
if __name__ == '__main__':
    # rospy.set_param('/robot_description', rospy.get_param('/fixed/robot_description'))
    rospy.init_node('move_mobile_action')
    server = MoveMobileServer(rospy.get_name())
    rospy.spin()
    
    