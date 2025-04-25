#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import actionlib
from nav_msgs.msg import Odometry
from multirobot_actions.msg import traj_planAction, traj_planResult

class MoveMobileLocalServer():
    def __init__(self, name):
        self.speed = 0.1 # [m/s]
        
        self.pose = Odometry()
        self.message_received = False  # Flag to indicate whether a message has been received
        self.EKF_subscriber = rospy.Subscriber('/odometry/filtered', Odometry, self.callback)
        
        self.velocity_publisher = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()

        # Action Server Initialization part
        self._action_name = name

        self._as = actionlib.SimpleActionServer(self._action_name,      # Server name string
                                    traj_planAction,                # Action message type
                                    self.publish_velocity,              # Action Function
                                    auto_start = False 
                                    )
        
        self._as.start()
        
    def callback(self, msg):
        self.pose = msg
        self.message_received = True  # Set the flag to True when a message is received

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
            _result = traj_planResult()

            self.current_distance = 0.0
            self.distance_x = goal.linear # [m]
            x_0 = self.pose.pose.pose.position.x

            if self.distance_x < 0:
                self.move_backward()
            else:
                self.move_forward()
            
            while(self.current_distance < abs(self.distance_x)):
                #Publish the velocity
                self.velocity_publisher.publish(self.vel_msg)

                x_current = self.pose.pose.pose.position.x

                self.current_distance = abs(x_current - x_0)

                print(self.current_distance)

            if self.current_distance >= self.distance_x:
                _result.finish = True
                rospy.loginfo('%s: Operation Result: Succeeded' % self._action_name)
                self._as.set_succeeded(_result, "True")

            else:
                _result.finish = False
                rospy.loginfo('%s: Operation Result: Failed' % self._action_name)
                self._as.set_aborted(_result, "False")
        

if __name__ == '__main__':
    rospy.init_node('move_mobile_localization_action')
    server = MoveMobileLocalServer(rospy.get_name())
    rospy.spin()
    
    