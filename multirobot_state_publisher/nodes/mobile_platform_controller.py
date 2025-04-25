#! /usr/bin/env python3

import threading
import rospy
from geometry_msgs.msg import Twist
import actionlib

import multirobot_actions.msg

class MobilePlatformControllerServer():
    """
    Mobile platform movement controller
    """
    def __init__(self, name):
        rospy.init_node(name, anonymous=False)

        self.constant_velocity = 0.000725 # [m/s]
        self.speed = 0.025 # [m/s]

        self.rate = rospy.Rate(10) # 10 Hz

        self.velocity_publisher = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()

        self.distance_x = 0
        self.tetha = 0
        self.command = None
        self.input_received = threading.Event()
        
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

    def constant_publisher(self):
        while not rospy.is_shutdown():
            if not self.input_received.is_set():
                self.vel_msg.linear.x = -abs(self.constant_velocity)
                self.vel_msg.linear.y = 0
                self.vel_msg.linear.z = 0
                self.vel_msg.angular.x = 0
                self.vel_msg.angular.y = 0
                self.vel_msg.angular.z = 0

                self.velocity_publisher.publish(self.vel_msg)
            self.rate.sleep()

    def publish_translation(self):
        self.t0 = rospy.Time.now().to_sec()
        self.current_distance = 0

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
            self.rate.sleep()

        # Reset input_received flag to allow constant_publisher to resume
        self.input_received.clear()

    def rotation_controller_client(self):
        client = actionlib.SimpleActionClient('mobile_platform_rotation_controller_action', multirobot_actions.msg.traj_planAction)

        # Wait until the action server has started up and started listening for goals.
        client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = multirobot_actions.msg.traj_planGoal(start = True,
                                                linear = 0,
                                                tetha = self.tetha
                                                )

        # Send the goal to the action server
        client.send_goal(goal)
        # Wait for the server to finish performing the action
        client.wait_for_result()

        # Reset input_received flag to allow constant_publisher to resume
        self.input_received.clear()


    def user_input(self):
        while not rospy.is_shutdown():
            input_command = input("Enter command and value (e.g., 'linear 0.3' or 'tetha 30'): ")
            parts = input_command.split()
            if len(parts) == 2:
                self.command = parts[0].lower()
                try:
                    value = float(parts[1])
                    if self.command == 'linear':
                        self.distance_x = value
                    elif self.command == 'tetha':
                        self.tetha = value
                    else:
                        rospy.logerr("Invalid command. Please enter 'linear' or 'theta' as the command.")
                        continue
                    # Set input_received flag to interrupt constant_publisher
                    self.input_received.set()
                except ValueError:
                    rospy.logerr("Invalid input. Please enter a valid translation.")
            else:
                rospy.logerr("Invalid input format. Please enter command and value separated by space.")

    def run(self):
        try:
            publish_thread = threading.Thread(target=self.constant_publisher)
            publish_thread.start()

            # Listen for user input
            input_thread = threading.Thread(target=self.user_input)
            input_thread.start()

            while not rospy.is_shutdown():
                if self.input_received.is_set():
                    if self.command == 'linear':
                        self.publish_translation()
                    else:
                        self.rotation_controller_client()

        except rospy.ROSInterruptException:
            pass

def main():
    node = MobilePlatformControllerServer('mobile_platform_controller_node')
    node.run()

if __name__ == '__main__':
    main()
    
    