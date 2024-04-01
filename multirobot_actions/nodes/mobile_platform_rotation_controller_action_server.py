#! /usr/bin/env python3

import rospy
import actionlib
import math
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from multirobot_actions.msg import traj_planAction, traj_planResult
from geometry_msgs.msg import Twist

MAX_ANG_VEL = 0.4
ANG_VEL_THRES = 0.00007
ΔΤ = 1/20

class RotationControllerServer():

    def __init__(self, name):

        # Controller values
        self.kP = 50
        self.kI = 24.5
        self.kD = 1.5  

        # Heading
        self.Θ = 0.0

        ## Maintaining values of the integrated control error and its derivative
        self.integral_ctrl_err = 0.0
        self.previous_ctrl_err = 0.0
        self.heading_error = 1.0

        self.heading_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)

        self.cmd_vel_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=1)
        # Action Server Initialization part
        self._action_name = name

        self._as = actionlib.SimpleActionServer(self._action_name,      # Server name string
                                    traj_planAction,                # Action message type
                                    self.publish_rotation,              # Action Function
                                    auto_start = False 
                                    )
        
        self._as.start()

    def imu_callback(self, msg):
        '''Call back function'''
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        (self.roll, self.pitch, self.Θ) = euler_from_quaternion(orientation_list)

    ## Control Function
    def heading_ctrl(self, meas, ref):
        target_rad = ref * math.pi / 180
        self.heading_error = target_rad - meas 
        
        ##  Transform the control error to the [-180, 180] interval and normalize it
        if self.heading_error > math.pi:
            self.heading_error -= 2 * math.pi
        elif self.heading_error < -math.pi:
            self.heading_error += 2 * math.pi
        self.heading_error /= math.pi
    
        cmd_angular_velocity = self.kP*self.heading_error + self.kI*self.integral_ctrl_err + self.kD * (self.heading_error-self.previous_ctrl_err)/ΔΤ

        rospy.loginfo("\nAngular Velocity BC={}\n".format(cmd_angular_velocity))

        if cmd_angular_velocity >= 0:
            if cmd_angular_velocity >= MAX_ANG_VEL:
                cmd_angular_velocity = MAX_ANG_VEL
        else:
            if cmd_angular_velocity < -MAX_ANG_VEL:
                cmd_angular_velocity = -MAX_ANG_VEL
          

        ## Anti Wind-Up
        if abs(cmd_angular_velocity) < MAX_ANG_VEL:
            self.integral_ctrl_err += (self.heading_error*ΔΤ)

        self.previous_ctrl_err = self.heading_error

        rospy.loginfo("Target={}  Current={}  Error={}\n".format(target_rad, meas, self.heading_error))

        return cmd_angular_velocity

    def publish_rotation(self, goal):

        if goal.start:
            _result = traj_planResult()

            rate = rospy.Rate(1/ΔΤ)

            while abs(self.heading_error) >= ANG_VEL_THRES:
                cmd_vel = Twist()
                # cmd_vel.angular.z = max(min(self.heading_ctrl(self.Θ, self.target), MAX_ANG_VEL), -MAX_ANG_VEL)
                cmd_vel.angular.z = self.heading_ctrl(self.Θ, goal.tetha)
                rospy.loginfo("\nAngular Velocity={}\n".format(cmd_vel.angular.z))
                self.cmd_vel_pub.publish(cmd_vel)
                rate.sleep()

            _result.finish = True
            rospy.loginfo('%s: Operation Result: Succeeded' % self._action_name)
            self._as.set_succeeded(_result, "True")

            
                 
def main():
    rospy.init_node("mobile_platform_rotation_controller_action")
    RotationControllerServer(rospy.get_name())
    rospy.spin()
            
if __name__ == '__main__':
    main()
    
    