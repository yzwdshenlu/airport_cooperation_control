#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import math

def deg2rad(degrees):
    return degrees * math.pi / 180.0

class CmdVel2Gazebo_Airplane:
    def __init__(self):
        rospy.init_node('cmdvel2gazebo_airplane', anonymous=True)
        robot_name = rospy.get_param('~topic_name')
        rospy.Subscriber('/{}/cmd_vel'.format(robot_name), Twist, self.callback, queue_size=1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_state_callback, queue_size=1)
        self.pub_steerL = rospy.Publisher(
            '/{}/front_left_velocity_controller/command'.format(robot_name),
            Float64,
            queue_size=1)
        self.pub_steerR = rospy.Publisher(
            '/{}/front_right_velocity_controller/command'.format(robot_name),
            Float64,
            queue_size=1)
        self.pub_rearL = rospy.Publisher(
            '/{}/rear_left_velocity_controller/command'.format(robot_name),
            Float64,
            queue_size=1)
        self.pub_rearR = rospy.Publisher(
            '/{}/rear_right_velocity_controller/command'.format(robot_name),
            Float64,
            queue_size=1)

        # initial velocity and tire angle are 0
        self.x = 0
        self.z = 0

        # car Wheelbase (in m)
        self.L = 1

        # car Tread
        self.T_front = 10
        self.T_rear = 10

        # how many seconds delay for the dead man's switch
        self.timeout = rospy.Duration.from_sec(0.2)
        self.lastMsg = rospy.Time.now()

        # maximum steer angle of the "inside" tire
        self.maxsteerInside = 3.1415926


        # turning radius for maximum steer angle just with the inside tire
        # tan(maxsteerInside) = wheelbase/radius --> solve for max radius at this angle
        rMax = self.L / math.tan(self.maxsteerInside)

        # radius of inside tire is rMax, so radius of the ideal middle tire (rIdeal) is rMax+treadwidth/2
        rIdeal = rMax + (self.T_front / 2.0)

        # maximum steering angle for ideal middle tire
        # tan(angle) = wheelbase/radius
        self.maxsteer = math.atan2(self.L, rIdeal)

        # set the maximun steering velocity in order not to make model shake
        self.angular_z_vel_limit = 2
        self.ajust = 0
        # loop
        rate = rospy.Rate(10)  # run at 10Hz
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def callback(self, data):
        # w = v / r
        self.x = data.linear.x
        # constrain the ideal steering angle such that the ackermann steering is maxed out
        self.z = data.angular.z
        self.lastMsg = rospy.Time.now()

    def model_state_callback(self, model_states_msg):
            try:
                model_index = model_states_msg.name.index('airplane_model')
                model_angular_vel_z = model_states_msg.twist[model_index].angular.z

                if abs(model_angular_vel_z) > self.angular_z_vel_limit:
                    self.adjust = 1

            except ValueError:
                pass 

    def publish(self):
        # now that these values are published, we
        # reset the velocity, so that if we don't hear new
        # ones for the next timestep that we time out; note
        # that the tire angle will not change
        # NOTE: we only set self.x to be 0 after 200ms of timeout
        delta_last_msg_time = rospy.Time.now() - self.lastMsg
        msgs_too_old = delta_last_msg_time > self.timeout
        if msgs_too_old:
            self.x = 0
            msgRear = Float64()
            msgRear.data = self.x
            self.pub_rearL.publish(msgRear)

            self.pub_rearR.publish(msgRear)

            msgSteer = Float64()
            msgSteer.data = 0
            self.pub_steerL.publish(msgSteer)

            self.pub_steerR.publish(msgSteer)

            return

        if abs(self.z) >= 0.1 or abs(self.x) >= 0.1:
            T_rear = self.T_rear
            T_front = self.T_front
            L = self.L
            adjust = self.ajust

            msgRight = Float64()
            msgRight.data = self.x * 3 + self.z * 1.2 - 5 * adjust
            self.pub_rearR.publish(msgRight)

            self.pub_steerR.publish(msgRight)

            msgLeft = Float64()
            msgLeft.data = self.x * 3 - self.z * 1.2 - 5 * adjust
            self.pub_rearL.publish(msgLeft)

            self.pub_steerL.publish(msgLeft)

        else:
            # if we aren't turning
            msgSpeed = Float64()
            msgSpeed.data = 0
            self.pub_rearL.publish(msgSpeed)

            self.pub_rearR.publish(msgSpeed)

            self.pub_steerL.publish(msgSpeed)

            self.pub_steerR.publish(msgSpeed)



if __name__ == '__main__':
    try:
        CmdVel2Gazebo_Airplane()

    except rospy.ROSInterruptException:
        pass
