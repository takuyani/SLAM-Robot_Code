#!/usr/bin/env python

"""@package ROS Node "vehicle_velocity"
"""
import rospy
from geometry_msgs.msg import Twist

class TeleopeVelocity(object):
    PARAM_NAME_LINEAR = '~linear'
    PARAM_NAME_ANGULAR = '~angular'
    LINEAR_VEL_DEF = 10.0  # linear velocity default value[mm/s]
    ANGULAR_VEL_DEF = 10.0  # angular velocity default value[deg/s]

    def __init__(self):
        rospy.loginfo('running node "vehicle_velocity"');
        self.pub = rospy.Publisher('dst/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('src/cmd_vel', Twist, self.callback)
        if not rospy.has_param(self.PARAM_NAME_LINEAR):
            rospy.set_param(self.PARAM_NAME_LINEAR, self.LINEAR_VEL_DEF)
        if not rospy.has_param(self.PARAM_NAME_ANGULAR):
            rospy.set_param(self.PARAM_NAME_ANGULAR, self.ANGULAR_VEL_DEF)

    def callback(self, twist):
        self.linear_mmps = rospy.get_param(self.PARAM_NAME_LINEAR)
        self.angular_dps = rospy.get_param(self.PARAM_NAME_ANGULAR)

        vel = Twist();
        vel.linear.x = self.linear_mmps * twist.linear.x
        vel.angular.z = self.angular_dps * twist.angular.z

        self.pub.publish(vel)
        rospy.loginfo('publish linear = ' + str(vel.linear.x) + '[mm/s], angular = ' + str(vel.angular.z) + '[deg/s]')

if __name__ == '__main__':
    rospy.init_node('teleope_velocity')
    teleope_velocity = TeleopeVelocity()
    rospy.spin()
