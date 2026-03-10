#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class TwistToWheels:
    def __init__(self):
        rospy.init_node('twist_to_wheels', anonymous=False)

        self.wheel_separation = rospy.get_param('~wheel_separation', 0.6775)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.178)
        self.min_angular = rospy.get_param('~min_angular', 0.45)

        # 6 wheel velocity controllers
        self.pub_fl = rospy.Publisher('wheel_vel_controller_fl/command', Float64, queue_size=10)
        self.pub_ml = rospy.Publisher('wheel_vel_controller_ml/command', Float64, queue_size=10)
        self.pub_rl = rospy.Publisher('wheel_vel_controller_rl/command', Float64, queue_size=10)
        self.pub_fr = rospy.Publisher('wheel_vel_controller_fr/command', Float64, queue_size=10)
        self.pub_mr = rospy.Publisher('wheel_vel_controller_mr/command', Float64, queue_size=10)
        self.pub_rr = rospy.Publisher('wheel_vel_controller_rr/command', Float64, queue_size=10)

        self.sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)

        rospy.loginfo("twist_to_wheels iniciado")

    def cmd_vel_callback(self, msg):
        linear_x = -msg.linear.x
        angular_z = msg.angular.z

        # Reforzar velocidad angular minima para vencer friccion
        if abs(angular_z) > 0.01 and abs(angular_z) < self.min_angular:
            angular_z = self.min_angular if angular_z > 0 else -self.min_angular

        v_left = (linear_x - angular_z * self.wheel_separation / 2.0) / self.wheel_radius
        v_right = (linear_x + angular_z * self.wheel_separation / 2.0) / self.wheel_radius

        self.pub_fl.publish(Float64(v_left))
        self.pub_ml.publish(Float64(v_left))
        self.pub_rl.publish(Float64(v_left))
        self.pub_fr.publish(Float64(v_right))
        self.pub_mr.publish(Float64(v_right))
        self.pub_rr.publish(Float64(v_right))

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = TwistToWheels()
        node.run()
    except rospy.ROSInterruptException:
        pass
