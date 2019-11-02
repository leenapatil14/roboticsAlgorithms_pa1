#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def talker():
    pub = rospy.Publisher('homing_signal', PoseStamped, queue_size=10)
    rospy.init_node('homing_beacon', anonymous=False)
    rate = rospy.Rate(10) # 10hz

    home_pose = PoseStamped()
    home_pose.header.frame_id = "odom"

    home_pose.pose.position.x = 4
    home_pose.pose.position.y = 2
    home_pose.pose.position.z = 0

    
    while not rospy.is_shutdown():
        home_pose.header.stamp = rospy.Time.now()

        pub.publish(home_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass