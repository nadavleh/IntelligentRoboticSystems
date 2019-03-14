#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def callback(data):
        pub = rospy.Publisher('ping', String, queue_size=10)
        rospy.loginfo(data.data)
	rate = rospy.Rate(1) # 10hz
   	rate.sleep()
	data.data="Pong"
        pub.publish(data)

def pong():
    rospy.init_node('pong', anonymous=True)
    rospy.Subscriber("pong", String, callback)
    rospy.spin()
if __name__ == '__main__':
    try:
        pong()
    except rospy.ROSInterruptException:
        pass
