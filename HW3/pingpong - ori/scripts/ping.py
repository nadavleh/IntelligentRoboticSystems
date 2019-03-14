#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def callback(data):
        pub = rospy.Publisher('pong', String, queue_size=10)
        rospy.loginfo(data.data)
	rate = rospy.Rate(1) # 10hz
  	rate.sleep()
	data.data="Ping"
        pub.publish(data)

def ping():
    rospy.init_node('ping', anonymous=True)
    rospy.Subscriber("ping", String, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        ping()
    except rospy.ROSInterruptException:
        pass

