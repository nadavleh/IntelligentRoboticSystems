#!/usr/bin/env python
# license removed for brevity
import rospy
import curses, time
from geometry_msgs.msg import Twist


def main():
    	rospy.init_node('KBcommand', anonymous=True)
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	win = curses.initscr()
	char=''
	while char !='q':
		msg=Twist()
		char=chr(win.getch())

		if char=='w':
			msg.linear.x=2
		elif char=='s':
			msg.linear.x=-2
		elif char=='a':
			msg.angular.z=1
		elif char=='d':
			msg.angular.z=-1
		else:
			msg.linear.x=0
			msg.angular.z=0
        
		pub.publish(msg)
	curses.endwin()
    	#rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

