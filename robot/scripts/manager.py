#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import dweepy

# command keywords
CMD_GOTO = "goto"
CMD_STANDBY = "standby"

# error keywords
ROS_ERROR = "[ROS ERROR]"

# success keywords
SUCCESS = "success"

busy = False

COMMAND_IOT_NAME = 'turtlebot_patrol_command'
STATUS_IOT_NAME = 'turtlebot_patrol_status'

prev_goal = None

def callback(data):
    global busy
    if(SUCCESS in data.data or ROS_ERROR in data.data):
		busy = False
		rospy.loginfo("[patrol_manager]" + 'received status report: %s', data.data)   
		dweepy.dweet_for(STATUS_IOT_NAME,{'status':str(data.data) + ' ' + str(prev_goal)})

def manager():
	global busy
	global prev_goal
	# send goal to robot
	pub = rospy.Publisher(COMMAND_IOT_NAME, String, queue_size=10)
	# receive robot status report
	rospy.Subscriber(STATUS_IOT_NAME, String, callback)
	rospy.init_node('patrol_manager', anonymous=True)
	rate = rospy.Rate(1)
	command = None
	while not rospy.is_shutdown():
		if not busy:
			try:
				command = dweepy.get_latest_dweet_for(COMMAND_IOT_NAME)[0]['content']
			except(dweepy.DweepyError):
				pass
			if(CMD_STANDBY in command.keys()):
				rospy.loginfo("[patrol_manager] standing by...")
				pass
			if(CMD_GOTO in command.keys()):
				goal = command[CMD_GOTO]	
				if(goal == prev_goal):
					    rospy.loginfo("[patrol_manager] standing by...")
					    busy = False
				else:
					busy = True
					prev_goal = goal
					pub.publish(goal)
					rospy.loginfo("[patrol_manager] send goal: " + goal + ". Waiting for status report from robot...")
		rate.sleep()

				
if __name__ == '__main__':
	try:
		manager()
	except rospy.ROSInterruptException:
		pass
