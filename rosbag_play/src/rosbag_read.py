#! /usr/bin/env python


import rosbag
import itertools
import roslib 
import rospy
import numpy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

class GoToPose():
    def __init__(self):

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
	rospy.on_shutdown(self.shutdown)
	
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

if __name__ == '__main__':


    rospy.init_node('nav_test', anonymous=False)
    navigator = GoToPose()

    tr=numpy.zeros(3)
    rt=numpy.zeros(4)
   
    bag = rosbag.Bag('test.bag')
    A=bag.read_messages('/tf','numbers')

    for topic, msg, t in itertools.islice(bag.read_messages(topics=['/tf', 'numbers']),1):
        
        A=msg.transforms.pop(0)    
     #   tr[0]=A.transform.translation.x+1
     #   tr[1]=A.transform.translation.y
     #   tr[2]=A.transform.translation.z
     #   rt[0]=A.transform.rotation.x
     #   rt[1]=A.transform.rotation.y
     #   rt[2]=A.transform.rotation.z
     #   rt[3]=A.transform.rotation.w
     #   print tr, rt
        
        position = {'x': A.transform.translation.x, 'y' : A.transform.translation.y}
        quaternion = {'r1' : A.transform.rotation.x, 'r2' : A.transform.rotation.y, 'r3' : A.transform.rotation.z, 'r4' : A.transform.rotation.w}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position, quaternion)

        if success:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

        
    bag.close()
