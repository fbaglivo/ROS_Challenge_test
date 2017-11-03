#! /usr/bin/env python
import rospy
import rostopic
import rosbag
from bag_tools.msg import bagInfo
from bag_tools.srv import *

import threading
import os
import sys

## Error handling exceptions
class BagNotPresent(Exception):
	def __init__(self, bag_id):
		self.bag_id = bag_id
	def __str__(self):
		return "Bag id: " + str(bag_id) + " has not been started."

class PathNotPresent(Exception):
	def __init__(self, path):
		self.path = path
	def __str__(self):
		return "The path " + self.path + " does not exist and was not asked to be created."

class BagAlreadyStarted(Exception):
	def __init__(self, bag_name):
		self.bag_name = bag_name
	def __str__(self):
		return "The bag " + str(bag_name) + " has already been started."

# Description: Meant to house and perform thread-safe operations on
#	the collection of bags
class BagManager:
	def __init__(self):
		rospy.logdebug("Bag manager class instantiated")
		self.bag_dict = {}
		self.completed_bag_dict = {}
		self.next_handle = 0
		self.handle_lock = threading.Lock()

	def get_next_id(self):
		self.handle_lock.acquire()
		handle = str(self.next_handle)
		self.next_handle += 1
		self.handle_lock.release()
		
		return handle

	def add_bag(self, bag_instance):
		self.bag_dict[bag_instance.get_id()] = bag_instance

	def stop_bag(self, bag_id):
		try:
			self.completed_bag_dict[bag_id] = self.bag_dict[bag_id]
			self.completed_bag_dict[bag_id].stop_record()
		except KeyError:
			raise BagNotPresent(bag_id)

	def list_bags(self):
		return (self.get_bag_infos(self.bag_dict), self.get_bag_infos(self.completed_bag_dict))

	def get_bag_infos(self, bag_dict):
		bag_list = []
		for k in self.bag_dict:
			bag_data = bagInfo()
			bag_data.topic_list = bag_dict[k].get_topic_list()
			bag_data.location = bag_dict[k].get_file_location()
			bag_data.start_time = bag_dict[k].get_start_time()
			bag_data.end_time = bag_dict[k].get_end_time()
			bag_list.append(bag_data)
		return bag_list
		


# Description: Holds data relevant to a recording instance
# Performs raw operations on bags
class Recorder:
	def __init__(self, bag_id, bag_path, topic_list, make_path=False):
		rospy.loginfo("Recorder class instantiated")
		self.bag_id = bag_id
		self.bag_file = None
		self.bag_lock = threading.Lock()
		self.bag_path = bag_path
		self.make_path = make_path
		self.topic_list = topic_list
		self.topic_sub_list = []
		self.start_time = None
		self.end_time = None

		self.init_complete = False
		self.start_record()
		self.init_complete = True

	def get_id(self):
		return self.bag_id

	def get_topic_list(self):
		return self.topic_list

	def get_file_location(self):
		return self.bag_path

	def get_start_time(self):
		return self.start_time

	def get_end_time(self):
		if self.end_time == None:
			return rospy.Duration(-1)
		else:
			return self.end_time

	def start_record(self):
		if not os.path.exists(os.path.dirname(self.bag_path)):
			if self.make_path:
				os.makedirs(os.dirname(self.bag_path))
			else:
				raise PathNotPresent("Hosting directory does not exist.")

		if os.path.exists(self.bag_path):
			raise BagAlreadyStarted("Bag file with path " + self.bag_path + " already exists.")
		
		rospy.loginfo("Starting bag " + self.bag_path)
		self.bag_file = rosbag.Bag(self.bag_path, "w")

		self.subscribe_topics()
		self.start_time = rospy.Time.now()
		rospy.loginfo("Bag " + self.bag_path + " fully recording.")

	def subscribe_topics(self):
		for topic in self.topic_list:
			rospy.loginfo("Subscribing to topic: '" + topic + "'")
			topic_class = rostopic.get_topic_class(topic, blocking=False)[0]
			if topic_class == None:
				rospy.logerr("Topic is not yet instantiated. Skipping.")
				self.topic_list.remove(topic)
				continue

			print "Data class: ", topic_class
			topic_sub = rospy.Subscriber(topic, topic_class, self.generic_cb, callback_args=topic)
			self.topic_sub_list.append(topic_sub)

	def generic_cb(self, msg, topic):
		# Discard messages if the initialization isnt complete
		if not self.init_complete:
			return

		self.bag_lock.acquire()
		self.bag_file.write(topic, msg)
		self.bag_lock.release()

	def stop_record(self):
		for sub in self.topic_sub_list:
			sub.unregister()

		self.end_time = rospy.Time.now()
		self.bag_file.close()
		del self.bag_lock



def handle_start_record_srv(req):
	global bag_mgr
	rospy.loginfo('Received request to record: ' + str(req.topic_list))

	try:
		new_bag = Recorder(bag_mgr.get_next_id(), req.bag_path, req.topic_list, make_path=req.create_path)
	except PathNotPresent as e:
		print e
		return recordStartResponse(recordStartResponse.PATH_NOT_CREATED, '')
	except BagAlreadyStarted as e:
		print e
		return recordStartResponse(recordStartResponse.BAG_ALREADY_STARTED, '')

	bag_mgr.add_bag(new_bag)
	return recordStartResponse(recordStartResponse.SUCCESS, new_bag.get_id())

def handle_stop_record_srv(req):
	global bag_mgr
	rospy.loginfo('Stopping to bag: ' + req.bag_id)
	
	try:
		bag_mgr.stop_bag(req.bag_id)
	except BagNotPresent as e:
		print e
		return recordStopResponse(recordStopResponse.NO_BAG_FOUND)

	return recordStopResponse(0)

def handle_list_bags_srv(req):
	global bag_mgr
	rospy.loginfo("Listing bags")
	
	return listBagsResponse(*bag_mgr.list_bags())

if __name__ == "__main__":
	rospy.init_node("rosbag_recorder") #If you need multiple recorders, use separate name spaces
	rospy.loginfo("Bag tools recorder online.")

	# Bag manager
	bag_mgr = BagManager()

	# Create ROS API
	bag_start_srv = rospy.Service('start_record', recordStart, handle_start_record_srv)
	bag_stop_srv = rospy.Service('stop_record', recordStop, handle_stop_record_srv)
	bag_list_srv = rospy.Service('list_bags', listBags, handle_list_bags_srv)

	rospy.spin()
