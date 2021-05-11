#!/usr/bin/env python
import rospy

from binpicking_msgs.srv import CalculateObjectposeFromPointcloud, CalculateObjectposeFromPointcloudRequest, CalculateObjectposeFromPointcloudResponse

from flexbe_core import EventState, Logger
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import PointCloud2

import copy


class CalculateObjectPoseState(EventState):
	'''
	Calcualtes the pose of a object from a pointcloud

	># pointcloud		PointCloud2		Pointcloud of the objects
	#> object_pose		PoseStamped		Pose of the detected object

	<= continue 						Given time has passed.
	<= failed 							Example for a failure outcome.

	'''

	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(CalculateObjectPoseState, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['pointcloud'], output_keys = ['object_pose'])

		# Store state parameter for later use.

		# The constructor is called when building the state machine, not when actually starting the behavior.
		# Thus, we cannot save the starting time now and will do so later.
		#rospy.loginfo("Waiting for service...")
		#rospy.wait_for_service('calculate_object_pose')
		# Create a service proxy.
		#self.calculate_object_pose = rospy.ServiceProxy('calculate_object_pose', CalculateObjectposeFromPointcloud)

	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.
		userdata.object_pose = copy.deepcopy(self.service_response.object_pose)
		return 'continue' # One of the outcomes declared above.


	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.

		# The following code is just for illustrating how the behavior logger works.
		# Text logged by the behavior logger is sent to the operator and displayed in the GUI.
		rospy.loginfo("Waiting for service...")
		rospy.wait_for_service('calculate_object_pose')
		# Create a service proxy.
		self.calculate_object_pose = rospy.ServiceProxy('calculate_object_pose', CalculateObjectposeFromPointcloud)


		request = CalculateObjectposeFromPointcloudRequest()
		request.pointcloud =  copy.deepcopy(userdata.pointcloud)
		try:
			# Call the service here.
			self.service_response = self.calculate_object_pose(request)

		except rospy.ServiceException, e:
			rospy.logerr("Service call failed")

		#pass # Nothing to do in this example.

	def on_exit(self, userdata):
		# This method is called when an outcome is returned and another state gets active.
		# It can be used to stop possibly running processes started by on_enter.

		pass # Nothing to do in this example.


	def on_start(self):
		# This method is called when the behavior is started.
		# If possible, it is generally better to initialize used resources in the constructor
		# because if anything failed, the behavior would not even be started.

		pass # Nothing to do in this example.


	def on_stop(self):
		# This method is called whenever the behavior stops execution, also if it is cancelled.
		# Use this event to clean up things like claimed resources.

		pass # Nothing to do in this example.
