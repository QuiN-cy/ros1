#!/usr/bin/env python


import roslib; roslib.load_manifest('robotiq_vacuum_grippers_control')
roslib.load_manifest('robotiq_modbus_rtu')
import rospy

import robotiq_vacuum_grippers_control.baseRobotiqVacuumGrippers
import robotiq_modbus_rtu.comModbusRtu
import os, sys
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from robotiq_vacuum_grippers_control.msg import _RobotiqVacuumGrippers_robot_input  as inputMsg
from robotiq_vacuum_grippers_control.msg import _RobotiqVacuumGrippers_robot_output as outputMsg


'''

Created on June 21 2020

@author: Gerard Harkema

'''

class EpickVacuumGripperControlState(EventState):
	'''
	State to control Robotiq Epick suction gripper"

	-- device			String		device name eg. /dev/ttyUSB0
	-- gripper_command		String		r: Reset; a: Activate; g: Grip; c: Release
	-- setteling_time 		float 		Time which needs to have passed since the behavior started.
	<= continue 					if the gripper activation or de-activation has been succesfully achieved
	<= failed 						otherwise

	'''

	def __init__(self, device, gripper_command, setteling_time):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(EpickVacuumGripperControlState, self).__init__(outcomes = ['continue', 'failed'])

		# Store state parameter for later use.
		self._target_time = rospy.Duration(setteling_time)

		# The constructor is called when building the state machine, not when actually starting the behavior.
		# Thus, we cannot save the starting time now and will do so later.
		self._start_time = None
		
		#Gripper is a Vacuum with a TCP connection
		self._gripper = robotiq_vacuum_grippers_control.baseRobotiqVacuumGrippers.robotiqbaseRobotiqVacuumGrippers()
		self._gripper.client = robotiq_modbus_rtu.comModbusRtu.communication()

                #We connect to the address received as an argument
                self._connected = self._gripper.client.connectToDevice(device)

		
	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.

		if not self._valid_command:
			rospy.logerr('invalid gripper command')
			return 'failed'

		if rospy.Time.now() - self._start_time > self._target_time:
			return 'continue' # One of the outcomes declared above.


	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.

	        command = outputMsg.RobotiqVacuumGrippers_robot_output()
	        
	        char = gripper_command[0]
	        valid_command = False
		if char == 'a':
			command.rACT = 1
			command.rMOD = 0
			command.rGTO = 1
			command.rPR = 0
			command.rSP  = 150
			command.rFR  = 50
			rospy.loginfo('activate gripper')
			self._valid_command = True
		
		if char == 'r':
			command.rACT = 0
			rospy.loginfo('reset gripper')
			self._valid_command = True

		if char == 'g':
			command.rPR = 0
			rospy.loginfo('grasp gripper')
			self._valid_command = True

		if char == 'c':
			command.rPR = 255   
			rospy.loginfo('release gripper')
			self._valid_command = True
        
	        if self._valid_command:
	        
			self._gripper.refreshCommand = command
			#Send the most recent command
			if self._connected:
	      			self._gripper.sendCommand()

			self._start_time = rospy.Time.now()
			time_to_wait = (self._target_time - (rospy.Time.now() - self._start_time)).to_sec()

			if time_to_wait > 0:
				Logger.loginfo('Need to wait for %.1f seconds.' % time_to_wait)
				Logger.loginfo('Executing vacuum gripper command')
		

	def on_exit(self, userdata):
		# This method is called when an outcome is returned and another state gets active.
		# It can be used to stop possibly running processes started by on_enter.

		pass # Nothing to do


	def on_start(self):
		# This method is called when the behavior is started.
		# If possible, it is generally better to initialize used resources in the constructor
		# because if anything failed, the behavior would not even be started.
		pass # Nothing to do

	def on_stop(self):
		# This method is called whenever the behavior stops execution, also if it is cancelled.
		# Use this event to clean up things like claimed resources.

		pass # Nothing to do
