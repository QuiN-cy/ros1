#!/usr/bin/env python
import rospy
import sys
import moveit_commander
import copy
import actionlib
import moveit_msgs
import sensor_msgs
from moveit_python import *
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient, ProxyServiceCaller

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes, RobotState, ExecuteTrajectoryGoal, ExecuteTrajectoryAction
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
from moveit_msgs.srv import GetCartesianPath, GetCartesianPathRequest


print("hallo")
