#!/usr/bin/env python

import sys
import rospy
from binpicking_msgs.srv import CapturePointcloud, CapturePointcloudRequest, CapturePointcloudResponse

def capture_pointcloud_client():
    # First wait for the service to become available.
    rospy.loginfo("Waiting for service...")
    rospy.wait_for_service('capture_pointcloud')
    try:
        # Create a service proxy.
        capture_pointcloud = rospy.ServiceProxy('capture_pointcloud', CapturePointcloud)

        # Call the service here.
        service_response = capture_pointcloud(0)

        # Return the response to the calling function.
        return service_response

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":

    # Initialize the client ROS node.
    rospy.init_node("capture_pointcloud_client", anonymous = False)

    # Call the service client function.
    service_response = capture_pointcloud_client()

    # Process the service response and display log messages accordingly.
    if(not service_response.success):
        rospy.logerr("Capture pointcloud unsuccessful!")
    else:
       rospy.loginfo("Capture pointcloud successful!")
#       print(service_response.camera_pose)
