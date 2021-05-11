#!/usr/bin/env python


from binpicking_msgs.srv import CapturePointcloud, CapturePointcloudRequest, CapturePointcloudResponse
import rospy
import copy
import numpy as np
import tf2_ros
import geometry_msgs.msg
import tf_conversions

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2

pcl_publisher = 0
capture = 0
tfBuffer = 0
listener = 0
pcl_publisher = 0
broadcaster = 0
transform_capture = 0

def capture_cb(request):
    rospy.loginfo('capturing')
    global capture
    capture = copy.deepcopy(request)
    capture.header.frame_id = 'camera_depth_optical_frame_capture'

    transform = tfBuffer.lookup_transform("base_link", "camera_depth_optical_frame", rospy.Time())#now)
#    print(trans)
    global transform_capture
    transform_capture = copy.deepcopy(transform)

    transform_capture.header.stamp = rospy.Time.now()
    transform_capture.header.frame_id = "base_link"
    transform_capture.child_frame_id = "camera_depth_optical_frame_capture"

    pcl_publisher.publish(capture)
    broadcaster.sendTransform(transform_capture)

# Service callback function.
def process_service_request(req):

    # Instantiate the response message object.
    res = CapturePointcloudResponse()
    res.success = False

    rospy.loginfo('Capture pointcloud')

    sb = rospy.Subscriber('/camera/depth/color/points', PointCloud2, capture_cb)
    rospy.wait_for_message('/camera/depth/color/points', PointCloud2)
    sb.unregister()

    #Return the response message.
    global transform_capture
    global capture
    res.pointcloud = capture
#    res.camera_pose.position = transform_capture.transform.translation
#    res.camera_pose.orientation = transform_capture.transform.rotation
    res.success = True
    return res

def capture_pointcloud_server():
    # ROS node for the service server.
    rospy.init_node('capture_pointcloud_server', anonymous = False)

    global tfBuffer
    tfBuffer = tf2_ros.Buffer()
    global listener
    listener = tf2_ros.TransformListener(tfBuffer)

    global pcl_publisher
    pcl_publisher = rospy.Publisher('/camera/depth/color/capture', PointCloud2, queue_size = 10)

    global broadcaster
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Create a ROS service type.
    service = rospy.Service('capture_pointcloud', CapturePointcloud, process_service_request)

    # Log message about service availability.
    rospy.loginfo('Capture pointcluod service is now available.')
    rospy.spin()

if __name__ == "__main__":
    capture_pointcloud_server()
