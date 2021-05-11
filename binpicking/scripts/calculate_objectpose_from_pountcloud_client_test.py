#! /usr/bin/env python
import rospy
import copy
import tf2_ros
import geometry_msgs.msg
import tf_conversions

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2

pcl_publisher = 0
capture = 0;

def capture_cb(request):
    rospy.loginfo('capturing')
#    capture = PointCloud2() # is deze nodig?
    global capture
    capture = copy.deepcopy(request)
    capture.header.frame_id = 'camera_depth_optical_frame_capture'

def caputere():
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    pcl_publisher = rospy.Publisher('/camera/depth/color/capture', PointCloud2, queue_size = 10)

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    rate = rospy.Rate(1000)
    rate.sleep()



    sb = rospy.Subscriber('/camera/depth/color/points', PointCloud2, capture_cb)
    rospy.wait_for_message('/camera/depth/color/points', PointCloud2)
    sb.unregister()

#    try:
    now = rospy.Time.now()
    trans = tfBuffer.lookup_transform("base_link", "camera_depth_optical_frame", rospy.Time())#now)
    print(trans)
    t = copy.deepcopy(trans)
#        t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"
    t.child_frame_id = "camera_depth_optical_frame_capture"

    broadcaster.sendTransform(t)

#    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#        rospy.loginfo('unable to lookup transform')


    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pcl_publisher.publish(capture)
        rate.sleep()

if __name__ == "__main__":

    rospy.init_node('capture_pountcloud', anonymous = False)
    rospy.loginfo('capturing started')
#    global pcl_publisher
#    pcl_publisher = rospy.Publisher('/camera/depth/color/capture', PointCloud2, queue_size=10)

#   rospy.wait_for_massage('/camera/depth/color/points', point_cloud2)
    caputere()



    rospy.spin()
