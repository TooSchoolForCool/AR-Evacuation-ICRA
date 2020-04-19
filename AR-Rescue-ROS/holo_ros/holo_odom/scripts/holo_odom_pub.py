#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

from holo_msgs.msg import HoloTransform


class HoloOdomPublisher(object):
    
    def __init__(self):
        # init ROS node
        rospy.init_node("holo_odom_pub", anonymous=True)

        self.tf_sub_ = rospy.Subscriber("holo_tf", HoloTransform, self.recv_tf_)
        self.odom_pub_ = rospy.Publisher('odom', Odometry, queue_size=10)

        self.base_link_ = rospy.get_param("base_link", default="base_footprint")

        self.tf_broadcaster_ = tf.TransformBroadcaster()

        self.last_time_ = rospy.Time.now()
        self.last_x_ = 0.0
        self.last_y_ = 0.0
        self.last_th_ = 0.0

    
    def launch(self):
        rospy.spin()

    
    def recv_tf_(self, data):
        position = data.position
        rotation = data.rotation

        cur_time = rospy.Time.now()
        
        x = -position.z
        y = position.x
        # the y axis in HoloLens is the z axis in ROS
        # rotation in HoloLens is in degree (0 - 360)
        # convert it to the radians
        th = (rotation.y - 180) / 180.0 * 3.1415926
        th = -th
        
        dx = x - self.last_x_
        dy = y - self.last_y_
        dth = th - self.last_th_
        dt = (cur_time - self.last_time_).to_sec()

        self.last_x_ = x
        self.last_y_ = y
        self.last_th_ = th
        self.last_time_ = cur_time

        rospy.loginfo("[INFO] theta: {}".format(th))

        self.publish_odom_(dx, dy, dth, dt)


    def publish_odom_(self, dx, dy, dth, dt):
        # send odom tf
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.last_th_)

        self.tf_broadcaster_.sendTransform(
            translation=(self.last_x_, self.last_y_, 0),
            rotation=odom_quat,
            time=self.last_time_,
            child=self.base_link_,
            parent="odom"
        )

        odom = Odometry()
        odom.header.stamp = self.last_time_
        odom.header.frame_id = "odom"

        odom.child_frame_id = self.base_link_

        odom.pose.pose = Pose(
            Point(self.last_x_, self.last_y_, 0),
            Quaternion(*odom_quat)
        )

        odom.twist.twist = Twist(
            Vector3(1.0 * dx / dt, 1.0 * dy / dt, 0), 
            Vector3(0, 0, 1.0 * dth / dt)
        )

        self.odom_pub_.publish(odom)


if __name__ == '__main__':
    try:
        node = HoloOdomPublisher()
        node.launch()
    except rospy.ROSInterruptException:
        pass