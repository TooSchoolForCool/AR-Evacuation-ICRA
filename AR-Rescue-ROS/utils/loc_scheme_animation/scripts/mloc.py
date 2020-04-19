#!/usr/bin/env python
import math

import numpy as np

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


class MLoc(object):
    
    def __init__(self):
        # init ROS node
        rospy.init_node("m_loc", anonymous=True)

        self.master_base_link_ = rospy.get_param("~master_base", default="agent_1_base_footprint")
        self.base_link_ = rospy.get_param("~base_link", default="agent_2_base_footprint")

        self.mu_ = rospy.get_param("~gaussian_mu", default=0.1)
        self.sigma_ = rospy.get_param("~gaussian_sigma", default=0.1)
        
        self.tf_broadcaster_ = tf.TransformBroadcaster()

        # record previous location
        self.x_ = None
        self.y_ = None
        self.theta_ = None

        self.drift_x_ = 0.0
        self.drift_y_ = 0.0
        self.drift_theta_ = 0.0

        print("master base: {}".format(self.master_base_link_))
        print("base link: {}".format(self.base_link_))
        print("gaussian mu: {}".format(self.mu_))
        print("gaussian sigma: {}".format(self.sigma_))
         

    def launch(self):
        listener = tf.TransformListener()
        rate = rospy.Rate(5)

        cnt = 0

        while not rospy.is_shutdown():
            try:
                trans, rot = listener.lookupTransform('/map', self.master_base_link_, rospy.Time(0))
            except:
                continue

            if cnt == 70:
                self.drift_theta_ = 0
                self.drift_x_ = 0.01
                self.drift_y_ = 0.01

                self.set_drift()
                cnt = 0
            
            if cnt > 5:
                self.update_(trans, rot)
            
            cnt += 1
            
            rate.sleep()
    
    
    def update_(self, trans, rot):
        # nx, ny, _ = trans
        # _, _, nthe = tf.transformations.euler_from_quaternion(rot)

        self.drift_x_ += self.gaussian_(0.0008, 0.02)
        self.drift_y_ += self.gaussian_(0.008, 0.02)
        # self.drift_theta_ += self.gaussian_(0.001, 0.2)

        self.set_drift()

    
    def set_drift(self):
        theta_quat = tf.transformations.quaternion_from_euler(
            0, 0, self.drift_theta_
        )

        self.tf_broadcaster_.sendTransform(
            translation=(self.drift_x_, self.drift_y_, 0),
            rotation=theta_quat,
            time=rospy.Time.now(),
            child=self.base_link_,
            parent=self.master_base_link_
        )

    
    def gaussian_(self, mu, sigma):
        return np.random.normal(mu, sigma)


if __name__ == "__main__":
    node = MLoc()
    node.launch()