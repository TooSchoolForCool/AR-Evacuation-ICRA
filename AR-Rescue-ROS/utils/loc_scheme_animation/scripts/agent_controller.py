#!/usr/bin/env python
import math

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


class AgentController(object):

    def __init__(self):
        # init ROS node
        rospy.init_node("agent_controller", anonymous=True)
        
        # agent config
        self.base_link_ = rospy.get_param("~base_link", default="agent_1_base_footprint")
        self.speed_ = rospy.get_param("~speed", default=0.01)
        self.herz_ = rospy.get_param("~herz", default=20)
        self.dth_ = rospy.get_param("~turn_speed", default=0.0345)
        
        self.orientation_offset_ = 1.5707
        self.orientation_ = None

        # ROS component
        self.tf_broadcaster_ = tf.TransformBroadcaster()
        self.rate_ = rospy.Rate(self.herz_)

        # init tf between robot and map
        self.move_(0, 0, 0)

        print("baselink: {}".format(self.base_link_))
        print("speed: {}".format(self.speed_))
        print("herz: {}".format(self.herz_))


    def launch(self, trajectory):
        assert(len(trajectory) > 1)
        segments = [(trajectory[i], trajectory[i+1]) for i in range(0, len(trajectory) - 1)]

        for src, dst in segments:
            self.turn_to_(src, dst)
            self.goto_(src, dst)
            
    
    def turn_to_(self, src, dst):
        theta = math.atan2(dst[1] - src[1], dst[0] - src[0])

        if self.orientation_ is not None:
            n_steps = int(abs((theta - self.orientation_)) / self.dth_)
            dth = (theta - self.orientation_) / n_steps

            for _ in range(n_steps):
                self.orientation_ += dth
                self.move_(src[0], src[1], self.orientation_)

        self.move_(src[0], src[1], theta)
        self.orientation_ = theta
    

    def goto_(self, src, dst):
        theta = math.atan2(dst[1] - src[1], dst[0] - src[0])
        dx = self.speed_ * math.cos(theta)
        dy = self.speed_ * math.sin(theta)

        dist = math.sqrt((dst[1] - src[1]) ** 2 + (dst[0] - src[0]) ** 2)
        steps = int(dist / self.speed_)

        x, y = src
        for _ in range(0, steps):
            x += dx
            y += dy

            self.move_(x, y, theta)
            
        self.move_(dst[0], dst[1], theta)

    
    def move_(self, x, y, theta):
        """
        Args:
            x (float):      position.x
            y (float):      position.y
            theta (float):  orientation
        """
        theta_quat = tf.transformations.quaternion_from_euler(
            0, 0, theta + self.orientation_offset_
        )

        self.tf_broadcaster_.sendTransform(
            translation=(x, y, 0.015),
            rotation=theta_quat,
            time=rospy.Time.now(),
            child=self.base_link_,
            parent="map"
        )

        self.rate_.sleep()


def main():
    trajectory = [
        (-8, 0.35),
        (-4, 0.35),
        (-4, 4.5),
        (-2.1, 9),
        (-2.1, 12.3)
    ]

    node = AgentController()

    # raw_input("Press <Enter> to start")
    node.launch(trajectory)


if __name__ == '__main__':
    main()