#!/usr/bin/env python
import threading
import struct
import collections

import rospy
from nav_msgs.msg import Path

from holo_msgs.msg import HoloTransform

from tcp_client import TcpClient


class HoloPublisher(object):

    def __init__(self):
        # init ROS node
        rospy.init_node("holo_publisher", anonymous=True)

        self.path_sub_ = rospy.Subscriber("path_plan", Path, self.recv_path_)

        # streaming nearest n_points in planned path
        self.n_points_ = rospy.get_param("n_points", default=100)
        self.step_size = rospy.get_param("step_size", default=5)

        self.sender_ = TcpClient("192.168.1.176", 12346)

        self.q_ = collections.deque(maxlen=3)
        self.queue_cond_ = threading.Condition()
        
        
    def launch(self):
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            self.queue_cond_.acquire()

            if len(self.q_) == 0:
                self.queue_cond_.wait()
            self.sender_.send( self.q_.popleft() )

            self.queue_cond_.release()

            print("Send Path plan")
            
            rate.sleep()



    def recv_path_(self, data):
        path = data.poses[::self.step_size]

        n_points = min(self.n_points_, len(path))

        path_bin = b""
        
        for pose_stamped in path[:n_points]:
            position = pose_stamped.pose.position
            
            x_bin = struct.pack(">f", position.y)
            y_bin = struct.pack(">f", -1.5)
            z_bin = struct.pack(">f", -position.x)

            path_bin += b"".join([x_bin, y_bin, z_bin])
        
        packsize_bin = struct.pack(">I", len(path_bin))
        pack_bin = packsize_bin + path_bin


        self.queue_cond_.acquire()

        self.q_.append(pack_bin)
        
        self.queue_cond_.notify()
        self.queue_cond_.release()

        
if __name__ == '__main__':
    try:
        node = HoloPublisher()
        node.launch()
    except rospy.ROSInterruptException:
        pass