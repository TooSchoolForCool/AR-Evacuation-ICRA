#!/usr/bin/env python
import threading
import struct

import rospy
from holo_msgs.msg import HoloTransform

from tcp_server import TcpServer


class HoloTransformReceiver(object):

    def __init__(self):
        # init ROS node
        rospy.init_node("holo_tf_receiver", anonymous=True)
        
        self.server_ = TcpServer(
            host="192.168.1.152",
            port=12345
        )

        self.pub_ = rospy.Publisher('holo_transform', HoloTransform, queue_size=10)

        # call self.server_.stop before node destroied
        rospy.on_shutdown(self.server_.stop)


    def launch(self):
        self.server_.launch()

        # publish data received from the HoloLens
        # We use a thread to launch the publish() function since we 
        # want to have the ROS handling the system interupts.
        # In case, publish() has been blocked.
        thread = threading.Thread(target=self.publish_)
        thread.start()

        rospy.spin()

    
    def publish_(self):
        for package in self.server_.fetch_package():
            pos, rot = self.unpack_(package)

            msg = HoloTransform()

            msg.position.x = pos[0]
            msg.position.y = pos[1]
            msg.position.z = pos[2]

            msg.rotation.x = rot[0]
            msg.rotation.y = rot[1]
            msg.rotation.z = rot[2]

            self.pub_.publish(msg)
    

    def unpack_(self, package):
        """
        Unpack TCP package

        Args:
            package (bytes string):
                HoloLens position: from 0 ~ 11 bytes
                    0 - 3: camera position.x
                    4 - 7: camera position.y
                    8 - 11: camera position.z
                HoloLens orientation (euler angle): from 12 - 27 bytes
                    12 - 15: euler.x
                    16 - 19: euler.y
                    20 - 23: euler.z
        """
        position = [struct.unpack('<f', package[i : i + 4])[0] 
            for i in range(0, 12, 4)]

        euler = [struct.unpack('<f', package[i : i + 4])[0] 
            for i in range(12, 24, 4)]

        print("Position: {}".format(position))

        return position, euler
    
    
if __name__ == '__main__':
    try:
        node = HoloTransformReceiver()
        node.launch()
    except rospy.ROSInterruptException:
        pass