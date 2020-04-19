#!/usr/bin/env python
import threading
import struct

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from holo_msgs.msg import HoloTransform
from cv_bridge import CvBridge, CvBridgeError

from tcp_server import TcpServer


class HoloSensorReceiver(object):

    def __init__(self):
        # init ROS node
        rospy.init_node("holo_sensor_receiver", anonymous=True)
        
        self.server_ = TcpServer(
            # get from private domain ~
            host=rospy.get_param("~host", default="192.168.1.152"),
            port=rospy.get_param("~port", default=8800)
        )

        self.pub_ = rospy.Publisher("image_topic", Image, queue_size=5)
        self.bridge_ = CvBridge()

        # self.pub_ = rospy.Publisher('holo_transform', HoloTransform, queue_size=10)

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
            width, height, img_bgr = self.unpack_(package)

            print("Receive img shape = {}, width = {}, height = {}".format(
                img_bgr.shape, width, height))
            
            img_bgr = cv2.resize(img_bgr, (320, 240))
            img_bgr = cv2.rotate(img_bgr, cv2.ROTATE_90_CLOCKWISE)

            try:
                self.pub_.publish(self.bridge_.cv2_to_imgmsg(img_bgr, "bgr8"))
            except CvBridgeError as e:
                print(e)
    

    def unpack_(self, package):
        """
        Unpack TCP package

        Args:
            package (bytes uint8):
                0 - 3:      width
                4 - 7:      height
                8 - EOF:    image (RGBA)
        """
        width = struct.unpack('<I', package[0 : 4])[0]
        height = struct.unpack('<I', package[4 : 8])[0]

        img_bin = package[8:]
        img_rgba = np.fromstring(img_bin, dtype=np.uint8).reshape((height, width, 4))
        img_bgr = cv2.cvtColor(img_rgba, cv2.COLOR_RGBA2BGR)

        return width, height, img_bgr
    
    
if __name__ == '__main__':
    try:
        node = HoloSensorReceiver()
        node.launch()
    except rospy.ROSInterruptException:
        pass