#!/bin/sh

LEFT_IMAGE_TOPIC="/right_camera_img"
RIGHT_IMAGE_TOPIC="/left_camera_img"

rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.03 \
    right:=$RIGHT_IMAGE_TOPIC left:=$LEFT_IMAGE_TOPIC