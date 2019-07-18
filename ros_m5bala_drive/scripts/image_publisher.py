#!/usr/bin/env python
import os
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

def operator():
    rospy.init_node('image_file_publisher', anonymous=True)
    pub_c = rospy.Publisher('image_data/compressed', CompressedImage, queue_size=10)
    pub_r = rospy.Publisher('image_data/raw', Image, queue_size=10)
    dir_name = rospy.get_param('~dir_name')
    file_name = rospy.get_param('~file_name')
    sequence_no = int(rospy.get_param('~sequence_no'))
    if sequence_no < 1 and file_name != "":
        filepath = os.path.abspath(os.path.join(os.path.dirname(dir_name), file_name))
    elif sequence_no >= 1 and sequence_no <= 999 and file_name == "":
        qr_file_name = "MarkerData_%03d.png" % sequence_no
        filepath = os.path.abspath(os.path.join(os.path.dirname(dir_name), qr_file_name))
    else:
        raise SyntaxError("sequence_no is 1 to 999 and file_name is not empty.")
    print filepath
    output_width_px = int(rospy.get_param("~output_width_px", "320"))
    output_height_px = int(rospy.get_param("~output_height_px", "240"))
    pub_rate = int(rospy.get_param("~pub_rate", "1"))
    # make bridge
    bridge = CvBridge()
    rate = rospy.Rate(pub_rate)
    while not rospy.is_shutdown():
        # read image
        try:
            im = cv2.imread(filepath, cv2.IMREAD_COLOR)
            tmp = im[:, :]
            input_height_px, input_width_px = im.shape[:2]
            new_img = cv2.resize(np.full((1, 1, 3), 255, np.uint8), (output_width_px, output_height_px))
            # Add Margin(Side || Side)
            tmp = cv2.resize(tmp, dsize=None, fx=(float(output_height_px)/float(input_height_px)), fy=(float(output_height_px)/float(input_height_px)))
            start = int((output_width_px - tmp.shape[1]) / 2)
            fin = int((tmp.shape[1] + output_width_px) / 2)
            new_img[0:output_height_px, start:fin] = tmp
            # make msg
            msg_c = bridge.cv2_to_compressed_imgmsg(new_img, dst_format='jpg')
            msg_r = bridge.cv2_to_imgmsg(new_img, encoding="bgr8")
            pub_c.publish(msg_c)
            pub_r.publish(msg_r)
        except:
            pass
        rate.sleep()

if __name__ == '__main__':
    try:
        operator()
    except rospy.ROSInterruptException:
        pass
