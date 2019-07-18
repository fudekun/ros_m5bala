#!/usr/bin/env python
# -*- coding:utf-8 -*-

import pyaudio
import rospy
import numpy as np

from std_msgs.msg import Int16MultiArray

RATE=4096
p=pyaudio.PyAudio()
stream=p.open(format = pyaudio.paInt16,
		channels = 1,
		rate = RATE,
		input = False,
		output = True)

def callback(data):
    input = np.array(data.data)
    input = input.astype(np.int16).tostring()
    output = stream.write(input)
    
def listener():
    rospy.init_node('sound_listener', anonymous=True)
    rospy.Subscriber("mic", Int16MultiArray, callback)
    rospy.spin()
        
if __name__ == '__main__':
    listener()
    stream.stop_stream()
    stream.close()
    p.terminate()
