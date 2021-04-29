#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy

#from voice_text_interface.srv import text_to_speech
from voice_text_interface.srv import *

__author__ = 'shashank shekhar'

def voice_recognition_client():
    rospy.wait_for_service('speech_to_text')
    try:
        speech_to_text_req = rospy.ServiceProxy('speech_to_text', speech_to_text) 
        resp1 = speech_to_text_req("ss")  
        print("Responding to the voice command!")   
        return resp1.status
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":    
    resp = voice_recognition_client()
    print('The status is: ', resp)
    if(resp == "failure"):
        print("Did not hear successfully!!")
        sys.exit()

