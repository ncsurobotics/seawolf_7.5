#!/usr/bin/env python
import rospy

from vision.msg import VisObj
from vision.srv import VisionRequest

#TODO make this communicate with entity python files
#Need a way to pass image frames to the entity files
#Create service server to determine what object to look for
#Publish identitied object to a  topic
