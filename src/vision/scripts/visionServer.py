#!/usr/bin/env python

#ROS Imports
import rospy

from vision.msg import VisObj
from vision.srv import VisionRequest

#seawolf imports
import seawolf as sw  #Seems to be needed for accessing sensor data idk
import svr            #Needed for accessing camera streams?


def hubConnect():
  """ Connecting to seawolf hub """
  sw.loadConfig("../conf/seawolf.conf")
  sw.init("visionServer : Main")  #Taken from run.py in mission control which said missionControl : Main

def svrConnect():
  """ Connecting to svr """
  svr.connect()
  """ Setting up svr streams """
  forward = svr.Stream("forward") #Forward? are these names of cameras?
  forward.unpause()
  down = svr.Stream("down")       #Down?    where are the names defined?
  down.unpause()

  global cameras
  cameras = {
             "forward" : forward,
             "down"    : down
            }

#def runVision():
#  
#  while rospy.is_shutdown():
    

#TODO make this communicate with entity python files
#Need a way to pass image frames to the entity files
#Create service server to determine what object to look for
#Publish identitied object to a  topic
