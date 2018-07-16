#!/usr/bin/env python
#
# Copyright 2018 <copyright holder> <email>
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# 
#
import rospy
from sensor_msgs.msg import LaserScan
import  math
from matplotlib.patches import Circle
import csv # csv reading needed library
from nlosExclusion.msg import GNSS_Raw,GNSS_Raw_Array # ros msg
from geometry_msgs.msg import Quaternion, Point, Pose, Twist,PoseStamped # ros message needed
from nav_msgs.msg import Odometry, Path 
from PyQt4.QtCore import *
import time
from PyQt4 import QtCore, QtGui
import tf,math
# import puCSV2Topi
#/*send control information
  #*author: WEN Weisong (17902061r@connect.polyu.hk)
  #*date:2018.07.05
  #* sendCarInfoKernel(float steeringAngle,unsigned char steeringTorque, float motionTorque, float motionAcc, unsigned char autuatorMode)
  #*detail:callback function
  #* msg->pose.pose.position.x  ->steeringAngle
  #* msg->pose.pose.position.y  ->steeringTorque
  #* msg->pose.pose.position.z  -> reserved
  #* msg->twist.twist.linear.x  -> motionTorque
  #* msg->twist.twist.linear.y  -> motionAcc
  #* msg->twist.twist.linear.z  -> autuatorMode
  #* 
  #*/
class eggVehicleControl(QtCore.QThread):
    def __init__(self, parent=None):
	super(eggVehicleControl, self).__init__(parent)
        rospy.Subscriber('/ndt_pose', PoseStamped, self.ndtPoseCallback)
	self.eggVehicleControl_Pub = rospy.Publisher('eggVehicleControl', Odometry,queue_size=100)  #
	self.control_ = Odometry()
	self.update = 0
	self.preTime = 0
	self.delt = 0
	
	# yaw control
	self.curYaw = 0
	self.objYaw = 10 # expected yaw
	self.yawKp = 5.0
	self.yawKd = 1.0
	
	# motion control
	self.prePose = PoseStamped()
	self.curPose = PoseStamped()
	self.curVel = 0.0
	self.objVel = 0.5
	self.velKp = 400
	self.velKd = 400
	
	# waypoint
	self.objectWayPoint = PoseStamped()
	self.waypointX = []
	self.waypointY = []
	self.waypointZ = []
	self.waypointYaw = []
	self.waypointIndex = 0
	self.fastReadCSV()

    def fastReadCSV(self): # read the path into list

        self.Fcsv_GNSS = csv.reader(open('pathPartial.csv',
                                        'r'))  # read csv context to csv_reader variable
        for rowCsv in self.Fcsv_GNSS:
            if(rowCsv[0] == 'self.poseX'):
                continue
	    self.waypointX.append(float(rowCsv[0]) )  # waypointX
	    self.waypointY.append(float(rowCsv[1]) )  # waypointY
	    self.waypointZ.append(float(rowCsv[2]) )  # waypointZ
	    self.waypointYaw .append(float(rowCsv[3])) # waypoint Yaw
	print 'finish read the path file, you can switch to automous driving mode now'

    def ndtPoseCallback(self, data):  # callback ndt pose
        self.curPose = data # 
        self.update = 1
        #print 'data.pose.position.x->',data.pose.position.x
        if(self.waypointIndex <2): # ndt_pose is updated
	  self.delt = rospy.get_time() - self.preTime
	  print 'self.delt->',self.delt
	  #print 'vehicle motion control '
	  self.velocityControl()
	  #print 'vehicle steering control ---'
	  self.objectYawUpdate()
	  self.yawControl()
	  self.preTime =rospy.get_time()
	  self.prePose = self.curPose
    
    def run(self):
      while(1):
	time.sleep(1)
	if(self.waypointIndex <2): # ndt_pose is updated
	  self.eggVehicleControl_Pub.publish(self.control_)
	  
	  
    def velocityControl(self): # motion control 
      deltS = pow((self.curPose.pose.position.x - self.prePose.pose.position.x),2)
      deltS = deltS + pow((self.curPose.pose.position.y - self.prePose.pose.position.y),2)
      deltS = math.sqrt(deltS)
      self.curVel = deltS / self.delt
      #print 'self.curVel->',self.curVel
      deltVel = self.objVel - self.curVel
      motionTorque = 11000
      motionTorque =math.fabs(10000 + self.velKp * deltVel + self.velKd * (deltVel/self.delt)) # 0~21500
      
      #motionAcc = math.fabs(1000 + self.velKp * deltVel + self.velKd * (deltVel/self.delt)) # 0~21500
      motionAcc =0
      if(deltVel>0):
	autuatorMode = 1 # accelerate 
      if(deltVel<0):
	autuatorMode = 1 # descend vel 
      self.control_.twist.twist.linear.x  = motionTorque
      self.control_.twist.twist.linear.y  = motionAcc
      self.control_.twist.twist.linear.z  = autuatorMode
      #print 'P->',self.velKp * deltVel,'     ', 'D->',self.velKd * (deltVel/self.delt),'   motionTorque->',motionTorque, 'velocity->' ,self.curVel
   
    def objectYawUpdate(self):
      if(self.waypointIndex == 0):
	deltx = self.waypointX[self.waypointIndex] - self.curPose.pose.position.x
	delty = self.waypointY[self.waypointIndex] - self.curPose.pose.position.y
	self.objYaw = 360 + math.atan2(delty,deltx) *180/3.14159
	self.waypointIndex =1
	print 'Go to first point'
      if(self.waypointIndex > 0):
	deltx = self.waypointX[self.waypointIndex] - self.curPose.pose.position.x
	delty = self.waypointY[self.waypointIndex] - self.curPose.pose.position.y
	delts = math.fabs(math.sqrt(deltx * deltx + delty * delty))
	
	deltxNext = self.waypointX[self.waypointIndex + 1] - self.curPose.pose.position.x
	deltyNext = self.waypointY[self.waypointIndex + 1] - self.curPose.pose.position.y
	deltsNext = math.fabs(math.sqrt(deltx * deltx + delty * delty))
	
	if(deltsNext<delts):
	  self.objYaw = 180 + math.atan2(delty,deltx) *180/3.14159
	  self.waypointIndex = self.waypointIndex + 1
	
	#if(delts > 1.0):
	  #self.waypointIndex =self.waypointIndex + 1
	  #deltx = self.waypointX[self.waypointIndex] - self.curPose.pose.position.x
	  #delty = self.waypointY[self.waypointIndex] - self.curPose.pose.position.y
	  #self.objYaw = 180 + math.atan2(delty,deltx) *180/3.14159
	  #print 'Go to next point'
      
    def yawControl(self): # steering control
      quaternion = (
      self.curPose.pose.orientation.x,
      self.curPose.pose.orientation.y,
      self.curPose.pose.orientation.z,
      self.curPose.pose.orientation.w)
      euler = tf.transformations.euler_from_quaternion(quaternion)
      self.curYaw = 180 + euler[2] *180/3.14159
      print 'current yaw->',self.curYaw, 'object yaw->',self.objYaw,'Waypoint',self.waypointX[self.waypointIndex],'  ',self.waypointY[self.waypointIndex]
      # -430~430
      #  20~200
      if(math.fabs(self.objYaw - self.curYaw) < 180):
	steeringAngle = 0 + self.yawKp * (self.objYaw - self.curYaw) + self.yawKd * ((self.objYaw - self.curYaw) / self.delt)
	steeringAngle = -steeringAngle
	print 'steeringAngle->', steeringAngle, 'yawKd->',self.yawKp * (self.objYaw - self.curYaw),'yawlast->',self.yawKd * ((self.objYaw - self.curYaw) / self.delt)
      if(math.fabs(self.objYaw - self.curYaw) > 180):
	if(self.objYaw < self.curYaw):
	  steeringAngle = 0 + 1*self.yawKp * (self.curYaw - self.objYaw -360) + 1 * self.yawKd * ((self.curYaw - self.objYaw -360) / self.delt)
	if(self.objYaw >self.curYaw):
	  steeringAngle = 0 + -1*self.yawKp * (self.objYaw - self.curYaw-360) + -1 * self.yawKd * ((self.objYaw - self.curYaw-360) / self.delt)
     
      steeringTorque =25
      self.control_.pose.pose.position.x = steeringAngle
      self.control_.pose.pose.position.y = steeringTorque


if __name__ == '__main__':
    rospy.init_node('eggVehicleControl', anonymous=True)
    eggVehicleControl_=eggVehicleControl()
    eggVehicleControl_.start()
    print 'process eggVehicleControl...'
    print '--------------------------'
    rate = rospy.Rate(30)  # 30hz
    while not rospy.is_shutdown():
	# rospy.spin()
	rate.sleep()