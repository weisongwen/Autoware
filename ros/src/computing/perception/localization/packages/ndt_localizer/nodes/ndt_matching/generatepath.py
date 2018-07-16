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
class generatePath(QtCore.QThread):
    def __init__(self, parent=None):
	super(generatePath, self).__init__(parent)
        rospy.Subscriber('/ndt_pose', PoseStamped, self.ndtPoseCallback)
        self.curPose = PoseStamped()
        self.prePose = PoseStamped()
        
        self.poseX = []
        self.poseY = []
        self.poseZ = []
        self.Yaw = []
        self.YawfromPose = []
	

    def ndtPoseCallback(self, data):  # callback ndt pose
        self.curPose = data # 
        delx = (self.curPose.pose.position.x - self.prePose.pose.position.x) * (self.curPose.pose.position.x - self.prePose.pose.position.x)
        dely = (self.curPose.pose.position.y - self.prePose.pose.position.y) * (self.curPose.pose.position.y - self.prePose.pose.position.y)
        delxy = math.sqrt(delx + dely)
        
        quaternion = (
	self.curPose.pose.orientation.x,
	self.curPose.pose.orientation.y,
	self.curPose.pose.orientation.z,
	self.curPose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	self.curYaw = euler[2] *180/3.14159
	
        if(delxy > 0.3):
	  self.poseX.append(self.curPose.pose.position.x)
	  self.poseY.append(self.curPose.pose.position.y)
	  self.poseZ.append(self.curPose.pose.position.z)
	  self.Yaw.append(self.curYaw + 180)
	  self.YawfromPose.append(180 + math.atan2(dely,delx) *180/3.14159)
	  self.prePose = self.curPose
        
        self.update = 1
        #self.delt = rospy.get_time() - self.preTime
	self.preTime =rospy.get_time()
        #print 'data.pose.position.x->',data.pose.position.x
        rows = zip(self.poseX,self.poseY,self.poseZ,self.Yaw,self.YawfromPose)
	with open('path.csvwithYaw', "w") as f: # output the integration positioning error
	    writer = csv.writer(f)
	    for row in rows:
		writer.writerow(row)
    
    def run(self):
      while(1):
	time.sleep(1)
	  


if __name__ == '__main__':
    rospy.init_node('generatePath', anonymous=True)
    generatePath_=generatePath()
    generatePath_.start()
    print 'process generatePath...'
    print '--------------------------'
    rate = rospy.Rate(30)  # 30hz
    while not rospy.is_shutdown():
	# rospy.spin()
	rate.sleep()