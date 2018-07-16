/*
 * Copyright (c) 2015-2020 idriverplus(Beijing ZhiXingZhe Inc.)
 * website: www.idriverplus.com
 * Distributed under the IVPT Software License
 * Author: zhangbaofeng
 * This node is used to read serial data and publish the data content, and subscription data content into a serial port.
 * * *************************************************************************
 * */

#include "actuator.h"

int main(int argc, char** argv)
{
   ros::init(argc,argv,"ivactuator");
   ros::NodeHandle nh;
   actuator* ivnode = new actuator(nh);
   ivnode->run();
   ros::spin();
 }
