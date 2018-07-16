/*
 * Copyright (c) 2015-2020 idriverplus(Beijing ZhiXingZhe Inc.)
 * website: www.idriverplus.com
 * Distributed under the IVPT Software License
 * Author: zhangbaofeng
 * This node is used to read serial data and publish the data content, and subscription data content into a serial port.
 * * *************************************************************************
 * */

#ifndef ACTUATOR_H
#define ACTUALOR_H

#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <string.h>
#include <boost/asio.hpp> 
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include "pthread.h"
#include "serial/serial.h"
#include "ivactuator/ivactuator.h"
#include "nav_msgs/Odometry.h"
/* FOREGROUND */
#define RST  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define FYEL(x) KYEL x RST
#define FBLU(x) KBLU x RST
#define FMAG(x) KMAG x RST
#define FCYN(x) KCYN x RST
#define FWHT(x) KWHT x RST
#define BOLD(x) "\x1B[1m" x RST
#define UNDL(x) "\x1B[4m" x RST


#define RECV_MSG_LEN  23

using namespace std;
using namespace boost::asio; 
using namespace boost;

class actuator{
  public:
    actuator(ros::NodeHandle handle);
    ~actuator(){}  
    void callbackControl(const nav_msgs::Odometry::ConstPtr& msg);
    void run();
    void recvCarInfoKernel();
    void sendCarInfoKernel(float steeringAngle,unsigned char steeringTorque, float motionTorque, float motionAcc, unsigned char autuatorMode);
    void callback_sendthread();
  
  public:
    //interface
    ros::Subscriber sub_control;
    
    float targetangle;
    unsigned char torque;
    
    float targettorque;
    float targetacc;
    unsigned char actuatormode;
  private:  	
    int  m_baudrate;
    int  m_deviceName;
    int  receiverCurrentByteCount;    //The number of have received byte
    unsigned char   tempDataArray[350];//Size of serial buffer
    std::string     m_serialport;
    serial::Serial  ser;
    ros::NodeHandle m_handle;
    ros::Publisher  pub_control;
    ivactuator::ivactuator ivactuatorMsg;
};

#endif /*ACTUALOR_H*/
