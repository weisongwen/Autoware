/*
 * Copyright (c) 2015-2020 idriverplus(Beijing ZhiXingZhe Inc.)
 * website: www.idriverplus.com
 * Distributed under the IVPT Software License
 * Author: zhangbaofeng
 * This node is used to read serial data and publish the data content, and subscription data content into a serial port.
 * * *************************************************************************
 * */

#include "actuator.h"

actuator::actuator(ros::NodeHandle handle)
{
    m_handle = handle;
    m_baudrate = 9600;
    m_serialport = "/dev/ttyUSB0";
    receiverCurrentByteCount = 0;
    memset(tempDataArray,0,sizeof(tempDataArray));
    // for steering control 
   targetangle = 0; //-430~430
  torque = 0; // 20~200
  
  // for motion control 
   targettorque = 0; // 7000~21500
   targetacc =0; // 0~21500
    actuatormode = 0; // 0:decrease vel 1: increase vel 2: jin ji zhi dong
    //set and open serial
    try{
      	std::cout<<FYEL("[ivactuator-->]")<<FGRN("Serial initialize start!")<<std::endl;
      	ser.setPort(m_serialport.c_str());// virtual com address  /dev/pts/3
      	ser.setBaudrate(m_baudrate);
      	serial::Timeout to = serial::Timeout::simpleTimeout(500);
      	ser.setTimeout(to);
      	ser.open();
    }catch(serial::IOException& e){
        std::cout<<FYEL("[ivactuator-->]")<<FRED("Unable to open port!")<<std::endl;
    }
    if(ser.isOpen()){
        std::cout<<FYEL("[ivactuator-->]")<<FGRN("Serial initialize successfully!")<<std::endl;
    }else{
        std::cout<<FYEL("[ivactuator-->]")<<FRED("Serial port failed!")<<std::endl;
    } 
}

void actuator::run()
{
    pub_control = m_handle.advertise<ivactuator::ivactuator>("ivactuator",1000);
    sub_control = m_handle.subscribe("eggVehicleControl",1000,&actuator::callbackControl,this);
    boost::thread thread(boost::bind(&actuator::callback_sendthread, this) );  
    thread.detach();
    int run_rate = 20; //50ms
    bool timeFlag = false;
    ros::Rate rate(run_rate);
    while(ros::ok()){
        ros::spinOnce();
        recvCarInfoKernel();   // read message from serial and pub message to contro
        rate.sleep();
    }
}

/*send control information
  *author: WEN Weisong (17902061r@connect.polyu.hk)
  *date:2018.07.05
  * sendCarInfoKernel(float steeringAngle,unsigned char steeringTorque, float motionTorque, float motionAcc, unsigned char autuatorMode)
  *detail:callback function
  * msg->pose.pose.position.x  ->steeringAngle
  * msg->pose.pose.position.y  ->steeringTorque
  * msg->pose.pose.position.z  -> reserved
  * msg->twist.twist.linear.x  -> motionTorque
  * msg->twist.twist.linear.y  -> motionAcc
  * msg->twist.twist.linear.z  -> autuatorMode
  * 
  */
 void actuator::callbackControl(const nav_msgs::Odometry::ConstPtr& msg){
   ROS_INFO("control information is received");
  // for steering control 
   targetangle = msg->pose.pose.position.x; //-430~430
  torque = msg->pose.pose.position.y; // 20~200
  
  // for motion control 
   targettorque = msg->twist.twist.linear.x; // 7000~21500
   targetacc =msg->twist.twist.linear.y; // 0~21500
    actuatormode = msg->twist.twist.linear.z; // 0:decrease vel 1: increase vel 2: jin ji zhi dong
//   sendCarInfoKernel(300,70,15000,0,1);//send message to serial 
//    sendCarInfoKernel(targetangle,torque,targettorque,targetacc,actuatormode);
   
 }

/**
 *callback_sendthread()
 *detail:therd send message to serial
 */
void actuator::callback_sendthread()
{
    while(1){
      if(targettorque >0) // control is available 
      {
	sendCarInfoKernel(targetangle,torque,targettorque,targetacc,actuatormode);
	   ROS_INFO("send the PID control information");

      }
      else // control is not available
      {
	 sendCarInfoKernel(0,25,10000,0,1);//send message to serial
	    ROS_INFO("send the defaut control information");
      }
        usleep(50*1000); //50ms
    }
}


/**
 *RecvCarInfoKernel()
 *detail:Read serial data, publish after parsing
 */
void actuator::recvCarInfoKernel()
{
    unsigned char str[1000];
    memset(&str,0,sizeof(str));
    std::string recvstr = ser.read(ser.available());
    int lenrecv= recvstr.size(); //here read serialport data
    if(lenrecv <= 0){
    	return;
    }else{
        if( (lenrecv+receiverCurrentByteCount) > sizeof(tempDataArray) )
            return;
    }
    // printf("ivactuator-->lenrecv = %d\n",lenrecv);
    for(int i=0; i<lenrecv; i++){
        tempDataArray[i+receiverCurrentByteCount] = recvstr[i];
    }
    receiverCurrentByteCount+=lenrecv;
    if(receiverCurrentByteCount < RECV_MSG_LEN){
        return;
    }
    int headStartPosition = 0;
    bool key = false;
    for(int i=0; i<receiverCurrentByteCount; i++){
        //printf("ivactuator-->recvstr[%d] = 0x%x\n",i,tempDataArray[i]);
        str[i] = tempDataArray[i];
        if((tempDataArray[i]==0XFF)&&(tempDataArray[i+1] == 0XA5)&&(tempDataArray[i+2]==0X5A) && !key){
            headStartPosition= i;
            if((headStartPosition+RECV_MSG_LEN)>(receiverCurrentByteCount))
                return;
            else
                key = true;
        }
    } 
    int count=0;
    for(int i = headStartPosition;i<receiverCurrentByteCount;i+=RECV_MSG_LEN){ 
        if(i+RECV_MSG_LEN<=receiverCurrentByteCount){
            count++;
            unsigned char sum = 0;
            for(int j=3; j<RECV_MSG_LEN-1;j++){
                sum += str[i + j];
            }
            if(sum ==  str[i+RECV_MSG_LEN-1]) {//check sum
                memset(&ivactuatorMsg,0,sizeof(ivactuatorMsg));
                ivactuatorMsg.uisteerangle      = (short)( (str[i+10]<<8)|str[i+11] );
                ivactuatorMsg.uispeed           = str[i+9]; 
               // printf("sucessful pub\n");
                pub_control.publish(ivactuatorMsg);
            }
        }
    }
    headStartPosition = headStartPosition+RECV_MSG_LEN*count;
    int lenthTemp = receiverCurrentByteCount;
    receiverCurrentByteCount = 0;
    memset(tempDataArray,0,sizeof(tempDataArray));
    if( headStartPosition < lenthTemp){
        int k=0;
        for(int j=headStartPosition; j<lenthTemp; j++,k++){
            tempDataArray[k]=str[j];
        }
        receiverCurrentByteCount = k;
    }
}

/**
 *SendCarInfoKernel()
 *detail:Write data to serial port
 */
void actuator::sendCarInfoKernel(float steeringAngle,unsigned char steeringTorque, float motionTorque, float motionAcc, unsigned char autuatorMode)
{
    // for steering control 
    float targetangle = steeringAngle; //-430~430
    unsigned char torque = steeringTorque; // 20~200
    
    // for motion control 
    float targettorque = motionTorque; // 7000~21500
    float targetacc =motionAcc; // 0~21500
    unsigned char actuatormode = autuatorMode; // 0:decrease vel 1: increase vel 2: jin ji zhi dong

    unsigned char sendBuffer[17] = {0xFF,0XA5,0X5A,0x0C,0x81};
    memset(&sendBuffer[5],0,sizeof(sendBuffer)-5);
    unsigned short tmp = targetangle;
    sendBuffer[5] = tmp / 256;
    sendBuffer[6] = tmp % 256;
    sendBuffer[7] = torque; 
    tmp = targettorque;
    sendBuffer[8] = tmp / 256;
    sendBuffer[9] = tmp  % 256;

    tmp = targetacc;
    sendBuffer[10] = tmp / 256;
    sendBuffer[11] = tmp % 256;

    // printf("dhdhdhdhh= \n");

    sendBuffer[12] = actuatormode; 
    sendBuffer[13] = 0x02;
    sendBuffer[14] = 1;
    sendBuffer[15] = 0;
    unsigned char sum = 0;
    for(int i = 3; i < 16; ++i)//clac sum
        sum += sendBuffer[i];
    sendBuffer[16] = (unsigned char)(sum);
    ser.write(sendBuffer,sizeof(sendBuffer));
}
