#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <map>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/JointState.h>
#include <time.h>
#include <unistd.h>
#include <rns_msgs/JointCmd.h>
#include <rns_msgs/JointConfig.h>
#include <std_msgs/Int32MultiArray.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#define DXL_COUNT                       14
#define DXL_FIRST_ID			1        
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyACM0"

#define PROTOCOL_VERSION                1.0 
#define PERIOD_PROTOCOL 				20
#define PERIOD_MOVEMENT 				20

DynamixelWorkbench dxl_wb;
//all id
uint8_t dxl_id[DXL_COUNT] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14};
//id by gorups
uint8_t dxl_id_group_one[6] = {1,2,3,4,5,6}; // right hand
uint8_t dxl_id_group_two[6] = {7,8,9,10,11,12}; //left hand
uint8_t dxl_id_group_three[2] = {13,14}; //head
int32_t goal_position_group_one[6] = {1024,1042,2048,512,400,600}; //zero poses
int32_t goal_position_group_two[6] = {3072,1024,2048,512,600,400};
int32_t goal_position_group_three[2] = {2048,2048};
int32_t velocity = 60;
std::vector<double> present_position(DXL_COUNT);
std::vector<double> present_speed(DXL_COUNT);
const uint8_t handler_index_one = 0;
double ms, ms0;
bool disable_flag = false;
//initialization of motors
bool dxl_init(char* port){
  	bool result = false;
  	uint16_t model_number = 0;
  	const char *log;
  	result = dxl_wb.init(DEVICENAME, BAUDRATE, &log);
  	if (result == false)
  	{
		ROS_ERROR_STREAM(log);
    	ROS_ERROR_STREAM("Failed to init");
    	return false;
  	}
  	else{
    		ROS_INFO_STREAM("Succeed to init: "<<BAUDRATE); 
  	}
  	for (int cnt = DXL_FIRST_ID-1; cnt < DXL_COUNT; cnt++)
  	{
    	result = dxl_wb.ping(dxl_id[cnt], &model_number, &log);
    	if (result == false)
    	{
    		ROS_ERROR_STREAM(log);
      		ROS_ERROR_STREAM("Failed to ping id: "<<(int)dxl_id[cnt]);
      		return(false);
    	}
    	else
    	{
    		ROS_INFO_STREAM("Succeeded to ping: "<<(int)dxl_id[cnt]);
    	}
	if(dxl_id[cnt] == 1 || dxl_id[cnt] == 7)
		result = dxl_wb.jointMode(dxl_id[cnt],50, 400, &log);
	else
    		result = dxl_wb.jointMode(dxl_id[cnt], velocity, 400, &log); //first value - speed, second value - acceleration
    	if (result == false)
    	{
    		ROS_ERROR_STREAM(log);
      		ROS_ERROR_STREAM("Failed to change joint mode");
      		return(false);
    	}
    	else
    	{
      		ROS_INFO_STREAM("Succeed to change joint mode");
    	}
  	}
	uint8_t d_gain = 128;
	uint8_t i_gain = 16;
	uint8_t p_gain = 16;
  	dxl_wb.addSyncWriteHandler(dxl_id[DXL_FIRST_ID-1], "Goal_Position");
	if (DXL_FIRST_ID == 1){
  		for(int iter = 0; iter < 3; iter++){
			dxl_wb.writeRegister(dxl_id[iter], uint16_t(26), uint16_t(1), &d_gain, &log);
			dxl_wb.writeRegister(dxl_id[iter], uint16_t(27), uint16_t(1), &i_gain, &log);
			dxl_wb.writeRegister(dxl_id[iter], uint16_t(28), uint16_t(1), &p_gain, &log);
		}
	}
	for(int iter = 6; iter < 8; iter++){
        dxl_wb.writeRegister(dxl_id[iter], uint16_t(26), uint16_t(1), &d_gain, &log);
        dxl_wb.writeRegister(dxl_id[iter], uint16_t(27), uint16_t(1), &i_gain, &log);
        dxl_wb.writeRegister(dxl_id[iter], uint16_t(28), uint16_t(1), &p_gain, &log);
        }

	return(true);
}

//send positions
bool sync_write_pos(){
	bool result = false;
	// const char *log;
	//send by groups, because you could use syncWrite only on 6 motors
	if (DXL_FIRST_ID == 1){
		result = dxl_wb.syncWrite((uint8_t)0, (uint8_t*)dxl_id_group_one, (uint8_t)6, 
	  							(int32_t*)goal_position_group_one, 1, NULL);
	}
	result = dxl_wb.syncWrite((uint8_t)0, (uint8_t*)dxl_id_group_two, (uint8_t)6, 
							  (int32_t*)goal_position_group_two, 1, NULL);
	result = dxl_wb.syncWrite((uint8_t)0, (uint8_t*)dxl_id_group_three, (uint8_t)2, 
							  (int32_t*)goal_position_group_three, 1, NULL);
	if(result == false){
		ROS_INFO_STREAM("Failed to send goal position");
	}
	return(result);
}
//enabling torque for motor with current id
bool enableTorque(uint8_t id){
	const char*log;
	bool result = false;
	result = dxl_wb.torqueOn(id, &log);
	if(result == false){
		ROS_ERROR_STREAM("Failed to enable Torque on id: "<<(int)id);
		ROS_ERROR_STREAM(log);
	}
	else 
		ROS_INFO_STREAM("Succeded to enable torque on id: "<<(int)id);	
}	

//disabling torque for motor with current id
bool disableTorque(uint8_t id){
	const char*log;
	bool result = false;
	result = dxl_wb.torqueOff(id, &log);
	if(result == false){
		ROS_ERROR_STREAM("Failed to disable Torque on id: "<<(int)id);
		ROS_ERROR_STREAM(log);
	}
	else 
		ROS_INFO_STREAM("Succeded to disable torque on id: "<<(int)id);
	return(result);
}

//read position of motor with current id
bool read_pos(uint8_t id){
  	const char*log;
  	bool result = false;
  	int32_t get_data = 0;
  	result = dxl_wb.itemRead(id, "Present_Position", &get_data, &log);
  	present_position[id-1] = get_data;
  	if (result == false)
  	{
   		ROS_ERROR_STREAM(log);
    	ROS_ERROR_STREAM("Failed to get present position on id = " << (int)id);
  		return(false);
  	}
  	return(true);
}
bool read_vel(uint8_t id){
	const char*log;
	bool result = false;
	int32_t get_data = 0;
	result = dxl_wb.itemRead(id,"Present_Speed", &get_data, &log);
	present_speed[id-1] = get_data;
	if(result == false)
	{
		ROS_ERROR_STREAM(log);
		return(false);
	}
	return(true);
}
//get time in milliseconds
double millis()
{
    struct timeval te;
    gettimeofday(&te, NULL);
    return (te.tv_sec + (te.tv_usec / 1000000.0)) * 1000.0;
}

void messageJointscmd(const rns_msgs::JointCmd::ConstPtr& toggle_msg)  //controll position
{
    
    int channel = toggle_msg->channel;
    
    if (channel<0||channel>13)
        return;
    
    if(channel < 6)
        goal_position_group_one[channel] = (int)toggle_msg->cmd;
    else if(channel<12)
        goal_position_group_two[channel-6] = (int)toggle_msg->cmd;
    else
        goal_position_group_three[channel-12] = (int)toggle_msg->cmd;
}

int main(int argc, char **argv)
{	
	std::cout << "test node" << std::endl;
	bool result = false;
	bool err_flag = false;
	int iter = 0;
//	result = sync_write_pos();
	ms0 = millis();
	ros::init(argc,argv, "Dxl_Arm");
	ros::NodeHandle nh;
	bool param;
	sensor_msgs::JointState joints_msg;
	ros::Subscriber jointcmd = nh.subscribe<rns_msgs::JointCmd> ("cmd_joints",10, messageJointscmd );
	ros::Publisher Joint_State = nh.advertise<sensor_msgs::JointState>("arm_joint_states", 64);

	ros::Rate loop_rate(PERIOD_PROTOCOL);
	nh.getParam("/arms/disable_torque",param);
	std::string port;
	nh.getParam("/arms/dxl_port", port);	
	ROS_INFO_STREAM("take param:"<<param);
       	if(param == true)
               disable_flag = true;
     	else
               disable_flag = false;
	int error_count = 0;
	char dxl_port[port.length()];
	for(int i = 0; i < sizeof(dxl_port); i++){
		dxl_port[i] = port[i];		
	} 
	while(result == false)
		result = dxl_init(dxl_port);
		sleep(0.01);
	while(ros::ok()){
	  //      ROS_INFO_STREAM("take_param "<<disable_param);
		for(int i = DXL_FIRST_ID-1; i < DXL_COUNT; i++){
			result = read_pos(dxl_id[i]);
			result = read_vel(dxl_id[i]);
		}
		ms = millis();
		double delta = ms-ms0;
		if(delta >= PERIOD_MOVEMENT && err_flag == false && disable_flag == false){
			err_flag = !sync_write_pos();
            error_count += err_flag;
		}
        	if (error_count>32)
           		 return 0;
       		 if(disable_flag){
        	for(int i = DXL_FIRST_ID-1; i < DXL_COUNT; i++)
			disableTorque(dxl_id[i]);
 	       }
		ms0 = millis();
		joints_msg.velocity = present_speed;
		joints_msg.position = present_position;
		joints_msg.header.stamp = ros::Time::now();
		Joint_State.publish(joints_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	//if interrupt -> disable torque on motors
	for(int i = DXL_FIRST_ID; i < DXL_COUNT; i++)
		disableTorque(dxl_id[i]);
	return result;
}
