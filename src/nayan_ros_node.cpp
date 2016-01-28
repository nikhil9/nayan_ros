
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial/serial.h"

#include "geometry_msgs/Vector3.h"
#include "nayan_msgs/nayan_dbg.h"
#include "nayan_msgs/nayan_gps_pose_vel.h"
#include "nayan_msgs/nayan_attitude.h"
#include "nayan_msgs/nayan_rc_in.h"
#include "nayan_msgs/nayan_imu.h"
#include "nayan_msgs/nayan_vision_estimate.h"
#include "nayan_msgs/nayan_pose_vel_setpoint.h"
#include "nayan_msgs/param_set.h"

using std::string;
using std::cout;
using std::endl;

nayan_msgs::nayan_dbg mavlink_dbg_var;

nayan_msgs::nayan_gps_pose_vel gps_pose_vel;

nayan_msgs::nayan_attitude attitude;

nayan_msgs::nayan_rc_in rc_in;

nayan_msgs::nayan_imu imu;

nayan_msgs::nayan_pose_vel_setpoint set_pt;

nayan_msgs::nayan_vision_estimate vision_est;


ros::Publisher imu_pub;

ros::Publisher debug_var_pub;

ros::Publisher gps_pose_vel_pub;

ros::Publisher attitude_pub;

ros::Publisher rc_in_pub;


ros::Subscriber vision_est_sub;

ros::Subscriber pose_vel_setpt_sub;


ros::ServiceServer param_srv;


unsigned long baud = 115200;
serial::Serial ser("/dev/ttyACM0", baud, serial::Timeout::simpleTimeout(1000));


//=================ADDITION FROM MAVLINK====================//

#include "mavlink/v1.0/mavlink_types.h"

#ifndef MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#endif

static mavlink_system_t mavlink_system;

static void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    if (chan == MAVLINK_COMM_0)
    {
    	ser.write(&ch, 1);
    }
}

#include "mavlink/v1.0/common/mavlink.h"

//==========================================================//

void handleMessage(mavlink_message_t* msg, mavlink_channel_t chan)
{
    switch (msg->msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT:
    {
    	ROS_INFO("Got HeartBeat!");
        break;
    }

    case MAVLINK_MSG_ID_HIGHRES_IMU:
    {
//    	ROS_INFO("Got Highres IMU!");

    	mavlink_highres_imu_t highres_imu;

    	mavlink_msg_highres_imu_decode(msg, &highres_imu);

    	imu.gyro.x = highres_imu.xgyro;
    	imu.gyro.y = highres_imu.ygyro;
    	imu.gyro.z = highres_imu.zgyro;

    	imu.accel.x = highres_imu.xacc;
    	imu.accel.y = highres_imu.yacc;
    	imu.accel.z = highres_imu.zacc;

    	imu.header.stamp = ros::Time::now();
       	imu_pub.publish(imu);
        break;
    }

    case MAVLINK_MSG_ID_DEBUG:
    {
//    	ROS_INFO("Got debug variable!");

    	mavlink_debug_t debug_t;

    	mavlink_msg_debug_decode(msg, &debug_t);

    	mavlink_dbg_var.value = debug_t.value;

    	mavlink_dbg_var.id = debug_t.ind;

    	mavlink_dbg_var.header.stamp = ros::Time::now();

    	debug_var_pub.publish(mavlink_dbg_var);

    	break;
    }

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    {
//    	ROS_INFO("Got gps baro pose vel !");

    	mavlink_global_position_int_t mavlink_gps_pose;

    	mavlink_msg_global_position_int_decode(msg, &mavlink_gps_pose);

    	gps_pose_vel.lat = mavlink_gps_pose.lat;
    	gps_pose_vel.lon = mavlink_gps_pose.lon;
    	gps_pose_vel.relative_alt = mavlink_gps_pose.alt;

    	gps_pose_vel.vx = mavlink_gps_pose.vx;
    	gps_pose_vel.vy = mavlink_gps_pose.vy;
    	gps_pose_vel.vz = mavlink_gps_pose.vz;

    	gps_pose_vel.header.stamp = ros::Time::now();

    	gps_pose_vel_pub.publish(gps_pose_vel);

    	break;
    }

    case MAVLINK_MSG_ID_ATTITUDE:
    {
//    	ROS_INFO("Got Attitude !");
    	mavlink_attitude_t att;
    	mavlink_msg_attitude_decode(msg, &att);

    	attitude.roll = att.roll;
    	attitude.pitch = att.pitch;
    	attitude.yaw = att.yaw;

    	attitude.header.stamp = ros::Time::now();

    	attitude_pub.publish(attitude);

    	break;
    }

    case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
    {
//    	ROS_INFO("Got RC IN!");
    	mavlink_rc_channels_raw_t rc_in_t;
    	mavlink_msg_rc_channels_raw_decode(msg, &rc_in_t);

    	rc_in.rc_in[0] = rc_in_t.chan1_raw;
    	rc_in.rc_in[1] = rc_in_t.chan2_raw;
    	rc_in.rc_in[2] = rc_in_t.chan3_raw;
    	rc_in.rc_in[3] = rc_in_t.chan4_raw;
    	rc_in.rc_in[4] = rc_in_t.chan5_raw;
    	rc_in.rc_in[5] = rc_in_t.chan6_raw;
    	rc_in.rc_in[6] = rc_in_t.chan7_raw;

    	rc_in.header.stamp = ros::Time::now();

    	rc_in_pub.publish(rc_in);

    	break;
    }

    default:
    	break;
    }
}

void update_mavlink(void){

    mavlink_message_t msg;
    mavlink_status_t status;

	if(ser.available()){

		uint16_t num_bytes = ser.available();

		uint8_t buffer[num_bytes];

		ser.read(buffer, num_bytes);

		for(uint16_t i = 0; i < num_bytes; i++){

	        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {

	            handleMessage(&msg, MAVLINK_COMM_0);

	        }
		}

	}

}


void update_vision_estimate(const nayan_msgs::nayan_vision_estimate::ConstPtr& vis_est){

	ROS_INFO("update_vision_estimate");

	uint64_t _time_now = (uint64_t)(ros::Time::now().nsec) * (0.001);

	mavlink_msg_vision_position_estimate_send(
			MAVLINK_COMM_0,
			_time_now,
			vis_est->pose.x,
			vis_est->pose.y,
			vis_est->pose.z,
			vis_est->attitude.x,
			vis_est->attitude.y,
			vis_est->attitude.z);

	mavlink_msg_vision_speed_estimate_send(
			MAVLINK_COMM_0,
			_time_now,
			vis_est->vel.x,
			vis_est->vel.y,
			vis_est->vel.z);


}

void update_setpoint_pose_vel(const nayan_msgs::nayan_pose_vel_setpoint::ConstPtr& pv_setpt){

	ROS_INFO("update_setpoint_pose_vel");

	uint64_t _time_now = (uint64_t)(ros::Time::now().nsec) * (0.001);

	mavlink_msg_set_position_target_local_ned_send(
			MAVLINK_COMM_0,
			_time_now,
			1, 1, 1, 1,
			pv_setpt->setpoint_pose.x,
			pv_setpt->setpoint_pose.y,
			pv_setpt->setpoint_pose.z,
			pv_setpt->setpoint_vel.x,
			pv_setpt->setpoint_vel.y,
			pv_setpt->setpoint_vel.z,
			0,0,0,
			pv_setpt->yaw,
			pv_setpt->yaw_rate);
}

bool update_param(nayan_msgs::param_set::Request &req, nayan_msgs::param_set::Response &res){
	ROS_INFO("update_param %f, %s", req.value, (char*)&req.param_id);
	res.response_string = "got param";

	char* param_name[16];

	memcpy(&param_name, &req.param_id, 16);

	mavlink_msg_param_set_send(
			MAVLINK_COMM_0, 1, 1,
			(const char*)param_name,
			req.value,
			MAV_PARAM_TYPE_REAL32);
	return true;

}


int main(int argc, char **argv){

	ros::init(argc, argv, "nayan_ros");

	ros::NodeHandle n;



	imu_pub = n.advertise<nayan_msgs::nayan_imu>("nayan_ros/imu", 1000);

	debug_var_pub = n.advertise<nayan_msgs::nayan_dbg>("nayan_ros/debug_var", 1000);

	gps_pose_vel_pub = n.advertise<nayan_msgs::nayan_gps_pose_vel>("nayan_ros/gps_pose_vel", 1000);

	attitude_pub = n.advertise<nayan_msgs::nayan_attitude>("nayan_ros/attitude", 1000);

	rc_in_pub = n.advertise<nayan_msgs::nayan_rc_in>("nayan_ros/rc_in", 1000);


	vision_est_sub = n.subscribe("nayan_ros/vision_estimate", 1000, update_vision_estimate);

	pose_vel_setpt_sub = n.subscribe("nayan_ros/setpoint_pose_vel", 1000, update_setpoint_pose_vel);


	param_srv = n.advertiseService("param_set", update_param);


	ros::Duration(2).sleep();

	if (ser.isOpen()){
		ROS_INFO("Serial Port Opened!");
	}else{
		ROS_ERROR("Serial Port Cannot be Opened!");
	}

	ros::Rate loop_rate(100);

	while(ros::ok()){

		update_mavlink();

		ros::spinOnce();

		loop_rate.sleep();
	}


	return 0;
}
