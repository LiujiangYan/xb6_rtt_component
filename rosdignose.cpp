/*
 * ros_diagnose.cpp
 *
 *  Created on: May 20, 2016
 *      Author: hanson
 */

#include <ros/ros_diagnose.hpp>
#include "ros_node.hpp"
#include <rtt/Logger.hpp>

using namespace RTT;
using namespace KDL;
RosDiagnose* RosDiagnose::m_instance = NULL;

RosDiagnose::RosDiagnose():
		m_joint_cmd(6),
		m_last_joint_cmd(6),
		m_pub_started(false),
		m_started(false){
}
RosDiagnose::~RosDiagnose(){
}

RosDiagnose *RosDiagnose::GetInstance(){

	if(m_instance == NULL) {
		m_instance = new RosDiagnose();
	}
	return m_instance;
}


int RosDiagnose::StartNode(){
	if(m_started){
		return 0;
	}
	int argc = 0;
	char **argv = NULL;
	ros::init(argc, argv, "LIRCOS_DIAGNOSE");
	ros::NodeHandle nh;

	m_rt_jntstate_servo_pub.reset( new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(nh,"/realtime_joint_states_from_servo",1));
	m_rt_jntstate_servo_pub->msg_.position.resize(6);
	m_rt_jntstate_servo_pub->msg_.velocity.resize(6);
	m_rt_jntstate_servo_pub->msg_.effort.resize(6);

	m_rt_planned_cmd_pub.reset( new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(nh,"/realtime_planned_jntcmd",1));
	m_rt_planned_cmd_pub->msg_.position.resize(6);
	m_rt_planned_cmd_pub->msg_.velocity.resize(6);
	m_rt_planned_cmd_pub->msg_.effort.resize(6);

	m_rt_planned_path_vel_pub.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64>(nh,"/realtime_planned_path_vel",1));

	m_started = true;
	return 0;
}

void RosDiagnose::StopNode(){

	ros::shutdown();
	if(!ros::ok()) // shutdown需要时间,应该进不来
	{
	}
}

void RosDiagnose::Destroy(){
	if(m_instance != NULL)
	{
		delete m_instance;
		m_instance = NULL;
	}
}


void RosDiagnose::PubJntStateFromServoRealTime(const JntArrayAcc& ja){
	if(m_rt_jntstate_servo_pub->trylock()){
		for(int i=0;i<6;++i){
			m_rt_jntstate_servo_pub->msg_.position[i] = ja.q(i);
			m_rt_jntstate_servo_pub->msg_.velocity[i] = ja.qdot(i);
			m_rt_jntstate_servo_pub->msg_.effort[i] = ja.qdotdot(i);
		}
		m_rt_jntstate_servo_pub->msg_.header.stamp = ros::Time::now();
		m_rt_jntstate_servo_pub->unlockAndPublish();
	}
}
void RosDiagnose::PubJntStateRealTime(const JntArrayAcc& ja, const ros::Time& time){

	if(m_rt_jntstate_servo_pub->trylock()){
		for(int i=0;i<6;++i){
			m_rt_jntstate_servo_pub->msg_.position[i] = ja.q(i);
			m_rt_jntstate_servo_pub->msg_.velocity[i] = ja.qdot(i);
			m_rt_jntstate_servo_pub->msg_.effort[i] = ja.qdotdot(i);
		}
		if(time == ros::Time(0.0)){
			m_rt_jntstate_servo_pub->msg_.header.stamp = ros::Time::now();
		}else{
			m_rt_jntstate_servo_pub->msg_.header.stamp = time;
		}
		m_rt_jntstate_servo_pub->unlockAndPublish();
	}
}

void RosDiagnose::PubSetpointRealTime(const JntArray& setpoint, double interp_period){
	if(!m_pub_started){
		m_last_joint_cmd.q = setpoint;
		m_pub_started = true;
	}

	double vel;
	if(m_rt_planned_cmd_pub->trylock()){
		for(int i=0;i<6;++i){
			m_rt_planned_cmd_pub->msg_.position[i] = setpoint(i);
			vel = (setpoint(i) - m_last_joint_cmd.q(i))/interp_period;
			m_rt_planned_cmd_pub->msg_.velocity[i] = vel;
			m_rt_planned_cmd_pub->msg_.effort[i] = (vel - m_last_joint_cmd.qdot(i))/interp_period;
			m_last_joint_cmd.qdot(i) = vel;
			m_last_joint_cmd.q(i) = setpoint(i);
		}
		m_rt_planned_cmd_pub->msg_.header.stamp = ros::Time::now();
		m_rt_planned_cmd_pub->unlockAndPublish();
	}
}
void RosDiagnose::PubSetpointAccRealTime(const JntArrayAcc& setpoint){
	if(m_rt_planned_cmd_pub->trylock()){
		for(int i=0;i<6;++i){
			m_rt_planned_cmd_pub->msg_.position[i] = setpoint.q(i);
			m_rt_planned_cmd_pub->msg_.velocity[i] = setpoint.qdot(i);
			m_rt_planned_cmd_pub->msg_.effort[i] = setpoint.qdotdot(i);
		}
		m_rt_planned_cmd_pub->msg_.header.stamp = ros::Time::now();
		m_rt_planned_cmd_pub->unlockAndPublish();
	}
}

/*
 * @brief 发布沿路径的sd
 */
void RosDiagnose::PubTrajVelRealTime(double sd){
	if(m_rt_planned_path_vel_pub->trylock()){
		m_rt_planned_path_vel_pub->msg_.data = sd;
		m_rt_planned_path_vel_pub->unlockAndPublish();
	}
}
//#endif
