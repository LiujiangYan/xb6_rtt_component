/*
 * ros_diagnose.hpp
 *
 *  Created on: May 20, 2016
 *      Author: hanson
 */

#ifndef ROS_DIAGNOSE_HPP_
#define ROS_DIAGNOSE_HPP_

#include <boost/scoped_ptr.hpp>
#include <util/unique_queue.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <kdl/jntarrayacc.hpp>

using namespace std;
using namespace KDL;

/*
 * 使用ros的realtime publisher来诊断控制器的状态
 */

class RosDiagnose{
public:
	/**
	 * @brief 通过单例模式实现RosDiagnose
	 * @return RosDiagnose的实例化指针
	 */
	static RosDiagnose* GetInstance();

	static void Destroy();

	/**
	 * @brief 建立控制系统与ROS之间的通信，在非实时线程中调用
	 */
	int StartNode();

	/**
	 * @brief 停止控制系统与ROS之间的通信，在非实时线程中调用
	 */
	void StopNode();

	/*--------------   sensor ---------------*/
	/*
	 * @brief 调用realtime_tools,在实时线程中发布, 通过伺服读上来的轴位置、轴速度、轴力矩
	 */
	void PubJntStateFromServoRealTime(const JntArrayAcc& ja);

	/*
	 * @brief caller控制每个topic的时间戳,与PubJntStateFromServoRealTime公用一个publisher
	 * @param ja 位置/速度/力矩(或加速度)
	 */
	void PubJntStateRealTime(const JntArrayAcc& ja, const ros::Time& time=ros::Time(0.0));


	/*-------------   command ---------------*/
	/*
	 * @brief 发布规划出来的指令点
	 */
	void PubSetpointRealTime(const JntArray& setpoint, double interp_period=0.002);
	/**
	 * @brief publish planned jntarrayacc
	 */
	void PubSetpointAccRealTime(const JntArrayAcc& setpoint);
	/*
	 * @brief 发布沿路径的sd
	 */
	void PubTrajVelRealTime(double sd);

	/*
	 * @brief 如果ros ok且存在发布者时,才会ok
	 */
	bool IsOK(){return (ros::ok() and m_started);}


	RosDiagnose();
	~RosDiagnose();
private:
	static RosDiagnose* m_instance;

	boost::scoped_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > m_rt_jntstate_servo_pub;

	boost::scoped_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > m_rt_planned_cmd_pub;

	boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64> > m_rt_planned_path_vel_pub;

	bool m_started;

	bool m_pub_started;

	JntArrayAcc m_joint_cmd;
	JntArrayAcc m_last_joint_cmd;
};

#endif
//#endif /* ROS_NODE_H_ */

