#ifndef SINESWEEP_PUB_COMPONENT_HPP
#define SINESWEEP_PUB_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/os/main.h>
#include <rtt/Logger.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/rtt-config.h>

#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>
#include <boost/scoped_ptr.hpp>

#include <iostream>
#include <math.h>
#include <vector>

using namespace RTT;
using namespace std;

class SineSweep{
public:
	double K_;
	double L_;
	double amplitude_;
	double duration_;
	double start_angular_freq_;
	double end_angular_freq_;
	double cmd_;

	SineSweep(){
		K_ = 0.0;
		L_ = 0.0;
		amplitude_ = 0.0;
		duration_ = 0.0;
		cmd_ = 0.0;
		start_angular_freq_ = 0.0;
		end_angular_freq_ = 0.0;
	}

	~SineSweep(){}

	bool init(double start_freq, double end_freq, double duration, double amplitude){
		if (start_freq > end_freq) return false;
		if (duration < 0) return false;
		if (amplitude < 0) return false;

		amplitude_ = amplitude;
		duration_ = duration;

		start_angular_freq_ = 2*M_PI*start_freq;
		end_angular_freq_ = 2*M_PI*end_freq;

		K_ = (start_angular_freq_*duration)/log(end_angular_freq_/start_angular_freq_);
		L_ = (duration)/log(end_angular_freq_/start_angular_freq_);

		cmd_ = 0.0;

		return true;
	}

	double update(double dt){
		if (dt <= duration_){
			cmd_ = amplitude_*sin(K_*(exp((dt)/(L_))-1));
		}
		else{
			cmd_ = 0.0;
		}
		return cmd_;
	}

	double getVelocity(double dt){
		return amplitude_*cos(K_*(exp((dt)/(L_))-1));
	}
};

class RosDiagnose{
public:
	//RosDiagnose():m_joint_cmd(6),m_pub_started(false),m_started(false){}
	RosDiagnose():m_started(false){}
	~RosDiagnose(){}

	int StartNode(string name){
		if(m_started)	return 0;

		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "SINESWEEP_DIAGNOSE");
		ros::NodeHandle nh;

		//m_rt_planned_cmd_pub.reset( new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(nh,"/realtime_planned_jntcmd",1));
		m_rt_planned_cmd_pub.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(nh,name,1));
		m_rt_planned_cmd_pub->msg_.position.resize(6);

		m_started = true;
		return 0;
	}

	void StopNode(){
		ros::shutdown();
	}

	void PubSetpointRealTime(const vector<double> setpoint){
		if(m_rt_planned_cmd_pub->trylock()){
			for(int i=0;i<6;++i){
				m_rt_planned_cmd_pub->msg_.position[i] = setpoint[i];
			}
			m_rt_planned_cmd_pub->msg_.header.stamp = ros::Time::now();
			m_rt_planned_cmd_pub->unlockAndPublish();
		}
	}

private:
	bool m_started;
	//bool m_pub_started;
	boost::scoped_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > m_rt_planned_cmd_pub;
	//vector<double> m_joint_cmd;
};

class sinesweepComponent : public RTT::TaskContext{
public:
	SineSweep sw;
	RosDiagnose rd_pub;
	RosDiagnose rd_recv;
	vector<double> joint_pos_command;
	vector<double> current_pos;
	vector<double> init_pos;
	double dt;
	bool m_stopping;
	int sign;

	sinesweepComponent(string const& name):RTT::TaskContext(name, PreOperational),
					joint_pos_command(6,0.0),
					current_pos(6,0.0),
					init_pos(6,0.0),
					xb6_pos_cmd("sinesweep_pos_cmd"),
					xb6_pos_current("sinesweep_pos_current"),
					dt(0.0),
					m_stopping(false),
					sign(1){
		vector<double> example(6, 0.0);
		xb6_pos_cmd.setDataSample(example);
		xb6_pos_cmd.write(example);

		this->ports()->addPort("sinesweep_pos_cmd", xb6_pos_cmd);
		this->ports()->addPort("sinesweep_pos_current", xb6_pos_current);

		this->addOperation("turnOff", &sinesweepComponent::turnOff, this, OwnThread);
		this->addOperation("turnOn", &sinesweepComponent::sinesweepSet, this, OwnThread);
		this->addOperation("sinesweepSet", &sinesweepComponent::sinesweepSet, this, OwnThread);

		//start frequenct, end frequency, duration, amplitude
		//sw.init(1, 10, 10, 3.14/180.0);
		
		rd_pub.StartNode("/realtime_planned_jntcmd");
		rd_recv.StartNode("/realtime_current_jnt");
	}

	~sinesweepComponent(){}

	bool sinesweepSet(double start_freq, double end_freq, double duration, double amplitude){
		sw.init(start_freq, end_freq, duration, amplitude);
		return ture;
	}

	bool configureHook(){
		if (!xb6_pos_cmd.connected()){
			log(Info)<<"sinesweep_pos_cmd.connected is wrong"<<endlog();
			return false;
		}
		if (!xb6_pos_current.connected()){
			log(Info)<<"sinesweep_pos_current.connected is wrong"<<endlog();
			return false;
		}
		return true;
	}

	void updateHook(){

		if(m_stopping){
			dt = 0;
		}
		else{
			if (sign > 0){
				for (int i=0; i<6; i++){
					double cmd = sw.update(dt);
					joint_pos_command[i] = cmd + offset;
					//cout << joint_pos_command[i] << " ";
				}
				dt += 0.002;
			}
			if (sign < 0){
				for (int i=0; i<6; i++){
					double cmd = sw.update(dt);
					joint_pos_command[i] = -cmd + offset;
					//cout << joint_pos_command[i] << " ";
				}
				dt -= 0.002;
			}
			if (dt > 10 || dt < 0){
				offset = cmd * sign;
				sign = -sign;
			}

			// for (int i=0; i<6; i++){
			// 	double cmd = sw.update(dt);
			// 	joint_pos_command[i] = cmd;
			// }
			// dt += 0.002;

			// if (dt > sw.duration_){
			// 	m_stopping = true;
			// }

			sinesweep_pos_cmd.write(joint_pos_command);
			sinesweep_pos_current.read(current_pos);

			rd_pub.PubSetpointRealTime(joint_pos_command);
			rd_recv.PubSetpointRealTime(current_pos);
		}
	}

	bool turnOn(){
		m_stopping = false;
		return true;
	}

	bool turnOff(){
		m_stopping = true;
		return true;
	}

	void stopHook(){}

private:
	OutputPort<vector<double> > sinesweep_pos_cmd;
	InputPort<vector<double> > sinesweep_pos_current;
};

#endif


