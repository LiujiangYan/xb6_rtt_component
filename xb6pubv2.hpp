#ifndef XB6_PUB_COMPONENT_HPP
#define XB6_PUB_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/os/main.h>
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

	SineSweep(string const& name){
		K_ = 0.0;
		L_ = 0.0;
		amplitude_ = 0.0;
		duration_ = 0.0;
		cmd_ = 0.0;
	}

	~SineSweep(){}

	bool init(double start_freq, double end_freq, double duration, double amplitude){
		if (start_freq > end_freq) return false;
		if (duration < 0) return false;
		if (amplitude < 0) return false;

		amplitude_ = amplitude;
		duration_ =duration;

		start_angular_freq_ =2*M_PI*start_freq;
		end_angular_freq_ =2*M_PI*end_freq;

		K_ = (start_angular_freq_*duration)/log(end_angular_freq_/start_angular_freq_);
		L_ = (duration)/log(end_angular_freq_/start_angular_freq_);

		cmd_ = 0.0;

		return true;
	}

	double update(double dt){
		if (dt <= duration_){
			cmd_ = amplitude_*sin(K_*(exp(dt)/(L_))-1);
		}
		else{
			cmd_ = 0.0;
		}
		return cmd_;
	}
};


class xb6pubcomponent : public RTT::TaskContext, xb6_pos_cmd("xb6_pos_cmd"), xb6_pos_current("xb6_pos_current"){
public:
	SineSweep sw;
	vector<double> joint_pos_command;
	vector<double> current_pos;
	vector<double> init_pos;
	double dt;

	xb6pubcomponent(string const& name){
		vector<double> example(6, 0.0);
		xb6_pos_cmd.setDataSample(example);
		xb6_pos_cmd.write(example);
		
		this->ports()->addPort("xb6_pos_cmd", xb6_pos_cmd);
		this->ports()->addPort("xb6_pos_current", xb6_pos_current);
		//this.setPeriod(0.002);

		SineSweep sw("sw");
		sw.init(20, 2000, 10, 2);

		init_pos.resize(6);
		dt = 0.0;
	}

	~xb6pubcomponent(){}

	bool configureHook(){
		if (!xb6_pos_cmd.connected())
			return false;
		if (!xb6_pos_current.connected())
			return false;
		return true;
	}

	bool startHook(){
		return true;
	}

	void updateHook(){
		xb6_pos_current.read(current_pos); 

		for (int i=0; i<6; i++){
			double cmd = sw.update(dt);
			joint_pos_command[i] = 2 * M_PI * cmd; 
		}

		xb6_pos_cmd.write(joint_pos_command);

		dt += 0.002;
		if (dt > 2){
			dt = 0;
		}
	}

	void stopHook(){}

private:
	OutputPort<vector<double> > xb6_pos_cmd;
	InputPort<vector<double> > xb6_pos_current;
};


#endif

