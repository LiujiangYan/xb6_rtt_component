
#ifndef TWO_COMPONENTS_HPP_
#define TWO_COMPONENTS_HPP_

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/OutputPort.hpp>
#include <iostream>
#include <vector>
#include <rtt/rtt-config.h>

#include <devlib>

using namespace RTT;
using namespace std;

class xb6recvcomponent : public TaskContext{
private:
    InputPort<vector<double> > input_pos_cmd;
    OutputPort<vector<double> > output_pos_current;

    std::vector<double> joint_cmd; // 接受的指令
    std::vector<double> joint_current; // 发送的setpoint

    DevInterface dev;

//,PreOperational
public:
    xb6recvcomponent(std::string name):TaskContext(name,PreOperational),
		input_pos_cmd("input_pos_cmd"),
		output_pos_current("output_pos_current"),
		joint_cmd(6,0.0),
		joint_current(6,0.0){
    	// define the type of outputPort
    	vector<double> example(6, 0.0);
    	output_pos_current.setDataSample(example);
    	output_pos_current.write(example);

    	this->ports()->addPort(input_pos_cmd).doc("Reads incoming command");
    	this->ports()->addPort(output_pos_current).doc("sends current position");

        dev.Init();
        dev.OpenDevice();
        dev.MotorOn();

    }

    bool configureHook(){
    	if(!input_pos_cmd.connected()){
    		log(Info)<<"input.connected is wrong"<<endlog();
    		return false;
    	}
    	if(!output_pos_current.connected()){
    		log(Info)<<"output.connected is wrong"<<endlog();
    		return false;
    	}
        if(!dev.IsMotorOn()){
            log(Info)<<"Motor is not on"<<endlog();
            return false;
        }
    	return true;
    }

    void updateHook(){
    	if(input_pos_cmd.read(joint_cmd) == NewData){
    		dev.SetPosData(joint_cmd);
    	}

    	for (int i=0; i<6; i++){
    		cout << joint_cmd[i] << " ";
   		}
    	cout << "\n";

        dev.GetPosData(joint_current);
    	output_pos_current.write(joint_current);
    }

    void stopHook(){
        dev.MotorOff();
    }

};
