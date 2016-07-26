
#ifndef TWO_COMPONENTS_HPP_
#define TWO_COMPONENTS_HPP_

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/OutputPort.hpp>
#include <iostream>
#include <vector>
#include <rtt/rtt-config.h>

using namespace RTT;
using namespace std;

/*
 * 两个component, 每个都有两部分, 一是收发double的例子, 二是收发joint_cmd的可能实现
 */

class Hello : public TaskContext{


protected:
    /**
     * OutputPorts publish data.
     */
    OutputPort<double> output;
    /**
     * InputPorts read data.
     */
    InputPort< double > input;

    /**
     * Since read() requires an argument, we provide this
     * attribute to hold the read value.
     */
    double read_helper;


    /*
     * RTT本身可以实时传输vector<double>的数据
     */
    InputPort<vector<double> > input_pos_cmd;
    // 接受指令相当于:
    // cmd_sub = nh.subscribe("input_pos_cmd")


    OutputPort<vector<double> > output_setpoint;
    // 相当于 setpoint_pub = nh.publish<SetPoint>("output_setpoint")


    std::vector<double> jntcmd; // 接受的指令

    std::vector<double> interpolated_setpoint; // 发送的setpoint



//,PreOperational
public:
    Hello(std::string name):
    	TaskContext(name,PreOperational),
		output("output",0.0), // 名字和初始值
		input("input"),
		read_helper(0.0),
		input_pos_cmd("input_pos_cmd"),
//		output_setpoint("interpolated_setpoint"),
		output_setpoint("output_setpoint"),
		jntcmd(6,0.0),
		interpolated_setpoint(6,0.0)
	{
    	// define the type of outputPort
    	vector<double> example(6, 0.0);
    	output_setpoint.setDataSample(example);
    	output_setpoint.write(example);

    	this->addAttribute("read_helper",read_helper);
    	this->ports()->addPort(output).doc("Data producing port.");
    	this->ports()->addPort(input).doc("Data consuming port.");

    	this->ports()->addPort(input_pos_cmd).doc("Reads incoming command");
    	this->ports()->addPort(output_setpoint).doc("sends interpolated setpoint");

    }

    bool configureHook(){
    	if(!input_pos_cmd.connected()){
    		log(Info)<<"input.connected is wrong"<<endlog();
    		return false;
    	}
    	if(!output_setpoint.connected()){
    		log(Info)<<"output.connected is wrong"<<endlog();
    		return false;
    	}
    	return true;
    	//return input.connected();
    }

    void updateHook(){

    	if(input.read(read_helper) == NewData){
    		log(Info)<<"Received data: "<<read_helper<<endlog();
    		output.write(read_helper);
    	}

    	if(input_pos_cmd.read(jntcmd) == NewData){
    		interpolate();
    	}
    	for (int i=0; i<6; i++){
    		cout << jntcmd[i] << " ";
   		}
    	cout << "\n";
    	output_setpoint.write(interpolated_setpoint);


    }

protected:
    void interpolate(){}
};
