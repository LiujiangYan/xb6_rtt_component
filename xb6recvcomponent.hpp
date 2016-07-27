#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/OutputPort.hpp>
#include <iostream>
#include <vector>
#include <rtt/rtt-config.h>
#include "dev_interface.hpp"

using namespace RTT;
using namespace std;

class xb6recvcomponent : public TaskContext{
private:
    InputPort<vector<double> > input_pos_cmd;
    OutputPort<vector<double> > output_pos_current;

    std::vector<double> joint_cmd; // 接受指令位置
    std::vector<double> joint_current; // 发送当前位置

    DevInterface* dev;

//PreOperational
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

		dev = DevInterface::GetInstance();

		std::string file_name= "dev_eni.xml";
		BusInfo bus_info;
		bus_info.bus_type = ETHERCAT_BUS;
		bus_info.bus_cyc_time = 2000;
		bus_info.adapter = I8254X;
		bus_info.instance = 1;
		bus_info.link_mode = 1;
		bus_info.verbose = 3;
		bus_info.conf_type = eCnfType_Filename;
		bus_info.pby_conf_data = (EC_T_PBYTE)file_name.c_str();
		bus_info.conf_data_len = 256;
		bus_info.ec_mode = eDcmMode_BusShift;
		bus_info.motor_num = 6;
		bus_info.io_board_num = 1;

		int encoder_offset[6] = {0,0,0,0,0,0};
		int encoder_resolution[6] = {131072,131072,131072,131072,131072,131072};
		double reduce_retio[6]= {100,100,100,100,100,100};

		dev->Init(&bus_info,false,encoder_offset,encoder_resolution,reduce_retio);
        dev->OpenDevice();
        dev->MotorOn(0);

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
        if(!dev->IsMotorOn()){
            log(Info)<<"Motor is not on"<<endlog();
            return false;
        }
    	return true;
    }

    void updateHook(){
 		double setpos[6];
    	if(input_pos_cmd.read(joint_cmd) == NewData){
    		for (int i=0; i<6; i++){
    			setpos[i] = joint_cmd[i];
    		}
    	}
 		dev->SetPosData(setpos);

//    	for (int i=0; i<6; i++){
//    		cout << joint_cmd[i] << " ";
//   		}
//    	cout << "\n";

 		double curpos[6];
        dev->GetPosData(curpos);
        for(int i=0; i<6; i++){
        	joint_current[i] = curpos[i];
        }
    	output_pos_current.write(joint_current);
    }

    void stopHook(){
        dev->MotorOff(0);
    }

};
