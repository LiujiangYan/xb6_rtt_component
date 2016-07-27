//#include <rtt/os/main.h>
//#include "dev_interface.hpp"
//
//#include <iostream>
//using namespace std;
//int ORO_main(int argc, char** argv){
//	DevInterface* dev = DevInterface::GetInstance();
//
//	std::string file_name= "dev_eni.xml";
//	BusInfo bus_info;
//	bus_info.bus_type = ETHERCAT_BUS;
//	bus_info.bus_cyc_time = 2000;
//	bus_info.adapter = I8254X;
//	bus_info.instance = 1;
//	bus_info.link_mode = 1;
//	bus_info.verbose = 3;
//	bus_info.conf_type = eCnfType_Filename;
//	bus_info.pby_conf_data = (EC_T_PBYTE)file_name.c_str();
//	bus_info.conf_data_len = 256;
//	bus_info.ec_mode = eDcmMode_BusShift;
//	bus_info.motor_num = 6;
//	bus_info.io_board_num = 1;
//
//	int encoder_offset[6] = {0,0,0,0,0,0};
//	int encoder_resolution[6] = {131072,131072,131072,131072,131072,131072};
//	double reduce_retio[6]= {100,100,100,100,100,100};
//
//	RTT::Logger::Instance()->setLogLevel(RTT::Logger::Info);
//	int res = dev->Init(&bus_info,false,encoder_offset,encoder_resolution,reduce_retio);
//
//	res = dev->OpenDevice();
//
//	cout<<"alarm : "<<dev->GetAlarmInfo()<<endl;
//
////	res = dev->RestartBusMaster();
//
//	if(!dev->IsDevSysNormal()){
//		DevCmd cmd;
//		cmd.cmd = DEV_SYS_CLEAR_ALARM;
//		dev->SendCommand(cmd);
//	}
//
//	dev->ConfigIOBoard(1);
//	cout<<"alarm : "<<dev->GetAlarmInfo()<<endl;
//
//	sleep(1);
//	res = dev->MotorOff(0);
//	sleep(3);
//	res = dev->MotorOn(0);
//	cout<< "res = "<<res<<endl;
//
//	cout<<"dev->IsMotorOn() = "<<dev->IsMotorOn()<<endl;
//
//	double pos[6];
//	cout<<dev->GetPosData(pos)<<endl;
//	cout<<pos[0]<<endl;
//
//	while(1){
//		sleep(1);
//	}
//
//	return 0;
//}
#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/os/main.h>

#include <iostream>
#include "xb6recvcomponent.hpp"
#include "xb6pubcomponent.hpp"

using namespace RTT;
using namespace std;

int ORO_main(int argc, char** argv){

	//cout << "step 1";
	xb6pubcomponent xb6pub("xb6pub");
	xb6recvcomponent xb6recv("xb6recv");

	connectPeers(&xb6recv, &xb6pub);

	xb6pub.setPeriod(0.002);
	xb6recv.setPeriod(0.002);

	Logger::Instance()->setLogLevel(Logger::Info);

	xb6pub.getPort("xb6_pos_cmd")->connectTo(xb6recv.getPort("input_pos_cmd"));
	xb6recv.getPort("output_setpoint")->connectTo(xb6pub.getPort("xb6_pos_current"));

	cout<<xb6pub.getPort("xb6_pos_cmd")->connected()<<endl;
	cout<<xb6recv.getPort("output_setpoint")->connected()<<endl;

	xb6recv.configure();
	xb6recv.start();

	xb6pub.configure();
	xb6pub.start();

	char a;
	std::cin>>a;
	//	ros::shutdown();
	return 0;
}
