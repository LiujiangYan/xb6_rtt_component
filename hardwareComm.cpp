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

	HardwareInterface hd("hd");
	xb6pubcomponent xb6pub("xb6pub");

	connectPeers(&hd, &xb6pub);

	hd.setPeriod(0.002);
	xb6pub.setPeriod(0.002);

	Logger::Instance()->setLogLevel(Logger::Info);

	xb6pub.getPort("xb6_pos_cmd")->connectTo(hd.getPort("")); // name of hd's inputport
	hd.getPort(" ")->connectTo(xb6pub.getPort("xb6_pos_current"));

	cout<<xb6pub.getPort("xb6_pos_cmd")->connected()<<endl;
	cout<<hd.getPort("output_setpoint")->connected()<<endl;

	xb6pub.configure();
	//xb6pub.start();



	// shutdown
	char a;
	std::cin>>a;
	//	ros::shutdown();
	return 0;

}