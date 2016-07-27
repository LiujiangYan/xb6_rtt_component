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
