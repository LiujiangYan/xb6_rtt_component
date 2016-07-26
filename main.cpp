#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/os/main.h>

#include <iostream>
#include "two_components.hpp"
#include "xb6pubcomponent.hpp"

using namespace RTT;
using namespace std;

int ORO_main(int argc, char** argv){

	xb6pubcomponent xb6pub("xb6pub");
	Hello hello("hello");

	connectPeers(&hello, &xb6pub);

	xb6pub.setPeriod(0.002);
	hello.setPeriod(0.002);

	Logger::Instance()->setLogLevel(Logger::Info);

	xb6pub.getPort("xb6_pos_cmd")->connectTo(hello.getPort("input_pos_cmd"));
	hello.getPort("output_setpoint")->connectTo(xb6pub.getPort("xb6_pos_current"));

	cout<<xb6pub.getPort("xb6_pos_cmd")->connected()<<endl;
	cout<<hello.getPort("output_setpoint")->connected()<<endl;

	if(!hello.start()){
		hello.configure();
		hello.start();
	}
	int res = xb6pub.configure();
	cout<<res<<endl;
	xb6pub.start();

	char a;
	std::cin>>a;
	//	ros::shutdown();
	return 0;
}
