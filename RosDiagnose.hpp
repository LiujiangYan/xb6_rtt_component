class RosDiagnose{
public:
	//RosDiagnose():m_joint_cmd(6),m_pub_started(false),m_started(false){}
	RosDiagnose():m_started(false){}
	~RosDiagnose(){}

	int StartNode(){
		if(m_started)	return 0;

		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "XB6_DIAGNOSE");
		ros::NodeHandle nh;

		m_rt_planned_cmd_pub.reset( new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(nh,"/realtime_planned_jntcmd",1));
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