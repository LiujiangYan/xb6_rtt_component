#include <rtt/Component.hpp>

#include "XB6Component.hpp"

namespace xb6_devlib{
	using namespace RTT;

	XB6Component::XB6Component(const std::string& name): TaskContext(name, PreOperational){
		this->ports()->addEventPort("xb6_joint_position", xb6_joint_position);
		this->ports()->addEventPort("xb6_joint_velocity", xb6_joint_velocity);
		this->ports()->addEventPort("xb6_joint_torque", xb6_joint_torque);

		this->ports()->addEventPort("m_xb6_joint_position", m_xb6_joint_position);
		this->ports()->addEventPort("m_xb6_joint_velocity", m_xb6_joint_velocity);
		this->ports()->addEventPort("m_xb6_joint_torque", m_xb6_joint_torque);

		DevInterface dev = new DevInterface();
	}


	XB6Component::~XB6Component(){
	}

	bool XB6Component::configureHook(){
		return true;
	}

	bool XB6Component::startHook(){
		// check connection
		if (!xb6_joint_position.connected())
			return false;
		if (!xb6_joint_velocity.connected())
			return false;
		if (!xb6_joint_torque.connected())
			return false;

		if (!m_xb6_joint_position.connected())
			return false;
		if (!m_xb6_joint_velocity.connected())
			return false;
		if (!m_xb6_joint_torque.connected())
			return false;

		// dev initialize & motor on
		dev.Init();
		dev.MotorOn();

		return true;
	}

	void XB6Component::updateHook(){
		std::vector<double> val(6, 0.0);

		if (xb6_joint_position.read(val) == RTT::NewData){
			dev.setPosData(val);
		}

		if (xb6_joint_velocity.read(val) == RTT::NewData){
			dev.setVelocity(val);
		}

		if (xb6_joint_torque.read(val) == RTT::NewData){
			dev.setTorque(val);
		}

		position = dev.GetPosData(val);
		m_xb6_joint_position.write(position);
		velocity = dev.GetCurVelocity(val);
		m_xb6_joint_velocity.write(velocity);
		torque = dev.GetCurTorque(val);
		m_xb6_joint_torque.write(torque);
	}
}
