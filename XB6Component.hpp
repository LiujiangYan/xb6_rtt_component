#ifndef _XB6_COMPONENT_HPP_
#define _XB6_COMPONENT_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>

#include </typekit/Types.hpp>

#include <devlib>

namespace xb6_devlib{

	class XB6Component : public TaskContext{
	public:
		XB6Component(const std::string& name);
		~XB6Component();

		virtual bool configureHook();
		virtual bool startHook();
			
		virtual void updateHook();
		virtual void stopHook();
		virtual void cleanupHook();

	private:
		DevInterface dev;

		InputPort<std::vector<double>> xb6_joint_position;
		InputPort<std::vector<double>> xb6_joint_velocity;
		InputPort<std::vector<double>> xb6_joint_torque;

		OutputPort<std::vector<double>> m_xb6_joint_position;
		OutputPort<std::vector<double>> m_xb6_joint_velocity;
		OutputPortputPort<std::vector<double>> m_xb6_joint_torque;

		std::vector<double> example(6, 0.0);

		xb6_joint_position.setDataSample(example);
		xb6_joint_position.write(example);
		xb6_joint_velocity.setDataSample(example);
		xb6_joint_velocity.write(example);
		xb6_joint_torque.setDataSample(example);
		xb6_joint_torque.write(example);

		m_xb6_joint_position.setDataSample(example);
		m_xb6_joint_position.write(example);
		m_xb6_joint_velocity.setDataSample(example);
		m_xb6_joint_velocity.write(example);
		m_xb6_joint_torque.setDataSample(example);
		m_xb6_joint_torque.write(example);
	};
}

#endif//_XB6_COMPONENT_HPP_
