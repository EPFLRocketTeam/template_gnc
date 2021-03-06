/*
* Node to synchronize the GNC algorithms by estimating the current state of the rocket
* (Idle, Rail, Launch, Coast)
*
* Inputs: 
*   - Sensor data (IMU and barometer) from av_interface: /sensor_pub
*   - Estimated state from template_navigation:		     /kalman_rocket_state
*	- Commanded gimbal state from template_control		 /gimbal_command_0
*
* Important parameters:
*   - Threshold for rocket ignition detection (Rail phase): in FSM_thread
*
* Outputs:
*   - Estimated finite state machine from flight data:	/gnc_fsm_pub
*
*/

#include "ros/ros.h"
#include "real_time_simulator/FSM.h"
#include "real_time_simulator/State.h"
#include "real_time_simulator/Sensor.h"
#include "real_time_simulator/GimbalMsg.h"

#include <time.h>

#include <sstream>
#include <string>

#include "real_time_simulator/GetFSM.h"
#include "std_msgs/String.h"

class FsmNode {
	private:

		// Current time and state machine
		real_time_simulator::FSM rocket_fsm;
		double time_zero;

		// Last received sensor data
		real_time_simulator::Sensor rocket_sensor;

		// Last received feedback control
		real_time_simulator::GimbalMsg gimbal_state;

		// Last received rocket state
		real_time_simulator::State rocket_state;

		// List of subscribers and publishers
		ros::ServiceServer timer_service;

		ros::Publisher timer_pub;

		ros::Subscriber rocket_state_sub;
		ros::Subscriber sensor_sub;
		ros::Subscriber control_sub;

		// Other parameters
		float rail_length = 0;

	public:
		FsmNode(ros::NodeHandle &nh)
		{
			// Initialize publishers and subscribers
        	initTopics(nh);

			// Initialize fsm
			rocket_fsm.time_now = 0;
			rocket_fsm.state_machine = "Idle";

			// Overwrite rocket mass to stay in launch mode at first iteration
			rocket_state.propeller_mass = 10;  


			timer_pub.publish(rocket_fsm);

			nh.getParam("/environment/rail_length", rail_length);
		}

		void initTopics(ros::NodeHandle &nh) 
		{
			// Create timer service
			timer_service = nh.advertiseService("getFSM_gnc", &FsmNode::sendFSM, this);

			// Create timer publisher and associated thread (100Hz)
			timer_pub = nh.advertise<real_time_simulator::FSM>("gnc_fsm_pub", 10);

			// Subscribe to state message
			rocket_state_sub = nh.subscribe("kalman_rocket_state", 1, &FsmNode::rocket_stateCallback, this);

			// Subscribe to sensors message
			sensor_sub = nh.subscribe("sensor_pub", 1, &FsmNode::sensorCallback, this);

			// Subscribe to commanded control message
			control_sub = nh.subscribe("gimbal_command_0", 1, &FsmNode::controlCallback, this);
		}

		void rocket_stateCallback(const real_time_simulator::State::ConstPtr& new_rocket_state)
		{
			rocket_state.pose = new_rocket_state->pose;
			rocket_state.twist = new_rocket_state->twist;
			rocket_state.propeller_mass = new_rocket_state->propeller_mass;
		}

		// Callback function to store last received sensor data
		void sensorCallback(const real_time_simulator::Sensor::ConstPtr& new_sensor)
		{
			rocket_sensor.IMU_acc = new_sensor->IMU_acc;
			rocket_sensor.IMU_gyro = new_sensor->IMU_gyro;
			rocket_sensor.baro_height = new_sensor->baro_height;
		}

		// Callback function to store last received commanded control
		void controlCallback(const real_time_simulator::GimbalMsg::ConstPtr& state)
		{
			gimbal_state.outer_angle = state->outer_angle;
			gimbal_state.inner_angle = state->inner_angle;
			gimbal_state.thrust = state->thrust;
		}

		// Service function: send back fsm (time + state machine)
		bool sendFSM(real_time_simulator::GetFSM::Request &req, real_time_simulator::GetFSM::Response &res)
		{
			// Update current time
			if (rocket_fsm.state_machine.compare("Idle") != 0) rocket_fsm.time_now = ros::Time::now().toSec() - time_zero;

			res.fsm.time_now = rocket_fsm.time_now;
			res.fsm.state_machine = rocket_fsm.state_machine;
			
			return true;
		}

		void updateFSM()
		{
			// Idle mode: wait for high acceleration to trigger flight mode
			if (rocket_fsm.state_machine.compare("Idle") == 0)
			{
				if(rocket_sensor.IMU_acc.z > 30)
				{

					rocket_fsm.state_machine = "Rail";
					time_zero = ros::Time::now().toSec();
				}
			}

			else
			{
				// Update current time
				rocket_fsm.time_now = ros::Time::now().toSec() - time_zero;
				
				if (rocket_fsm.state_machine.compare("Rail") == 0)
				{
					// End of rail
					if(rocket_state.pose.position.z > rail_length)
					{
						rocket_fsm.state_machine = "Launch";
					}

				}

				else if (rocket_fsm.state_machine.compare("Launch") == 0)
				{
					// End of burn -> no more thrust
					if(rocket_state.propeller_mass < 0 || gimbal_state.thrust == 0)
					{
						rocket_fsm.state_machine = "Coast";
					}

				}

				else if (rocket_fsm.state_machine.compare("Coast") == 0)
				{
					// Do nothing for now
				}

				// Publish time + state machine    
				timer_pub.publish(rocket_fsm);
			}
		}


};







int main(int argc, char **argv)
{

	// Init ROS time keeper node
	ros::init(argc, argv, "gnc_fsm");
	ros::NodeHandle nh;

	FsmNode fsmNode(nh);
	
	// Thread to compute FSM. Duration defines interval time in seconds
	ros::Timer FSM_thread = nh.createTimer(ros::Duration(0.001),
	[&](const ros::TimerEvent&) 
	{
		fsmNode.updateFSM();
	});

	// Automatic callback of service and publisher from here
	ros::spin();

}
