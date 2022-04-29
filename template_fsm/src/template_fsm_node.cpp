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
#include "rocket_utils/FSM.h"
#include "rocket_utils/State.h"
#include "rocket_utils/GimbalControl.h"
#include "rocket_types/rocket_types.h"
#include "rocket_types/ros_conversions.h"

#include <sstream>
#include <string>

#include "std_msgs/String.h"

using namespace rocket;

class FsmNode
{
private:
  // Current state machine
  RocketFSMState current_fsm;
  ros::Time launch_time;

  // Last received feedback control
  RocketGimbalControl gimbal_control;

  // Last received rocket state
  RocketState rocket_state;

  // List of subscribers and publishers
  ros::Publisher fsm_pub;

  ros::Subscriber rocket_state_sub;
  ros::Subscriber sensor_sub;
  ros::Subscriber control_sub;
  ros::Subscriber command_sub;

  // Other parameters
  double rail_length = 0;

public:
  double frequency = 200;

  FsmNode()
  {
    ros::NodeHandle nh("~");

    // Initialize publishers and subscribers
    initTopics(nh);

    // Initialize fsm
    current_fsm = RocketFSMState::IDLE;

    // Overwrite rocket mass to stay in launch mode at first iteration
    rocket_state.propeller_mass = 10;

    nh.getParam("/environment/rail_length", rail_length);
  }

  void initTopics(ros::NodeHandle& nh)
  {
    // Create timer publisher and associated thread (100Hz)
    fsm_pub = nh.advertise<rocket_utils::FSM>("/gnc_fsm_pub", 10);

    // Subscribe to state message
    rocket_state_sub = nh.subscribe("/kalman_rocket_state", 1, &FsmNode::rocketStateCallback, this);

    // Subscribe to commanded control message
    control_sub = nh.subscribe("/gimbal_command_0", 1, &FsmNode::controlCallback, this);

    // Subscribe to commands
    command_sub = nh.subscribe("/commands", 10, &FsmNode::processCommand, this);
  }

  void rocketStateCallback(const rocket_utils::State::ConstPtr& rocket_state_msg)
  {
    rocket_state = fromROS(*rocket_state_msg);
  }

  // Callback function to store last received commanded control
  void controlCallback(const rocket_utils::GimbalControl::ConstPtr& gimbal_control_msg)
  {
    gimbal_control = fromROS(*gimbal_control_msg);
  }

  void processCommand(const std_msgs::String& command)
  {
    if (command.data == "stop" || command.data == "Stop")
    {
      current_fsm = RocketFSMState::STOP;
    }
    else if (current_fsm == RocketFSMState::IDLE)
    {
      // received launch command
      launch_time = ros::Time::now();
      current_fsm = RocketFSMState::LAUNCH;
    }
  }

  void updateFSM()
  {
    switch (current_fsm)
    {
      case IDLE: {
        // Do nothing
        break;
      }

      case RAIL: {
        // End of rail
        if (rocket_state.position.z > rail_length)
        {
          current_fsm = RocketFSMState::LAUNCH;
        }
        break;
      }

      case LAUNCH: {
        // End of burn -> no more thrust
        if (rocket_state.propeller_mass < 0 || gimbal_control.thrust == 0)
        {
          current_fsm = RocketFSMState::COAST;
        }
        break;
      }

      case COAST: {
        // Do nothing for now
        break;
      }

      case STOP: {
        // Do nothing
        break;
      }

      default:
        throw std::runtime_error("Invalid FSM");
    }

    // Publish the current state machine
    rocket_utils::FSM fsm_msg = toROS(current_fsm);
    fsm_msg.header.stamp = ros::Time::now();
    fsm_msg.launch_time = launch_time;
    fsm_pub.publish(fsm_msg);
  }
};

int main(int argc, char** argv)
{
  // Init ROS FSM node
  ros::init(argc, argv, "gnc_fsm");
  FsmNode fsm_node;

  ros::Rate loop_rate(fsm_node.frequency);

  while (ros::ok())
  {
    ros::spinOnce();
    fsm_node.updateFSM();

    loop_rate.sleep();
  }
}
