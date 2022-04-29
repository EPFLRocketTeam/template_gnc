/*
 * Node to send control commands to the rocket engine.
 * Can also be used by the simulation for SIL and PIL tests.
 *
 * Inputs:
 *   - Finite state machine from the template_fsm :	    /gnc_fsm_pub
 *   - Estimated state from template_navigation:		      /kalman_rocket_state
 *
 * Important parameters:
 *   - Rocket model: 		  /config/rocket_parameters.yaml
 *   - Environment model: 	/config/environment_parameters.yaml
 *	  - P gain: 		        P_control()
 *   - Control loop period control_thread()
 *
 * Outputs:
 *   - Commanded angles and thrust for the rocket gimbal:  /gimbal_command_0
 *
 */

#include "ros/ros.h"

#include "template_control.hpp"

#include <memory>
#include "rocket_types/ros_conversions.h"
#include "ros_config_loader.h"

class RocketControlNode
{
public:
  double frequency = 20;

  RocketControlNode() : rocket_fsm(RocketFSMState::IDLE)
  {
    ros::NodeHandle nh("~");

    // Load the rocket properties from the ROS parameter server
    RocketProps rocket_props = loadRocketProps(nh);

    // Instantiate the controller
    controller = std::unique_ptr<TemplateController>(new TemplateController(rocket_props));

    // Initialize publishers and subscribers
    initTopics(nh);
  }

  void initTopics(ros::NodeHandle& nh)
  {
    // Create control publishers
    gimbal_command_pub = nh.advertise<rocket_utils::GimbalControl>("/gimbal_command_0", 10);
    gmc_command_pub = nh.advertise<rocket_utils::ControlMomentGyro>("/cmg_command_0", 10);

    // Subscribe to state message from basic_gnc
    rocket_state_sub = nh.subscribe("/kalman_rocket_state", 1, &RocketControlNode::rocketStateCallback, this);

    // Subscribe to fsm time_keeper
    fsm_sub = nh.subscribe("/gnc_fsm_pub", 1, &RocketControlNode::fsmCallback, this);
  }

  void run()
  {
    ros::Time time_now = ros::Time::now();

    switch (rocket_fsm)
    {
      case IDLE: {
        // Do nothing
        break;
      }

      // Compute attitude control and send control message
      // in both RAIL and LAUNCH mode
      case RAIL:
      case LAUNCH: {
        // Compute control
        RocketGimbalControl gimbal_control =
            controller->computeBasicControl(rocket_state, (time_now - launch_time).toSec());

        // Convert to ROS message and publish
        rocket_utils::GimbalControl gimbal_control_msg = toROS(gimbal_control);
        gimbal_control_msg.header.stamp = time_now;
        gimbal_command_pub.publish(gimbal_control_msg);
        break;
      }

      case COAST:
      case STOP: {
        // Do nothing
        break;
      }

      default:
        throw std::runtime_error("Unhandled FSM");
    }

    switch (rocket_fsm)
    {
      case IDLE:
      case RAIL: {
        // Do nothing
        break;
      }

      // Compute roll control and send control message
      // in both LAUNCH and COAST mode
      case LAUNCH:
      case COAST: {
        // Compute roll control
        RocketControlMomentGyro gmc_control = controller->computeRollControl(rocket_state);

        // Convert to ROS message and publish
        rocket_utils::ControlMomentGyro gmc_control_msg = toROS(gmc_control);
        gmc_control_msg.header.stamp = time_now;
        gmc_command_pub.publish(gmc_control_msg);
        break;
      }

      case STOP: {
        // Do nothing
        break;
      }

      default:
        throw std::runtime_error("Unhandled FSM");
    }
  }

private:
  std::unique_ptr<TemplateController> controller;

  // Last received rocket state
  RocketState rocket_state{};

  // Last requested fsm
  RocketFSMState rocket_fsm;

  // List of subscribers and publishers
  ros::Publisher gimbal_command_pub;
  ros::Publisher gmc_command_pub;

  ros::Subscriber rocket_state_sub;
  ros::Subscriber fsm_sub;

  ros::Time launch_time;

  /* ------------ Callbacks functions ------------ */

  // Callback function to store last received fsm
  void fsmCallback(const rocket_utils::FSM::ConstPtr& fsm)
  {
    rocket_fsm = fromROS(*fsm);
    launch_time = fsm->launch_time;
  }
  // Callback function to store last received state
  void rocketStateCallback(const rocket_utils::State::ConstPtr& rocket_state_msg)
  {
    rocket_state = fromROS(*rocket_state_msg);
  }
};

int main(int argc, char** argv)
{
  // Init ROS control node
  ros::init(argc, argv, "control");

  RocketControlNode control_node;

  ros::Rate loop_rate(control_node.frequency);

  while (ros::ok())
  {
    ros::spinOnce();
    control_node.run();

    loop_rate.sleep();
  }
}