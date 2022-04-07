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

#include <ros/package.h>

#include "real_time_simulator/FSM.h"
#include "real_time_simulator/State.h"
#include "real_time_simulator/Waypoint.h"
#include "real_time_simulator/Trajectory.h"
#include "real_time_simulator/Gimbal.h"

#include "geometry_msgs/Vector3.h"

#include "real_time_simulator/GetFSM.h"
#include "real_time_simulator/GetWaypoint.h"

#include <time.h>
#include <sstream>
#include <string>

#include <iomanip>
#include <iostream>
#include <chrono>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "rocket_model.hpp"

class ControlNode {
  private:
      // Class with useful rocket parameters and methods
      Rocket rocket;

      // Last received rocket state
      real_time_simulator::State rocket_state;

      // Last requested fsm
      real_time_simulator::FSM rocket_fsm;

      // List of subscribers and publishers
      ros::Publisher control_pub;

      ros::Subscriber rocket_state_sub;
      ros::Subscriber fsm_sub;

      ros::ServiceClient client_fsm;
      real_time_simulator::GetFSM srv_fsm;

    public:

      ControlNode(ros::NodeHandle &nh)
      {
        // Initialize publishers and subscribers
        initTopics(nh);

        // Initialize fsm
        rocket_fsm.time_now = 0;
        rocket_fsm.state_machine = "Idle";

        // Initialize rocket class with useful parameters
        rocket.init(nh);
      }

      void initTopics(ros::NodeHandle &nh) 
      {
        // Create control publisher
        control_pub = nh.advertise<real_time_simulator::Gimbal>("gimbal_command_0", 10);

        // Subscribe to state message from basic_gnc
        rocket_state_sub = nh.subscribe("kalman_rocket_state", 1, &ControlNode::rocket_stateCallback, this);

        // Subscribe to fsm and time from time_keeper
        fsm_sub = nh.subscribe("gnc_fsm_pub", 1, &ControlNode::fsm_Callback, this);

        // Setup Time_keeper client and srv variable for FSM and time synchronization
        client_fsm = nh.serviceClient<real_time_simulator::GetFSM>("getFSM_gnc");

      }

      // Callback function to store last received state
      void rocket_stateCallback(const real_time_simulator::State::ConstPtr& new_rocket_state)
      {
        rocket_state.pose = new_rocket_state->pose;
        rocket_state.twist = new_rocket_state->twist;
        rocket_state.propeller_mass = new_rocket_state->propeller_mass;
      }

      void fsm_Callback(const real_time_simulator::FSM::ConstPtr& fsm)
      {
        rocket_fsm.state_machine = fsm->state_machine;
        rocket_fsm.time_now = fsm->time_now;
      }

      real_time_simulator::Gimbal basic_control()
      {
        // Init control message
        real_time_simulator::Gimbal control_law;

        control_law.thrust = rocket.get_full_thrust(rocket_fsm.time_now);
        control_law.outer_angle = 2e-2*rocket_state.twist.angular.x;
        control_law.inner_angle = 2e-2*rocket_state.twist.angular.y;

        return control_law;
      }

      real_time_simulator::Gimbal PD_attitude_control()
      {
        // Init control message
        real_time_simulator::Gimbal control_law;

        Eigen::Quaternion<double> attitude(rocket_state.pose.orientation.w, rocket_state.pose.orientation.x, rocket_state.pose.orientation.y, rocket_state.pose.orientation.z);
        Eigen::Matrix<double, 3, 3> rot_matrix = attitude.toRotationMatrix();
        
        // Convert angular rate in body frame
        Eigen::Matrix<double, 3, 1> angular_rate{rocket_state.twist.angular.x, rocket_state.twist.angular.y, rocket_state.twist.angular.z};
        angular_rate = rot_matrix.transpose()*angular_rate;

        // Basic PD controller in orientation
        // Outer gimbal rotates around rocket X axis
        // Inner gimbal rotates around rotated Y axis
        control_law.outer_angle = 1e-1*rocket_state.pose.orientation.x + 7e-1*angular_rate(0);
        control_law.inner_angle = 1e-1*rocket_state.pose.orientation.y + 7e-1*angular_rate(1);

        // Basic PD controller in altitude to reach 20 meters
        control_law.thrust = 10*(20-rocket_state.pose.position.z) + 9.81*(rocket_state.propeller_mass+rocket.dry_mass) - 20*rocket_state.twist.linear.z;

        return control_law;
      }


      void updateControl()
      {
        // Init default control to zero
        real_time_simulator::Gimbal control_law;

        //Get current FSM and time
        if(client_fsm.call(srv_fsm))
        {
          rocket_fsm = srv_fsm.response.fsm;
        }
      
        // ----------------- State machine -----------------
        if (rocket_fsm.state_machine.compare("Idle") == 0)
        {
          // Do nothing
        }

        else 
        {
          if (rocket_fsm.state_machine.compare("Rail") == 0)
          {
            control_law = PD_attitude_control();
          }

          else if (rocket_fsm.state_machine.compare("Launch") == 0)
          {
            control_law = PD_attitude_control();
          }

          else if (rocket_fsm.state_machine.compare("Coast") == 0)
          {

          }
        
          control_pub.publish(control_law);
        }
      }

      

};






int main(int argc, char **argv)
{
	// Init ROS control node
  ros::init(argc, argv, "control");
  ros::NodeHandle nh;

  ControlNode controlNode(nh);
	
  // Thread to compute control. Duration defines interval time in seconds
  ros::Timer control_thread = nh.createTimer(ros::Duration(0.05), [&](const ros::TimerEvent&) 
	{
    controlNode.updateControl();
    
  });

	// Automatic callback of service and publisher from here
	ros::spin();

}

