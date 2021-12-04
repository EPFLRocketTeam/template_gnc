/*
* Node to estimate the rocket full state (position, velocity, quaternion, angular rate and mass) 
* from the sensor data and commanded thrust and torque of the rocket engine
*
* Inputs: 
*   - Finite state machine from the basic_gnc package:	\gnc_fsm_pub
*   - 3D force and torque from the rocket engine:		\control_pub
*   - Sensor data (IMU and barometer):					\sensor_pub
*
* Parameters:
*   - Rocket model: 		/config/rocket_parameters.yaml
*   - Environment model: 	/config/environment_parameters.yaml
#	- Kalman matrix: 		class NavigationNode
*
* Outputs:
*   - Complete estimated state : \kalman_rocket_state
*
*/

#include "ros/ros.h"

#include "real_time_simulator/FSM.h"
#include "real_time_simulator/State.h"
#include "real_time_simulator/Control.h"
#include "real_time_simulator/Sensor.h"

#include "geometry_msgs/Vector3.h"

#include <time.h>
#include <sstream>
#include <string>

#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include <unsupported/Eigen/EulerAngles>

#include "rocket_model.hpp"

#include <type_traits>

#define DEG2RAD 0.01745329251


class NavigationNode {

	private:
		// Class with useful rocket parameters and methods
		Rocket rocket;

		// Last received fsm
		real_time_simulator::FSM rocket_fsm;

		// Last received control
		real_time_simulator::Control rocket_control;

		// Last received sensor data
		real_time_simulator::Sensor rocket_sensor;

		// Last received real state from simulator
		real_time_simulator::State rocket_state_simu;

		// List of subscribers and publishers
		ros::Publisher nav_pub;

		ros::Subscriber fsm_sub;
		ros::Subscriber control_sub;
		ros::Subscriber rocket_state_sub;
		ros::Subscriber sensor_sub;

		// Kalman matrix
		double Q, R, P;

		// Kalman state
		Eigen::Matrix<double, 14, 1> X;
	
	public:
		NavigationNode(ros::NodeHandle nh)
		{
			// Initialize publishers and subscribers
        	initTopics(nh);

			// Initialize fsm
			rocket_fsm.time_now = 0;
			rocket_fsm.state_machine = "Idle";

			// Initialize rocket class with useful parameters
			rocket.init(nh);

			//Get initial orientation and convert in Radians
			float roll = 0, zenith = 0, azimuth = 0.0;
			nh.getParam("/environment/rocket_roll", roll);
			nh.getParam("/environment/rail_zenith", zenith);
			nh.getParam("/environment/rail_azimuth", azimuth);

			roll *= DEG2RAD; zenith *= DEG2RAD; azimuth *= DEG2RAD;

			typedef Eigen::EulerSystem<-Eigen::EULER_Z, Eigen::EULER_Y, Eigen::EULER_Z> Rail_system;
			typedef Eigen::EulerAngles<double, Rail_system> angle_type;

			angle_type init_angle(azimuth, zenith, roll);

			Eigen::Quaterniond q(init_angle);

			// Init state X   
			X << 0, 0, 0,   0, 0, 0,     0.0, 0.0 , 0.0 , 1.0 ,     0.0, 0.0, 0.0,    rocket.propellant_mass;
			X.segment(6,4) = q.coeffs();

			// Initialize kalman parameters
			P = 0;
			Q = 1;
			R = 5;
		}

		void initTopics(ros::NodeHandle &nh) 
		{
			// Create filtered rocket state publisher
			nav_pub = nh.advertise<real_time_simulator::State>("kalman_rocket_state", 10);

			// Subscribe to time_keeper for fsm and time
			fsm_sub = nh.subscribe("gnc_fsm_pub", 100, &NavigationNode::fsmCallback, this);

			// Subscribe to control for kalman estimator
			control_sub = nh.subscribe("control_pub", 100, &NavigationNode::controlCallback, this);

			// Subscribe to state message from simulation
			rocket_state_sub = nh.subscribe("rocket_state", 100, &NavigationNode::rocket_stateCallback, this);

			// Subscribe to sensor for kalman correction
			sensor_sub = nh.subscribe("sensor_pub", 100, &NavigationNode::sensorCallback, this);
		}

		/* ------------ Callbacks functions ------------ */

		// Callback function to store last received fsm
		void fsmCallback(const real_time_simulator::FSM::ConstPtr& fsm)
		{
			rocket_fsm.time_now = fsm->time_now;
			rocket_fsm.state_machine = fsm->state_machine;
		}

		// Callback function to store last received control
		void controlCallback(const real_time_simulator::Control::ConstPtr& control)
		{
			rocket_control.torque = control->torque;
			rocket_control.force = control->force;
		}

		// Callback function to store last received state 
		// !! Only for simulation !!
		void rocket_stateCallback(const real_time_simulator::State::ConstPtr& rocket_state)
		{
			rocket_state_simu.pose = rocket_state->pose;
			rocket_state_simu.twist = rocket_state->twist;
			rocket_state_simu.propeller_mass = rocket_state->propeller_mass;
		}

		// Callback function to store last received sensor data
		void sensorCallback(const real_time_simulator::Sensor::ConstPtr& sensor)
		{
			rocket_sensor.IMU_acc = sensor->IMU_acc;
			rocket_sensor.IMU_gyro = sensor->IMU_gyro;
			rocket_sensor.baro_height = sensor->baro_height;

			update_step(rocket_sensor.baro_height);
		}

		/* ------------ User functions ------------ */
	
		void state_dynamics(Eigen::Ref<Eigen::Matrix<double, 14, 1>> x, Eigen::Ref<Eigen::Matrix<double, 14, 1>> xdot)
		{
			// -------------- Simulation variables -----------------------------
			double g0 = 9.81;  // Earth gravity in [m/s^2]

			Eigen::Matrix<double, 3, 1> rocket_control;
			rocket_control << rocket_control.force.x, rocket_control.force.y, rocket_control.force.z;

			// Orientation of the rocket with quaternion
			Eigen::Quaternion<double> attitude(x(9), x(6), x(7), x(8));
			attitude.normalize();
			Eigen::Matrix<double, 3, 3> rot_matrix = attitude.toRotationMatrix();

			// Current acceleration and angular rate from IMU
			Eigen::Matrix<double, 3, 1> IMU_acc; IMU_acc << rocket_sensor.IMU_acc.x, rocket_sensor.IMU_acc.y, rocket_sensor.IMU_acc.z;
			x.segment(10,3) << rocket_sensor.IMU_gyro.x, rocket_sensor.IMU_gyro.y, rocket_sensor.IMU_gyro.z;

			// Angular velocity omega in quaternion format to compute quaternion derivative
			Eigen::Quaternion<double> omega_quat(0.0, x(10), x(11), x(12));
			
			// -------------- Differential equation ---------------------

			// Position variation is speed
			xdot.head(3) = x.segment(3,3);

			// Speed variation is acceleration
			xdot.segment(3,3) =  rot_matrix*IMU_acc - Eigen::Vector3d::UnitZ()*g0;

			// Quaternion variation is 0.5*w◦q
			xdot.segment(6, 4) =  0.5*(omega_quat*attitude).coeffs();

			// Angular speed assumed to be constant between two measure
			xdot.segment(10, 3) << 0, 0, 0;

			// Mass variation is proportional to total thrust
			xdot(13) = -rocket_control.norm()/(rocket.Isp*g0);
		}

		void RK4(double dT)
		{
			Eigen::Matrix<double, 14, 1> k1, k2, k3, k4, X_inter;

			state_dynamics(X, k1); 			X_inter = X+k1*dT/2;
			state_dynamics(X_inter, k2); 	X_inter = X+k2*dT/2;
			state_dynamics(X_inter, k3); 	X_inter = X+k3*dT;
			state_dynamics(X_inter, k4);

			X = X + (k1+2*k2+2*k3+k4)*dT/6;	 
		}

		void predict_step()
		{ 
			static double last_predict_time = ros::Time::now().toSec();

			double dT = ros::Time::now().toSec() - last_predict_time;
			RK4(dT);
			last_predict_time = ros::Time::now().toSec();

			P += Q;
		}

		void update_step(double z_baro)
		{	
			double K = P/(P+R);

			X(2) = X(2) + K*(z_baro - X(2));
			P = (1.0-K)*P;
		}

		void updateNavigation()
		{
			// State machine ------------------------------------------
			if (rocket_fsm.state_machine.compare("Idle") == 0)
			{
				// Do nothing
			}

			else if (rocket_fsm.state_machine.compare("Launch") == 0 || rocket_fsm.state_machine.compare("Rail") == 0)
			{
				predict_step();
			}

			else if (rocket_fsm.state_machine.compare("Coast") == 0)
			{
				predict_step();
			}
			// Parse navigation state and publish it on the /nav_pub topic
			real_time_simulator::State rocket_state;

			rocket_state.pose.position.x = X(0);
			rocket_state.pose.position.y = X(1);
			rocket_state.pose.position.z = X(2);

			rocket_state.twist.linear.x = X(3);
			rocket_state.twist.linear.y = X(4);
			rocket_state.twist.linear.z = X(5);

			rocket_state.pose.orientation.x = X(6);
			rocket_state.pose.orientation.y = X(7);
			rocket_state.pose.orientation.z = X(8);
			rocket_state.pose.orientation.w = X(9);

			rocket_state.twist.angular.x = X(10);
			rocket_state.twist.angular.y = X(11);
			rocket_state.twist.angular.z = X(12);

			rocket_state.propeller_mass = X(13);

			nav_pub.publish(rocket_state);

		}
};


int main(int argc, char **argv)
{
	// Init ROS time keeper node
	ros::init(argc, argv, "data_fusion");
	ros::NodeHandle nh;

	NavigationNode navigationNode(nh);

	// Thread to compute navigation state. Duration defines interval time in seconds
	ros::Timer control_thread = nh.createTimer(ros::Duration(0.010),
	[&](const ros::TimerEvent&) 
	{
		navigationNode.updateNavigation();	
	});

	// Automatic callback of service and publisher from here
	ros::spin();
}
