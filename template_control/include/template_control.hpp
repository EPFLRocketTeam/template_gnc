#pragma once

#include "rocket_types/rocket_types.h"
#include "rocket_model.hpp"
#include "rocket_types/eigen_conversions.h"
#include <memory>

using namespace rocket;

class TemplateController
{
public:
  struct Config
  {
    double kp1 = 2e-2;
    double kp2 = 1e-1;
    double kd2 = 7e-1;
    double kp_roll = 5e0;
  };

  // Constructor using default config
  TemplateController(RocketProps rocket_props) : TemplateController(Config{}, rocket_props)
  {
  }

  TemplateController(Config config_, RocketProps& rocket_props) : config(config_), rocket_model(rocket_props)
  {
  }

  RocketGimbalControl computeBasicControl(const RocketState& rocket_state, double time_now)
  {
    Eigen::Quaterniond attitude = toEigenQuaternion(rocket_state.orientation);
    Eigen::Matrix3d rot_matrix = attitude.toRotationMatrix();
    // Angular rate in body frame
    Eigen::Vector3d angular_rate = rot_matrix.transpose() * toEigen(rocket_state.angular_velocity);

    RocketGimbalControl gimbal_control;
    gimbal_control.thrust = rocket_model.getFullThrust(time_now);
    gimbal_control.outer_angle = config.kp1 * angular_rate(0);
    gimbal_control.inner_angle = config.kp1 * angular_rate(1);

    return gimbal_control;
  }

  RocketGimbalControl computePDAttitudeControl(const RocketState& rocket_state, double time_now)
  {
    Eigen::Quaterniond attitude = toEigenQuaternion(rocket_state.orientation);
    Eigen::Matrix3d rot_matrix = attitude.toRotationMatrix();
    // Angular rate in body frame
    Eigen::Vector3d angular_rate = rot_matrix.transpose() * toEigen(rocket_state.angular_velocity);

    RocketGimbalControl gimbal_control;
    gimbal_control.thrust = rocket_model.getFullThrust(time_now);
    gimbal_control.outer_angle = config.kp2 * rocket_state.orientation.x + config.kd2 * angular_rate(0);
    gimbal_control.inner_angle = config.kp2 * rocket_state.orientation.y + config.kd2 * angular_rate(1);

    return gimbal_control;
  }

  RocketControlMomentGyro computeRollControl(const RocketState& rocket_state)
  {
    Eigen::Quaterniond attitude = toEigenQuaternion(rocket_state.orientation);
    Eigen::Matrix3d rot_matrix = attitude.toRotationMatrix();
    // Angular rate in body frame
    Eigen::Vector3d angular_rate = rot_matrix.transpose() * toEigen(rocket_state.angular_velocity);

    RocketControlMomentGyro gmc_control;
    gmc_control.outer_angle = 0;
    gmc_control.inner_angle = 0;
    gmc_control.torque = config.kp_roll * angular_rate(2);

    return gmc_control;
  }

private:
  Config config;
  RocketModel rocket_model;
};