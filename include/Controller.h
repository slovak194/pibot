//
// Created by slovak on 12/30/20.
//

#ifndef PIBOT_INCLUDE_CONTROLLER_H_
#define PIBOT_INCLUDE_CONTROLLER_H_

#include <iostream>
#include <memory>

#include <nlohmann/json.hpp>

//#include "Config.h"

#include "../submoudles/remote-config/include/remote_config/Server.h"
#include "Joystick.h"
#include "Motor.h"
#include "Imu.h"



template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}


struct State {

  std::shared_ptr<remote_config::Server> m_conf;

  double x = 0.0;
  double theta = 0.0;
  double x_dot = 0.0;
  double theta_dot = 0.0;
  double omega = 0.0;
  bool valid = false;

  State(std::shared_ptr<remote_config::Server> conf)
  : m_conf(conf) {}

  nlohmann::json Update(std::vector<Motor> &motors, Imu &imu) {

    auto wheel_base = m_conf->get<double>("/model/wheel_base");
    auto wheel_radius = m_conf->get<double>("/model/wheel_radius");

    auto velocity_l = 2.0 * M_PI * wheel_radius * (motors[0].m_state.velocity + imu.m_state.theta_dot/(2.0 * M_PI));
    auto velocity_r = 2.0 * M_PI * wheel_radius * (motors[1].m_state.velocity + imu.m_state.theta_dot/(2.0 * M_PI));


    x = (motors[0].m_state.position + motors[1].m_state.position)/2.0;
    theta = imu.m_state.theta;
    x_dot = (velocity_r + velocity_l) / 2.0;
    theta_dot = imu.m_state.theta_dot;
    omega = (velocity_r - velocity_l) / wheel_base;

    valid = imu.m_state.valid;

    nlohmann::json state_dbg;

    state_dbg["x"] = x;
    state_dbg["theta"] = theta;
    state_dbg["x_dot"] = x_dot;
    state_dbg["theta_dot"] = theta_dot;
    state_dbg["omega"] = omega;
    state_dbg["valid"] = valid;

//    state_dbg["acc_x"] = imu.m_acc.x;
//    state_dbg["acc_y"] = imu.m_acc.y;
//    state_dbg["acc_z"] = imu.m_acc.z;

    return state_dbg;

  }

};


class Controller {

 public:

  std::shared_ptr<remote_config::Server> m_conf;

  double m_f_x = 0.0;
  double m_f_x_dot = 0.0;
  double m_f_theta = 0.0;
  double m_f_theta_dot = 0.0;
  double m_f_omega = 0.0;
  double m_f_ff = 0.0;
  double m_f = 0.0;

  double m_x = 0.0;

  double m_x_dot_ref = 0.0;

  Controller(std::shared_ptr<remote_config::Server> conf)
      : m_conf(conf) {

  }

  std::pair<std::vector<double>, nlohmann::json> Step(State state, Joystick& joy, double dt_ms);

};

#endif //PIBOT_INCLUDE_CONTROLLER_H_
