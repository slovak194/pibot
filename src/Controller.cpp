//
// Created by slovak on 12/30/20.
//

#include "Controller.h"

std::pair<std::vector<double>, nlohmann::json> Controller::Step(State state, Joystick& joy) {

  if (m_x == 0.0 || std::abs(joy.m_axes[1]) > 0.05) {
    m_x = state.x;
  }

  double theta_k = (*m_conf)["ctrl"]["theta_k"];
  double theta_dot_k = (*m_conf)["ctrl"]["theta_dot_k"];
  double x_k = (*m_conf)["ctrl"]["x_k"];
  double x_dot_k = (*m_conf)["ctrl"]["x_dot_k"];
  double omega_k = (*m_conf)["ctrl"]["omega_k"];
  double x_dot_abs_max = (*m_conf)["ctrl"]["x_dot_abs_max"];

  double k_j_omega = (*m_conf)["joy"]["k_j_omega"];
  double k_j_x_dot = (*m_conf)["joy"]["k_j_x_dot"];

  double wheel_radius = (*m_conf)["model"]["wheel_radius"];

  double m_p = (*m_conf)["model"]["m_p"];
  double m_c = (*m_conf)["model"]["m_c"];
  double g = (*m_conf)["model"]["g"];

  double theta = state.theta;
  double theta_dot = state.theta_dot;
  double x = state.x;
  double x_dot = state.x_dot;
  double omega = state.omega;

  double x_dot_ref = k_j_x_dot*joy.m_axes[1];
//  double omega_ref = k_j_omega*joy.m_axes[0] * sgn(x_dot_ref);
    double omega_ref = k_j_omega*joy.m_axes[3];

  m_f_theta = theta * theta_k;
  m_f_theta_dot = theta_dot * theta_dot_k;

  m_f_x = x_k * (x - m_x);
  m_f_x_dot = x_dot_k * (x_dot_ref - x_dot);

  //    axis:  3, value: ROT RIGHT + ROT LEFT -

  m_f_omega = omega_k * (omega_ref - omega) / 2.0;

  m_f_ff = g*(m_c + m_p)*std::tan(theta);

//    std::cout << joy.m_axes[3] << "\t" << state.omega << "\n";

  m_f = m_f_theta + m_f_theta_dot + m_f_x_dot + m_f_x + m_f_ff;

  double t = m_f * wheel_radius / 2.0f;

  double t_l = t + m_f_omega;
  double t_r = t - m_f_omega;

  if (!state.valid) {
    t_l = 0.0;
    t_r = 0.0;
  }

  if (std::abs(x_dot) > x_dot_abs_max) {
    std::cout << "Too fast, exiting ..." << std::endl;
    exit(1);
  }

  if (joy.m_buttons[6]) {
    std::cout << "Stop requested, exiting ..." << std::endl;
    exit(0);
  }

  nlohmann::json ctrl_debug;

  ctrl_debug["m_f_theta"] = m_f_theta;
  ctrl_debug["m_f_theta_dot"] = m_f_theta_dot;
  ctrl_debug["m_f_x"] = m_f_x;
  ctrl_debug["m_f_x_dot"] = m_f_x_dot;
  ctrl_debug["m_f_omega"] = m_f_omega;
  ctrl_debug["m_f_ff"] = m_f_ff;
  ctrl_debug["m_f"] = m_f;

  ctrl_debug["t_l"] = t_l;
  ctrl_debug["t_r"] = t_r;

//    std::cout << "f_theta: " << f_theta << " f_x_dot: " << f_x_dot << " f: " << f << "\n";

  return {{t_l, t_r}, ctrl_debug};
}
