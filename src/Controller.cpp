//
// Created by slovak on 12/30/20.
//

#include "Controller.h"

std::pair<std::vector<double>, nlohmann::json> Controller::Step(State state, Joystick& joy, double dt_ms) {

  if (m_x == 0.0 || std::abs(joy.m_axes[1]) > 0.1 || std::abs(state.x_dot) > 0.5) {
    m_x = state.x;
  }



  auto theta_k = m_conf->get<double>("/ctrl/theta_k");
  auto theta_dot_k = m_conf->get<double>("/ctrl/theta_dot_k");
  auto x_k = m_conf->get<double>("/ctrl/x_k");
  auto x_dot_k = m_conf->get<double>("/ctrl/x_dot_k");
  auto omega_k = m_conf->get<double>("/ctrl/omega_k");
  auto x_dot_abs_max = m_conf->get<double>("/ctrl/x_dot_abs_max");
  auto x_ddot_max = m_conf->get<double>("/ctrl/x_ddot_max");

  auto k_j_omega = m_conf->get<double>("/joy/k_j_omega");
  auto k_j_x_dot = m_conf->get<double>("/joy/k_j_x_dot");

  auto wheel_radius = m_conf->get<double>("/model/wheel_radius");

  auto m_p = m_conf->get<double>("/model/m_p");
  auto m_c = m_conf->get<double>("/model/m_c");
  auto g = m_conf->get<double>("/model/g");

  double theta = state.theta;
  double theta_dot = state.theta_dot;
  double x = state.x;
  double x_dot = state.x_dot;
  double omega = state.omega;

  double x_dot_ref = k_j_x_dot*joy.m_axes[1];
//

  double x_ddot_ref = (x_dot_ref - m_x_dot_ref)/(dt_ms*1e-3);

  if (std::abs(x_ddot_ref) < x_ddot_max) {
    m_x_dot_ref = x_dot_ref;
  } else {
    m_x_dot_ref = m_x_dot_ref + sgn(x_ddot_ref) * x_ddot_max * dt_ms*1e-3;
  }

  x_dot_ref = m_x_dot_ref;


//  double omega_ref = k_j_omega*joy.m_axes[0] * sgn(x_dot_ref);
    double omega_ref = k_j_omega*joy.m_axes[3];

//  m_f_x = x_k * (x - m_x);

  double theta_ref = m_conf->get<double>("/ctrl/theta_theta_dot_k") * (x_dot_ref - x_dot) - x_k * (x - m_x);

  m_f_theta = theta_k * (theta_ref - theta);
  m_f_theta_dot = theta_dot * theta_dot_k;

  //    axis:  3, value: ROT RIGHT + ROT LEFT -

  m_f_omega = omega_k * (omega_ref - omega) / 2.0;

  m_f_ff = g*(m_c + m_p)*std::tan(theta);

  m_f = m_f_theta + m_f_theta_dot + m_f_x_dot + m_f_ff;

  double t = m_f * wheel_radius / 2.0f;

  double t_l = t + m_f_omega;
  double t_r = t - m_f_omega;

  if (!state.valid) {
    t_l = 0.0;
    t_r = 0.0;
  }

//  t_l = 0.0; // TODO, OLSLO, remove this
//  t_r = 0.0; // TODO, OLSLO, remove this

  if (std::abs(x_dot) > x_dot_abs_max) {
    std::cout << "Too fast, exiting ..." << std::endl;
    exit(1);
  }

  if (joy.m_buttons[6]) {
    std::cout << "Stop requested, exiting ..." << std::endl;
    exit(0);
  }

  nlohmann::json ctrl_debug;

  ctrl_debug["theta_ref"] = theta_ref;
  ctrl_debug["m_f_theta"] = m_f_theta;
  ctrl_debug["m_f_theta_dot"] = m_f_theta_dot;
  ctrl_debug["m_f_x"] = m_f_x;
  ctrl_debug["x_m_x"] = x - m_x;
  ctrl_debug["x_dot_ref"] = x_dot_ref;
  ctrl_debug["m_f_x_dot"] = m_f_x_dot;
  ctrl_debug["m_f_omega"] = m_f_omega;
  ctrl_debug["m_f_ff"] = m_f_ff;
  ctrl_debug["m_f"] = m_f;

  ctrl_debug["t_l"] = t_l;
  ctrl_debug["t_r"] = t_r;

  return {{t_l, t_r}, ctrl_debug};
}
