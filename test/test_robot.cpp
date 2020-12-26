#include <memory>
#include <chrono>
#include <unistd.h>
#include <iostream>
#include <fstream>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <boost/asio.hpp>
#include <boost/asio/signal_set.hpp>

#include <nlohmann/json.hpp>

#include <Json2Eigen.hpp>

#include "bno055_interface.h"
#include "motor.h"


struct State {
  double x;
  double theta;
  double x_dot;
  double theta_dot;
  double omega;
};

struct Parameters {
  double m_c = 0.390;
  double m_p = 0.614;
  double L = 0.154;
  double g = 9.81;

  double wheel_r = 0.035f;
  double wheel_base = 0.183;
};

class Controller {

 public:

  Parameters m_params;

  double m_f_x = 0.0;
  double m_f_x_dot = 0.0;
  double m_f_theta = 0.0;
  double m_f_theta_dot = 0.0;
  double m_f_omega = 0.0;
  double m_f = 0.0;

  double m_x = 0.0;

  std::vector<double> Step(State state) {

    if (m_x == 0.0) {
      m_x = state.x;
    }

    double k_p = 20.0;
    double k_d = 2.0;

    m_f_theta = state.theta * k_p;
    m_f_theta_dot = state.theta_dot * k_d ;

    m_f_x = 1.0 * (state.x - m_x);
    m_f_x_dot = 1.0 * state.x_dot;

    m_f_omega = 0.05 * state.omega / 2.0;

    m_f = m_f_theta + m_f_theta_dot + m_f_x_dot + m_f_x;

    double t = m_f * m_params.wheel_r / 2.0f;

    double t_l = t + m_f_omega;
    double t_r = t - m_f_omega;

//    t = t*0.0;

//    std::cout << "f_theta: " << f_theta << " f_x_dot: " << f_x_dot << " f: " << f << "\n";

    return {t_l, t_r};
  }
};


class Vehicle : public boost::asio::io_service {
 private:

  State m_state;
  Controller m_ctrl;

  BNO055 m_bno_interface;

  struct sockaddr_can m_can_address;
  struct ifreq ifr;
  int m_native_socket;
  boost::asio::posix::stream_descriptor m_stream;
  boost::asio::signal_set m_signals;
  boost::asio::deadline_timer m_d_timer;

  boost::asio::io_service::strand m_file_io_strand;

  std::vector<Motor> m_motors = {Motor(1, false), Motor(2, true)};

  canfd_frame m_receive_frame;

  void SetSigintHandler() {
    m_signals.async_wait([this](const boost::system::error_code &error, int signal_number) {
      std::cout << "Exiting ..." << std::endl;
      this->stop();
    });

  }

  void SetStopAll() {
    for (auto &m : m_motors) {
      auto[ctrl_buffer, ctrl_handler] = m.BuildStopBuffer();
      m_stream.async_write_some(ctrl_buffer, ctrl_handler);
      m_stream.async_write_some(ctrl_buffer, ctrl_handler);
      m_stream.async_write_some(ctrl_buffer, ctrl_handler);
      m_stream.async_write_some(ctrl_buffer, ctrl_handler);
    }
  }

  void ScheduleBnoReceiveTimed() {
    m_d_timer.expires_from_now(boost::posix_time::milliseconds(4));
    m_d_timer.async_wait([this](auto ...vn) { this->BnoReceive(); });
  }

  void BnoReceive() {

    auto t0 = std::chrono::steady_clock::now();
    m_bno_interface.Update();
    auto t1 = std::chrono::steady_clock::now();
//    std::cout << "bno update: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "\n";
    if (std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() > 15) {
      exit(0);
    }


    // Update state

//    auto velocity_l = 2 * M_PI * 0.035 * (m_motors[0].m_state.velocity - 2 * M_PI * m_bno_interface.m_state.theta_dot);
//    auto velocity_r = 2 * M_PI * 0.035 * (m_motors[1].m_state.velocity - 2 * M_PI * m_bno_interface.m_state.theta_dot);

    auto velocity_l = 2 * M_PI * 0.035 * m_motors[0].m_state.velocity;
    auto velocity_r = 2 * M_PI * 0.035 * m_motors[1].m_state.velocity;

    double wheel_base = 0.183;

    m_state.x = (m_motors[0].m_state.position + m_motors[1].m_state.position)/2.0;
    m_state.theta = m_bno_interface.m_state.theta;
    m_state.x_dot = (velocity_r + velocity_l) / 2.0;
    m_state.theta_dot = m_bno_interface.m_state.theta_dot;
    m_state.omega = (velocity_r - velocity_l) / wheel_base;

    auto torque = m_ctrl.Step(m_state);

    for (int n = 0; n < m_motors.size(); n++) {
      auto[ctrl_buffer, ctrl_handler] = m_motors[n].BuildTorqueBuffer(torque[n]);
      m_stream.async_write_some(ctrl_buffer, ctrl_handler);

      auto[query_buffer, query_handler] = m_motors[n].BuildQueryBuffer();
      m_stream.async_write_some(query_buffer, query_handler);
    }

    ScheduleBnoReceiveTimed();

    // Log some data ...

//    std::cout << "mode0: " << int(m_motors[0].m_state.mode) << " mode1: " << int(m_motors[1].m_state.mode) << std::endl;

    nlohmann::json json;

    json["timestamp"] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

    json["m_f_theta"] = m_ctrl.m_f_theta;
    json["m_f_theta_dot"] = m_ctrl.m_f_theta_dot;
    json["m_f_x"] = m_ctrl.m_f_x;
    json["m_f_x_dot"] = m_ctrl.m_f_x_dot;
    json["m_f_omega"] = m_ctrl.m_f_omega;
    json["m_f"] = m_ctrl.m_f;

    json["x"] = m_state.x;
    json["theta"] = m_bno_interface.m_state.theta;
    json["theta_dot"] = m_bno_interface.m_state.theta_dot;
    json["omega"] = m_bno_interface.m_state.omega;

    json["raw_theta"] = m_bno_interface.m_state_raw.theta;
    json["raw_theta_dot"] = m_bno_interface.m_state_raw.theta_dot;
    json["raw_omega"] = m_bno_interface.m_state_raw.omega;

    json["torque0"] = torque[0];
    json["torque1"] = torque[1];

    json["omega"] = m_state.omega;
    json["x_dot"] = m_state.x_dot;

    json["velocity_l"] = velocity_l;
    json["velocity_r"] = velocity_r;

    json["position0"] = m_motors[0].m_state.position;
    json["position1"] = m_motors[1].m_state.position;

    json["mode0"] = m_motors[0].m_state.mode;
    json["mode1"] = m_motors[1].m_state.mode;

    post(m_file_io_strand.wrap([json](){
      const auto msgpack = nlohmann::json::to_msgpack(json);
      std::ofstream("dump.msg", std::ios::app | std::ios::binary).write(
          reinterpret_cast<const char *>(msgpack.data()), msgpack.size() * sizeof(uint8_t));
    }));

  }

  void ScheduleCanReceive() {
    m_stream.async_read_some(
        boost::asio::buffer(&m_receive_frame, sizeof(m_receive_frame)),
        [this](auto ...vn) { this->OnCanReceive(vn...); });
  }

  void OnCanReceive(const boost::system::error_code &error, size_t bytes_transferred) {
    // TODO, OLSLO, perform lookup in a map instead.
    for (auto &m : m_motors) { m.MaybeReceive(m_receive_frame); }
    ScheduleCanReceive();
  }

  void SetupCan() {
    m_native_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    // Enable reception of CAN FD frames
    {
      int enable = 1;

      auto rc = ::setsockopt(
          m_native_socket,
          SOL_CAN_RAW,
          CAN_RAW_FD_FRAMES,
          &enable,
          sizeof(enable)
      );
      if (-1 == rc) {
        std::perror("setsockopt CAN FD");
        std::exit(1);
      }
    }

    strcpy(ifr.ifr_name, "can0");
    ioctl(m_native_socket, SIOCGIFINDEX, &ifr);

    m_can_address.can_family = AF_CAN;
    m_can_address.can_ifindex = ifr.ifr_ifindex;
    if (bind(m_native_socket, (struct sockaddr *) &m_can_address, sizeof(m_can_address)) < 0) {
      perror("Error in socket bind");
      exit(1);
    }

    m_stream.assign(m_native_socket);

  }

 public:
  Vehicle()
      : m_stream(*this), m_signals(*this, SIGINT), m_d_timer(*this, boost::posix_time::seconds(50)),
        m_file_io_strand(*this) {
    auto now = std::chrono::steady_clock::now();
    SetupCan();
    SetSigintHandler();
    ScheduleCanReceive();
    SetStopAll();
    ScheduleBnoReceiveTimed();
  }

};

int main(int argc, char *argv[]) {

  Vehicle veh;
  veh.run();

  return 0;

}