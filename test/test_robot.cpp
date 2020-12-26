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


class Vehicle : public boost::asio::io_service {
 private:

  BNO055 m_bno_interface;

  struct sockaddr_can m_can_address;
  struct ifreq ifr;
  int m_native_socket;
  boost::asio::posix::stream_descriptor m_stream;
  boost::asio::signal_set m_signals;
  boost::asio::deadline_timer m_d_timer;

  boost::asio::io_service::strand m_file_io_strand;

  std::vector<Motor> m_motors = {Motor(1), Motor(2)};

  canfd_frame m_receive_frame;

  static std::vector<float> GetTorque(double pitch, double dpitch) {

    double k_p = 5.0;
    double k_d = 1.0;

    auto f = static_cast<float>(pitch * k_p + dpitch * k_d);

    float wheel_r = 0.07f;

    float t = f * wheel_r / 2.0f;

    return {t, -t};
  }

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

    auto torque = GetTorque(m_bno_interface.m_state.theta, m_bno_interface.m_state.theta_dot);

    for (int n = 0; n < m_motors.size(); n++) {
      auto[ctrl_buffer, ctrl_handler] = m_motors[n].BuildTorqueBuffer(torque[n]);
      m_stream.async_write_some(ctrl_buffer, ctrl_handler);

      auto[query_buffer, query_handler] = m_motors[n].BuildQueryBuffer();
      m_stream.async_write_some(query_buffer, query_handler);
    }

    ScheduleBnoReceiveTimed();

    // Log some data ...

    nlohmann::json json;

    json["timestamp"] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

    json["theta"] = m_bno_interface.m_state.theta;
    json["theta_dot"] = m_bno_interface.m_state.theta_dot;
    json["omega"] = m_bno_interface.m_state.omega;

    json["raw_theta"] = m_bno_interface.m_state_raw.theta;
    json["raw_theta_dot"] = m_bno_interface.m_state_raw.theta_dot;
    json["raw_omega"] = m_bno_interface.m_state_raw.omega;

    json["torque0"] = torque[0];
    json["torque1"] = torque[1];

    auto velocity_l = 2 * M_PI * 0.07 * m_motors[0].m_state.velocity;
    auto velocity_r = 2 * M_PI * 0.07 * m_motors[1].m_state.velocity * -1;

    double wheel_base = 0.183;

    auto omega = (velocity_r - velocity_l) / wheel_base;

    json["omega"] = omega;
    json["x_dot"] = (velocity_r - velocity_l) / 2.0;

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