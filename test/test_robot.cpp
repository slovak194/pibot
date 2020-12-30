#include <memory>
#include <cstdio>
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

#include "Imu.h"
#include "Motor.h"
#include "Config.h"

#include "Joystick.h"
#include "Controller.h"

class Vehicle : public boost::asio::io_service {
 private:

  State m_state;
  Controller m_ctrl;

  Imu m_imu;
  Joystick m_joy;

  std::shared_ptr<Config> m_conf;

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
    m_imu.Update();
    auto t1 = std::chrono::steady_clock::now();
//    std::cout << "bno update: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "\n";
    if (std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() > 15) {
      std::cout << "loop time > 15ms, exiting ... " << std::endl;
      exit(-1);
    }

    auto state_dbg = m_state.Update(m_motors, m_imu);
    auto[torque, ctrl_debug] = m_ctrl.Step(m_state, m_joy);

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
    json["ctrl"] = ctrl_debug;
    json["state"] = state_dbg;

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
  Vehicle(std::shared_ptr<Config> conf)
      : m_state(conf), m_joy(*this), m_ctrl(conf), m_conf(conf), m_stream(*this), m_signals(*this, SIGINT), m_d_timer(*this, boost::posix_time::seconds(50)),
        m_file_io_strand(*this) {

    SetupCan();
    SetSigintHandler();
    ScheduleCanReceive();
    SetStopAll();
    ScheduleBnoReceiveTimed();
  }

};

int main(int argc, char *argv[]) {

  auto conf = std::make_shared<Config>();

  std::string dump_file_path = (*conf)["dump_base_path"].get<std::string>() + "dump.msg";

  if (std::remove(dump_file_path.c_str())) {
    std::cout << "Cannot remove dump.msg!" << std::endl;
  } else {
    std::cout << "dump.msg has been removed." << std::endl;
  }

  Vehicle veh(conf);
  veh.run();

  return 0;

}