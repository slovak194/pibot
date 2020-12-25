#include <stdio.h>
#include <memory>
#include <chrono>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
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

#include "moteus_protocol.h"

#include "bno055_interface.h"

using namespace mjbots;
using namespace std::chrono_literals;

constexpr std::uint32_t EXT_ID_ENABLE = 1U << 31U;
constexpr std::uint32_t FORCE_SERVO_RESPONCE = 1U << 15U;


template<typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

struct Motor {
  std::uint32_t m_can_send_resp_id;
  std::uint32_t m_can_send_no_resp_id;
  std::uint32_t m_can_receive_id;
  std::uint32_t m_id;
  canfd_frame m_send_frame;
  canfd_frame m_receive_frame;

  explicit Motor(std::uint32_t id = 1U)
      : m_id(id), m_can_send_resp_id(EXT_ID_ENABLE | FORCE_SERVO_RESPONCE | id), m_can_send_no_resp_id(EXT_ID_ENABLE | id), m_can_receive_id(0x100 * id) {
  }

  void dump() {
    const auto result = moteus::ParseQueryResult(m_receive_frame.data, m_receive_frame.len);

    std::cout << std::hex << m_receive_frame.can_id << std::dec
              << /*" position: " << result.position <<*/ " velocity: " << result.velocity << std::endl;
  }

};

class Vehicle : public boost::asio::io_service {
 private:

  BNO055 m_bno_interface;

  struct sockaddr_can m_can_address;
  struct ifreq ifr;
  int m_native_socket;
  boost::asio::posix::stream_descriptor m_stream;
  boost::asio::signal_set m_signals;
  boost::asio::deadline_timer m_d_timer;

  std::vector<Motor> m_motors = {Motor(1), Motor(2)};

  canfd_frame m_receive_frame;

  std::atomic_bool m_running = true;

  std::chrono::time_point<std::chrono::steady_clock> m_time_prev[2];

  static std::vector<float> GetTorque(double pitch, double dpitch) {

    double k_p = 5.0;
    double k_d = 1.0;

    float f = static_cast<float>(pitch * k_p + dpitch * k_d);

    float wheel_r = 0.07f;

    float t = f*wheel_r/2.0f;

    return {t, -t};
  }

  void SetSigintHandler() {
    m_signals.async_wait([this](const boost::system::error_code &error, int signal_number) {
      this->m_running = false;
//      this->SetStopAll();
//      this->SetStopAll();
//      this->SetStopAll();
      this->post([](auto ... vn) {
        std::cout << "Exiting ..." << std::endl;
        exit(0);
      });
    });

  }

  void SetStopAll() {

    for (auto &m : m_motors) {

      moteus::CanFrame f;
      moteus::WriteCanFrame can_frame{&f};

      EmitStopCommand(&can_frame);

      for (int i = 0; i < 64; i++) {
        m.m_send_frame.data[i] = f.data[i];
      }

      m.m_send_frame.len = f.size;
      m.m_send_frame.flags = 1;

      m_stream.async_write_some(boost::asio::buffer(&m.m_send_frame, sizeof(m.m_send_frame)),
                                [](auto ... vn) { std::cout << "stop sent" << std::endl; });

    }

  }

  void ScheduleBnoReceiveTimed() {
    if (this->m_running) {
      m_d_timer.expires_from_now(boost::posix_time::milliseconds(4));
      m_d_timer.async_wait([this](auto ...vn) { this->BnoReceive(); });
    }
  }

  void BnoReceive() {

//    auto t0 = std::chrono::steady_clock::now();
    auto euler = m_bno_interface.GetEuler();
    auto pitch = euler.r * M_PI / 180.0;
//    auto t1 = std::chrono::steady_clock::now();
    auto gyro_y = m_bno_interface.GetGyroY();
//    auto t2 = std::chrono::steady_clock::now();

//    std::cout << "bno euler: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "\n";
//    std::cout << "bno gyro: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "\n";

//    if (std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t0).count() > 10) {
//      exit(0);
//    }

    auto torque = GetTorque(pitch, gyro_y);

    nlohmann::json json;

//    Eigen::Vector2d tt;
//
//    tt(0) = torque[0];
//    tt(1) = torque[1];

    json["torque0"] = torque[0];
    json["torque1"] = torque[1];
    json["pitch"] = pitch;
    json["gyro_y"] = gyro_y;

    const auto msgpack = nlohmann::json::to_msgpack(json);
    std::ofstream("dump.msg", std::ios::app | std::ios::binary).write(
        reinterpret_cast<const char *>(msgpack.data()), msgpack.size() * sizeof(uint8_t));


//    std::cout
//    << "euler.p: " << euler.p << "\t"
//    << "euler.h: " << euler.h << "\t"
//    << "euler.r: " << euler.r << "\n";

//    std::cout
//    << "gyro.x: " << gyro.x << "\t"
//    << "gyro.y: " << gyro.y << "\t"
//    << "gyro.z: " << gyro.z << "\n";

//    std::cout
//    << "torque[0]: " << torque[0] << "\n";



    for (int n = 0; n < m_motors.size(); n++) {



      if (std::abs(torque[n]) > 1.0) {
        torque[n] = 0.0;
      }

//      std::cout << torque[n] << std::endl;
      torque[n] = torque[n] * 0;


      moteus::CanFrame f;
      moteus::WriteCanFrame can_frame{&f};

      moteus::PositionCommand pos;
      moteus::PositionResolution res;

      res.position = moteus::Resolution::kIgnore;
      res.velocity = moteus::Resolution::kIgnore;
      res.feedforward_torque = moteus::Resolution::kFloat;
      res.kp_scale = moteus::Resolution::kFloat;
      res.kd_scale = moteus::Resolution::kFloat;
      res.maximum_torque = moteus::Resolution::kIgnore;
      res.stop_position = moteus::Resolution::kIgnore;
      res.watchdog_timeout = moteus::Resolution::kFloat;

      pos.feedforward_torque = torque[n];
      pos.kp_scale = 0.0;
      pos.kd_scale = 0.0;
      pos.watchdog_timeout = 0.1;

      EmitPositionCommand(&can_frame, pos, res);

      for (int i = 0; i < 64; i++) {
        m_motors[n].m_send_frame.data[i] = f.data[i];
      }

      m_motors[n].m_send_frame.can_id = m_motors[n].m_can_send_no_resp_id;

      m_motors[n].m_send_frame.len = f.size;
      m_motors[n].m_send_frame.flags = 1;

      m_stream.async_write_some(boost::asio::buffer(&m_motors[n].m_send_frame, sizeof(m_motors[n].m_send_frame)),
                                [this, n](auto ... vn) {});

      moteus::CanFrame f1;
      moteus::WriteCanFrame can_frame_1{&f1};

      moteus::QueryCommand cmd;
      moteus::EmitQueryCommand(&can_frame_1, cmd);

      for (int i = 0; i < 64; i++) {
        m_motors[n].m_send_frame.data[i] = f1.data[i];
      }

      m_motors[n].m_send_frame.can_id = m_motors[n].m_can_send_resp_id;

      m_motors[n].m_send_frame.len = f1.size;
      m_motors[n].m_send_frame.flags = 1;

      m_stream.async_write_some(boost::asio::buffer(&m_motors[n].m_send_frame, sizeof(m_motors[n].m_send_frame)),
                                [this, n](auto ... vn) {});

    }

    ScheduleBnoReceiveTimed();

  }

  void ScheduleCanReceive() {
    m_stream.async_read_some(
        boost::asio::buffer(&m_receive_frame, sizeof(m_receive_frame)),
        [this](auto ...vn){
          this->OnCanReceive(vn...);
        });
  }

  void OnCanReceive(const boost::system::error_code& error, size_t bytes_transferred){
    for (auto &m : m_motors) {
      if (m_receive_frame.can_id == m.m_can_receive_id) {
        m.m_receive_frame = m_receive_frame;
      }
    }

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
      : m_stream(*this), m_signals(*this, SIGINT), m_d_timer(*this, boost::posix_time::seconds(50)) {
    auto now = std::chrono::steady_clock::now();
    m_time_prev[0] = now;
    m_time_prev[1] = now;
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