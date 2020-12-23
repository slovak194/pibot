#include <stdio.h>
#include <bitset>
#include <memory>
#include <chrono>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <iostream>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/signal_set.hpp>

#include "moteus_protocol.h"

using namespace mjbots;
using namespace std::chrono_literals;

#include "bno055_interface.h"

template<typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

struct Motor {
  std::uint32_t m_can_send_id;
  std::uint32_t m_can_receive_id;
  std::uint32_t m_id;
  canfd_frame m_frame;

  explicit Motor(std::uint32_t id = 1U)
      : m_id(id), m_can_send_id((1U << 15U) + id), m_can_receive_id(0x100 + id) {
    m_frame.can_id = m_can_send_id;
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

  std::atomic_bool m_running = true;

  std::chrono::time_point<std::chrono::steady_clock> m_time_prev[2];

  static std::vector<float> GetTorque(double pitch, double dpitch) {

    double k_p = 0.3;
    double k_d = 0.03;

    auto t = static_cast<float>(pitch * k_p + dpitch * k_d);

    return {t, -t};
  }

  void SetSigintHandler() {
    m_signals.async_wait([this](const boost::system::error_code &error, int signal_number) {
      this->m_running = false;
      this->SetStopAll();
      this->SetStopAll();
      this->SetStopAll();
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
        m.m_frame.data[i] = f.data[i];
      }

      m.m_frame.len = f.size;
      m.m_frame.flags = 1;

      m_stream.async_write_some(boost::asio::buffer(&m.m_frame, sizeof(m.m_frame)),
                                [](auto ... vn) { std::cout << "stop sent" << std::endl; });

    }

  }

  void ScheduleBnoReceive() {
    if (this->m_running) {
      post([this]() { this->BnoReceive(); });
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
//    auto t1 = std::chrono::steady_clock::now();
    auto gyro_y = m_bno_interface.GetGyroY();
//    auto t2 = std::chrono::steady_clock::now();


//    std::cout << "bno euler: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "\n";
//    std::cout << "bno gyro: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "\n";

//    if (std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t0).count() > 10) {
//      exit(0);
//    }

    auto torque = GetTorque(euler.r * M_PI / 180.0, gyro_y);


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



      if (std::abs(torque[n]) > 1.5) {
        torque[n] = 1.5 * sgn(torque[n]);
      }

//      std::cout << torque[n] << std::endl;
//      torque[n] = torque[n] * 0;


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

//      pos.position = 0.0;
//      pos.velocity = 0.0;
      pos.feedforward_torque = torque[n];
      pos.kp_scale = 0.0;
      pos.kd_scale = 0.0;
//      pos.maximum_torque = 0.1;
//      pos.stop_position = 0.0;
      pos.watchdog_timeout = 0.1;

      EmitPositionCommand(&can_frame, pos, res);

      for (int i = 0; i < 64; i++) {
        m_motors[n].m_frame.data[i] = f.data[i];
      }

      m_motors[n].m_frame.len = f.size;
      m_motors[n].m_frame.flags = 1;

      m_stream.async_write_some(boost::asio::buffer(&m_motors[n].m_frame, sizeof(m_motors[n].m_frame)),
                                [this, n](auto ... vn) {
//                                  auto time_now = std::chrono::steady_clock::now();
//                                  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - this->m_time_prev[n - 1]).count();
//                                  std::cout << duration << "\n";
//                                  this->m_time_prev[n - 1] = time_now;
//                                  if (duration > 20) {
//                                    exit(0);
//                                  }
                                });

    }

//    ScheduleBnoReceive();
    ScheduleBnoReceiveTimed();

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
    SetStopAll();
//    ScheduleBnoReceive();
    ScheduleBnoReceiveTimed();
  }

};

int main(int argc, char *argv[]) {

  Vehicle veh;
  veh.run();

  return 0;

}