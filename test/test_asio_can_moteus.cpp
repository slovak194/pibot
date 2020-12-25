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

struct Motor {
  std::uint32_t m_can_send_resp_id;
  std::uint32_t m_can_send_no_resp_id;
  std::uint32_t m_can_receive_id;
  std::uint32_t m_id;
  canfd_frame m_send_frame;
  canfd_frame m_receive_frame;

  moteus::QueryResult m_state;

  explicit Motor(std::uint32_t id = 1U)
      : m_id(id), m_can_send_resp_id(EXT_ID_ENABLE | FORCE_SERVO_RESPONCE | id), m_can_send_no_resp_id(EXT_ID_ENABLE | id), m_can_receive_id(0x100 * id) {
  }

  void dump() {
    const auto result = moteus::ParseQueryResult(m_receive_frame.data, m_receive_frame.len);

    std::cout << std::hex << m_receive_frame.can_id << std::dec
              << /*" position: " << result.position <<*/ " velocity: " << result.velocity << std::endl;
  }

  [[nodiscard]] auto BuildTorqueBuffer(double t_Nm) const {

    if (std::abs(t_Nm) > 1.0) {
      t_Nm = 0.0;
    }

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

    pos.feedforward_torque = t_Nm;
    pos.kp_scale = 0.0;
    pos.kd_scale = 0.0;
    pos.watchdog_timeout = 0.1;

    EmitPositionCommand(&can_frame, pos, res);

    auto frame_sh_ptr = std::make_shared<canfd_frame>();

    for (int i = 0; i < 64; i++) {
      frame_sh_ptr->data[i] = f.data[i];
    }

    frame_sh_ptr->can_id = m_can_send_no_resp_id;
    frame_sh_ptr->len = f.size;
    frame_sh_ptr->flags = 1;

    return std::make_pair(boost::asio::buffer(&(*frame_sh_ptr), sizeof(*frame_sh_ptr)), [frame_sh_ptr](auto ...vn) {});
  }

  [[nodiscard]] auto BuildQueryBuffer() const {

    moteus::CanFrame f;
    moteus::WriteCanFrame can_frame{&f};

    moteus::QueryCommand cmd;

    cmd.mode = moteus::Resolution::kInt16;
    cmd.position = moteus::Resolution::kFloat;
    cmd.velocity = moteus::Resolution::kFloat;
    cmd.torque = moteus::Resolution::kFloat;
    cmd.q_current = moteus::Resolution::kIgnore;
    cmd.d_current = moteus::Resolution::kIgnore;
    cmd.rezero_state = moteus::Resolution::kIgnore;
    cmd.voltage = moteus::Resolution::kInt8;
    cmd.temperature = moteus::Resolution::kInt8;
    cmd.fault = moteus::Resolution::kInt8;

    moteus::EmitQueryCommand(&can_frame, cmd);

    auto frame_sh_ptr = std::make_shared<canfd_frame>();

    for (int i = 0; i < 64; i++) {
      frame_sh_ptr->data[i] = f.data[i];
    }

    frame_sh_ptr->can_id = m_can_send_resp_id;
    frame_sh_ptr->len = f.size;
    frame_sh_ptr->flags = 1;

    return std::make_pair(boost::asio::buffer(&(*frame_sh_ptr), sizeof(*frame_sh_ptr)), [frame_sh_ptr](auto ...vn) {});
  }

  [[nodiscard]] auto BuildStopBuffer() const {
    moteus::CanFrame f;
    moteus::WriteCanFrame can_frame{&f};

    EmitStopCommand(&can_frame);

    auto frame_sh_ptr = std::make_shared<canfd_frame>();

    for (int i = 0; i < 64; i++) {
      frame_sh_ptr->data[i] = f.data[i];
    }

    frame_sh_ptr->can_id = m_can_send_resp_id;
    frame_sh_ptr->len = f.size;
    frame_sh_ptr->flags = 1;

    return std::make_pair(
        boost::asio::buffer(&(*frame_sh_ptr), sizeof(*frame_sh_ptr)),
        [frame_sh_ptr](auto ...vn) { std::cout << "stop sent" << std::endl; }
    );

  }

  void MaybeReceive(canfd_frame frame) {
    if (frame.can_id == m_can_receive_id) {
      m_receive_frame = frame;
      m_state = moteus::ParseQueryResult(m_receive_frame.data, m_receive_frame.len);
    }
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

    json["velocity0"] = m_motors[0].m_state.velocity;
    json["velocity1"] = m_motors[1].m_state.velocity;

    json["position0"] = m_motors[0].m_state.position;
    json["position1"] = m_motors[1].m_state.position;

    post(m_file_io_strand.wrap([json](){
      const auto msgpack = nlohmann::json::to_msgpack(json);
      std::ofstream("dump.msg", std::ios::app | std::ios::binary).write(
          reinterpret_cast<const char *>(msgpack.data()), msgpack.size() * sizeof(uint8_t));
    }));

    for (int n = 0; n < m_motors.size(); n++) {
      auto[ctrl_buffer, ctrl_handler] = m_motors[n].BuildTorqueBuffer(torque[n]);
      m_stream.async_write_some(ctrl_buffer, ctrl_handler);

      auto[query_buffer, query_handler] = m_motors[n].BuildQueryBuffer();
      m_stream.async_write_some(query_buffer, query_handler);
    }
    ScheduleBnoReceiveTimed();
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