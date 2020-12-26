//
// Created by slovak on 12/26/20.
//

#ifndef PIBOT_INCLUDE_MOTOR_H_
#define PIBOT_INCLUDE_MOTOR_H_

#include <memory>
#include <iostream>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <boost/asio.hpp>

#include "moteus_protocol.h"

using namespace mjbots;

struct Motor {
  static constexpr std::uint32_t EXT_ID_ENABLE = 1U << 31U;
  static constexpr std::uint32_t FORCE_SERVO_RESPONCE = 1U << 15U;

  std::uint32_t m_can_send_resp_id;
  std::uint32_t m_can_send_no_resp_id;
  std::uint32_t m_can_receive_id;
  std::uint32_t m_id;
  canfd_frame m_receive_frame;

  bool m_reversed = false;

  moteus::QueryResult m_state;

  explicit Motor(std::uint32_t id = 1U, bool reversed = false)
      : m_reversed(reversed), m_id(id), m_can_send_resp_id(EXT_ID_ENABLE | FORCE_SERVO_RESPONCE | id), m_can_send_no_resp_id(EXT_ID_ENABLE | id), m_can_receive_id(0x100 * id) {
  }

  void dump() {
    std::cout << std::hex << m_receive_frame.can_id << std::dec
              << /*" position: " << result.position <<*/ " velocity: " << m_state.velocity << std::endl;
  }

  [[nodiscard]] auto BuildTorqueBuffer(double t_Nm) const {

    if (std::abs(t_Nm) > 1.0) {
      t_Nm = 0.0;
    }

    if (m_reversed) {
      t_Nm = t_Nm * -1;
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
        [frame_sh_ptr](auto ...vn) { std::cout << "stop sent to " << std::hex << frame_sh_ptr->can_id << std::dec << std::endl; }
    );

  }

  void MaybeReceive(canfd_frame frame) {
    if (frame.can_id == m_can_receive_id) {
      m_receive_frame = frame;
      m_state = moteus::ParseQueryResult(m_receive_frame.data, m_receive_frame.len);

      if (m_reversed) {
        m_state.velocity = m_state.velocity * -1;
        m_state.position = m_state.position * -1;
      }

    }
  }

};

#endif //PIBOT_INCLUDE_MOTOR_H_
