//
// Created by slovak on 12/17/20.
//

#include <iostream>

#include <thread>
#include <fstream>
#include <chrono>

#include <nlohmann/json.hpp>

#include "Imu.h"

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {

  if (std::remove("/home/pi/pibot/build/imu_dump.msg")) {
    std::cout << "Cannot remove imu_dump.msg!" << std::endl;
  } else {
    std::cout << "imu_dump.msg has been removed." << std::endl;
  }

  Imu imu;

  int i = 0;

  std::uint32_t t_max = 0U;

  bno055_accel_t m_acc = {0};

  bool first = true;

  while (i < 1000000) {

//    auto euler = bno.GetEuler();
//    std::cout << "euler: " << euler.h << " " << euler.p << " " << euler.r << "\n";

    auto t0 = std::chrono::steady_clock::now();
    auto acc = imu.GetAx();
    imu.Update();


//    bno055_accel_t acc;
//    auto result = bno055_convert_double_gravity_xyz_msq()


    auto t1 = std::chrono::steady_clock::now();

    std::uint32_t t = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

    if (t_max == 0) {
      t_max = t;
    } else {
      if (t > t_max) {
        t_max = t;
      }
    }

    if (first && (std::abs(acc.x) < 1000 && std::abs(acc.y) < 1000 && std::abs(acc.z) < 1000)) {
      first = false;
      m_acc = acc;
    } else {
      if (m_acc.x != acc.x || m_acc.y != acc.y || m_acc.z != acc.z) {

        if (std::abs(m_acc.x - acc.x) < 100 && std::abs(m_acc.y - acc.y) < 100 && std::abs(m_acc.z - acc.z) < 100) {
          nlohmann::json json;

          json["timestamp"] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

          json["x"] = acc.x;
          json["y"] = acc.y;
          json["z"] = acc.z;
          json["theta"] = imu.m_state.theta;

          const auto msgpack = nlohmann::json::to_msgpack(json);
          std::ofstream("imu_dump.msg", std::ios::app | std::ios::binary).write(
              reinterpret_cast<const char *>(msgpack.data()), msgpack.size() * sizeof(uint8_t));

          std::cout
              << "acc.x: " << acc.x << "\t"
              << "acc.y: " << acc.y << "\t"
              << "acc.z: " << acc.z << "\n";

          m_acc = acc;
        }
      }
    }


//    std::cout << " t_max: " << t_max << " t: " << t << "\n";


    i++;
  }

  return 0;
}
