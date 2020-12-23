//
// Created by slovak on 12/17/20.
//

#include <iostream>

#include <thread>
#include <chrono>

using namespace std::chrono_literals;

#include "bno055_interface.h"

int main(int argc, char * argv[]){

  BNO055 bno;

  int i = 0;

  std::uint32_t tt = 0U;

  while (i < 1000000) {

//    auto euler = bno.GetEuler();
//    std::cout << "euler: " << euler.h << " " << euler.p << " " << euler.r << "\n";

    auto t0 = std::chrono::steady_clock::now();
    auto gyro_y = bno.GetGyroY();
    auto t1 = std::chrono::steady_clock::now();


    std::uint32_t t = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();

    if (tt == 0) {
      tt = t;
    } else {
      if (t > tt) {
        tt = t;
      }
    }

//    std::cout << " tt: " << tt << " t: " << t << "\n";
    std::cout << " gyro_y: " << gyro_y << "\n";

//    std::this_thread::sleep_for(10ms);
    i++;
  }

  return 0;
}
