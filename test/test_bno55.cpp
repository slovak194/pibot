//
// Created by slovak on 12/17/20.
//

#include <iostream>

#include <thread>
#include <chrono>

using namespace std::chrono_literals;

//#include "bno055.h"
#include "bno055_interface.h"

int main(int argc, char * argv[]){

//  std::cout << "fasdf" << std::endl;

  struct bno055_euler_double_t d_euler_hpr;
  struct bno055_euler_t euler_hrp;
  struct bno055_euler_t euler_hrp_prev;

  bno055_init();

  int i = 0;



  auto t0 = std::chrono::steady_clock::now();

  while (i < 100000) {

//    bno055_read();

//    auto result = bno055_convert_double_euler_hpr_deg(&d_euler_hpr);
//    std::cout << result << " " << "d_euler_hpr: " << d_euler_hpr.h << " " << d_euler_hpr.p << " " << d_euler_hpr.r << "\n";

    auto result = bno055_read_euler_hrp(&euler_hrp);

    if (euler_hrp_prev.h != euler_hrp.h || euler_hrp_prev.r != euler_hrp.r || euler_hrp_prev.p != euler_hrp.p) {
      auto t1 = std::chrono::steady_clock::now();
      std::cout << result << " " << "euler_hrp: " << euler_hrp.h << " " << euler_hrp.p << " " << euler_hrp.r << "\n";

      auto diff_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
      std::cout << 1.0 / ((1e-6)*diff_us) << std::endl;

      t0 = t1;
      euler_hrp_prev = euler_hrp;

    }


//    std::this_thread::sleep_for(50ms);
    i++;
  }

  return 0;
}
