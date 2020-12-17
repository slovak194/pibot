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

  auto t0 = std::chrono::steady_clock::now();

  while (i < 100000) {

    auto euler = bno.GetEuler();
    std::cout << "euler: " << euler.h << " " << euler.p << " " << euler.r << "\n";

    std::this_thread::sleep_for(10ms);
    i++;
  }

  return 0;
}
