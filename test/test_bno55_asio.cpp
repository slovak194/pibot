//
// Created by slovak on 12/17/20.
//

#include <iostream>

#include <thread>
#include <chrono>

#include <boost/asio.hpp>

using namespace std::chrono_literals;
using namespace boost;

#include "bno055_interface.h"

class BnoAsio : public asio::io_service {
 private:
  asio::deadline_timer m_timer;

  void schedule_receive() {
    m_timer.expires_from_now(boost::posix_time::seconds(1));
    m_timer.async_wait([this](auto ... vn){this->on_receive(vn...);});
  }
  void on_receive(const boost::system::error_code &error) {
    std::cout << "on_receive" << std::endl;
    schedule_receive();
  }

 public:
  BnoAsio() : m_timer(*this, boost::posix_time::seconds(1)) {
    schedule_receive();
    run();
  }

};


int main(int argc, char * argv[]){

//  BNO055 bno;
//
//  int i = 0;
//
//  auto t0 = std::chrono::steady_clock::now();
//
//  while (i < 100000) {
//
//    auto euler = bno.GetEuler();
//    std::cout << "euler: " << euler.h << " " << euler.p << " " << euler.r << "\n";
//
//    std::this_thread::sleep_for(10ms);
//    i++;
//  }

  BnoAsio bno;


  return 0;
}
