//
// Created by slovak on 12/17/20.
//

#include <iostream>

#include <boost/asio.hpp>

using namespace std::chrono_literals;
using namespace boost;

#include "Imu.h"

class BnoService : public asio::io_service {
 private:

  Imu m_bno_interface;

  void schedule_receive() {
    post([this](){this->receive();});
  }
  void receive(void) {
    auto euler = m_bno_interface.GetEuler();
    std::cout << "euler: " << euler.h << " " << euler.p << " " << euler.r << "\n";

    schedule_receive();
  }

 public:
  BnoService() {
    schedule_receive();
  }

};


int main(int argc, char * argv[]){

  BnoService bno;
  bno.run();

  return 0;
}
