#include "Joystick.h"


auto main(int argc, char** argv) -> int {
//  if (argc != 2) {
//    std::cerr << "Usage: " << argv[0] << " <device>" << std::endl;
//    return -1;
//  }
//
//  {
//    char name[255];
//    int axes, buttons;
//    ioctl(fd, JSIOCGNAME(255), name);
//    ioctl(fd, JSIOCGAXES, &axes);
//    ioctl(fd, JSIOCGBUTTONS, &buttons);
//
//    std::cout << name << ": " << axes << " axes/" << buttons << " buttons" << std::endl;
//  }

  boost::asio::io_service io_service;

  Joystick j(io_service);

  io_service.run();
}