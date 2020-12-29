//
// Created by slovak on 12/29/20.
//

#ifndef PIBOT_INCLUDE_JOYSTICK_H_
#define PIBOT_INCLUDE_JOYSTICK_H_

#include <iomanip>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

extern "C" {
#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <unistd.h>
}

class Joystick {
 public:

  /** Maximum value of axes range */
  static const short MAX_AXES_VALUE = 32767;


  Joystick(boost::asio::io_service& io_service) : joystick_(io_service) {

    const auto fd = open("/dev/input/js0", O_RDONLY);
    if (fd < 0) {
      std::cerr << "Cannot open \"/dev/input/js0\" " << /*argv[1] << */": " << strerror(errno) << std::endl;
      exit(-1);
    }

//    button:  0, value: 0
//    button:  1, value: 0
//    button:  2, value: 0
//    button:  3, value: 0
//    button:  4, value: 0
//    button:  5, value: 0
//    button:  6, value: 0
//    button:  7, value: 0
//    button:  8, value: 0
//    button:  9, value: 0
//    button: 10, value: 0
//    axis:  0, value: 0 // LEFT -, RIGHT +
//    axis:  1, value: -2  // FORWARD - BACKWARD +
//    axis:  2, value: -32767
//    axis:  3, value: ROT RIGHT + ROT LEFT -
//    axis:  4, value: -2
//    axis:  5, value: -32767
//    axis:  6, value: 0
//    axis:  7, value: 0


    joystick_.assign(fd);

    boost::asio::async_read(
        joystick_, buffer_, boost::asio::transfer_exactly(sizeof(js_event)),
        boost::bind(&Joystick::handle_read, this, boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
  }

  ~Joystick() {
    joystick_.close();
  }

// private:

  void handle_read(const boost::system::error_code& error, std::size_t bytes_transferred) {
    if (error) {
      std::cerr << "error: " << error.message() << std::endl;
      return;
    }

    if (bytes_transferred == sizeof(js_event)) {
      auto jse = boost::asio::buffer_cast<const js_event*>(buffer_.data());

      switch (jse->type & ~JS_EVENT_INIT) {
        case JS_EVENT_BUTTON:
          m_buttons[jse->number] = static_cast<bool>(jse->value);
          std::cout << "button: ";

          std::cout << std::setw(2) << static_cast<int>(jse->number) << ", value: " << jse->value
                    << std::endl;
          break;
        case JS_EVENT_AXIS:

          auto value = static_cast<double>(jse->value);

          if (value > 0.0) {
            value = value / static_cast<double>(MAX_AXES_VALUE);
          }

          if (value < 0.0) {
            value = value / static_cast<double>(MAX_AXES_VALUE);
          }

          m_axes[jse->number] = value;
//          std::cout << "  axis: ";
          break;
      }

//      for (const auto& b : m_buttons) {
//        std::cout << b << " ";
//      }
//      for (const auto& a : m_axes) {
//        std::cout << a << "\t";
//      }
//
//      std::cout << std::endl;

    }

    buffer_.consume(bytes_transferred);

    boost::asio::async_read(
        joystick_, buffer_, boost::asio::transfer_exactly(sizeof(js_event)),
        boost::bind(&Joystick::handle_read, this, boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
  }

  std::array<bool, 11> m_buttons = {false};
  std::array<double, 8> m_axes = {0.0};

  boost::asio::posix::stream_descriptor joystick_;
  boost::asio::streambuf buffer_;
};


#endif //PIBOT_INCLUDE_JOYSTICK_H_
