#include <stdio.h>
#include <bitset>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <iostream>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "moteus_protocol.h"

using namespace mjbots::moteus;


void data_send(boost::asio::posix::stream_descriptor &stream, boost::asio::deadline_timer &timer) {
  std::cout << "sending ..." << std::endl;

  CanFrame f;
  WriteCanFrame can_frame{&f};

  PositionCommand pos;
  PositionResolution res;

  res.position = Resolution::kFloat;
  res.velocity = Resolution::kFloat;
  res.feedforward_torque = Resolution::kFloat;
  res.kp_scale = Resolution::kIgnore;
  res.kd_scale = Resolution::kIgnore;
  res.maximum_torque = Resolution::kFloat;
  res.stop_position = Resolution::kIgnore;
  res.watchdog_timeout = Resolution::kIgnore;

  pos.position = 0.0;
  pos.velocity = 0.0;
  pos.feedforward_torque = 0.0;
  pos.kd_scale = 1.0;
  pos.maximum_torque = 0.1;
  pos.stop_position = 0.0;
  pos.watchdog_timeout = 0.5;

  EmitPositionCommand(&can_frame, pos, res);

  struct canfd_frame frame;

  for (int i = 0; i < f.size; i++) {
    frame.data[i] = f.data[i];
  }

  frame.can_id = 0x00008002;
  frame.len = f.size;
  frame.flags = 1;

  stream.async_write_some(boost::asio::buffer(&frame, sizeof(frame)),
                          [](auto ... vn){std::cout << "sent" << std::endl;});

  timer.expires_from_now(boost::posix_time::milliseconds(50));

  timer.async_wait([&stream, &timer](auto ... vn){
    data_send(stream, timer);
  });

}

void data_rec(struct canfd_frame &rec_frame, boost::asio::posix::stream_descriptor &stream) {
  std::cout << std::hex << rec_frame.can_id << "  ";
  for (int i = 0; i < rec_frame.len; i++) {
    std::cout << std::hex << int(rec_frame.data[i]) << " ";
  }

  std::cout << "flags: |" << std::bitset<8>(rec_frame.flags) << "|" << static_cast<unsigned int>(rec_frame.flags) << "|";

  std::cout << std::dec << std::endl;
  stream.async_read_some(
      boost::asio::buffer(&rec_frame, sizeof(rec_frame)),
      boost::bind(data_rec, boost::ref(rec_frame), boost::ref(stream)));
}

int main(void) {
  struct sockaddr_can addr;
  struct canfd_frame rec_frame;
  struct ifreq ifr;

  int natsock = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  // Enable reception of CAN FD frames
  {
    int enable = 1;

    auto rc = ::setsockopt(
        natsock,
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
  ioctl(natsock, SIOCGIFINDEX, &ifr);

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(natsock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("Error in socket bind");
    return -2;
  }

  CanFrame f;
  WriteCanFrame can_frame{&f};

  EmitStopCommand(&can_frame);

  struct canfd_frame frame;

  for (int i = 0; i < f.size; i++) {
    frame.data[i] = f.data[i];
  }

  frame.can_id = 0x00008002;
  frame.len = f.size;
  frame.flags = 1;

  boost::asio::io_service ios;
  boost::asio::posix::stream_descriptor stream(ios);
  stream.assign(natsock);

  stream.async_write_some(boost::asio::buffer(&frame, sizeof(frame)),
                          [](auto ... vn){std::cout << "sent" << std::endl;});

  boost::asio::deadline_timer timer(ios, boost::posix_time::milliseconds(50));

  timer.async_wait([&stream, &timer](auto ... vn){
    data_send(stream, timer);
  });

  stream.async_read_some(
      boost::asio::buffer(&rec_frame, sizeof(rec_frame)),
      boost::bind(data_rec, boost::ref(rec_frame), boost::ref(stream)));
  ios.run();
}
