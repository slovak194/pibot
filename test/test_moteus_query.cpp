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

using namespace mjbots;


void data_send(canfd_frame &frame, boost::asio::posix::stream_descriptor &stream, boost::asio::deadline_timer &timer);
void ScheduleCanSend(canfd_frame &frame, boost::asio::posix::stream_descriptor &stream, boost::asio::deadline_timer &timer);

void OnCanSend(canfd_frame &frame, boost::asio::posix::stream_descriptor &stream, boost::asio::deadline_timer &timer) {
//  std::cout << "omg sent" << std::endl;

  ScheduleCanSend(frame, stream, timer);
}

void ScheduleCanSend(canfd_frame &frame, boost::asio::posix::stream_descriptor &stream, boost::asio::deadline_timer &timer) {
//  std::cout << "receive scheduled .." << std::endl;

  timer.expires_from_now(boost::posix_time::milliseconds(10));

  timer.async_wait([&](auto ...vn){
    stream.async_write_some(
        boost::asio::buffer(&frame, sizeof(frame)),
        [&](auto ...vn){
          OnCanSend(frame, stream, timer);
        });
  });

}

void data_rec(struct canfd_frame &rec_frame, boost::asio::posix::stream_descriptor &stream) {
//  std::cout << std::hex << rec_frame.can_id << "  ";
//  for (int i = 0; i < rec_frame.len; i++) {
//    std::cout << std::hex << int(rec_frame.data[i]) << " ";
//  }
//
//  std::cout << "flags: |" << std::bitset<8>(rec_frame.flags) << "|" << static_cast<unsigned int>(rec_frame.flags) << "|";
//
//  std::cout << std::dec << std::endl;

  const auto result = moteus::ParseQueryResult(rec_frame.data, rec_frame.len);
  std::cout /*<< " mode: " << int(result.mode) << " position: " << result.position*/
  << " velocity: " << result.velocity*1000 << std::endl;

  stream.async_read_some(
      boost::asio::buffer(&rec_frame, sizeof(rec_frame)),
      boost::bind(data_rec, boost::ref(rec_frame), boost::ref(stream)));
}

int main(void) {
  struct sockaddr_can addr;
  struct canfd_frame frame;
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

  moteus::CanFrame f1;
  moteus::WriteCanFrame can_frame_1{&f1};

  moteus::QueryCommand cmd;
  moteus::EmitQueryCommand(&can_frame_1, cmd);

  for (int i = 0; i < 64; i++) {
    frame.data[i] = f1.data[i];
  }

  constexpr std::uint32_t EXT_ID_ENABLE = 1U << 31U;
  constexpr std::uint32_t FORCE_SERVO_RESPONCE = 1U << 15U;
  std::uint32_t id = 1U;

  frame.len = f1.size;
  frame.can_id = (EXT_ID_ENABLE | FORCE_SERVO_RESPONCE | id);
  frame.flags = 1;

//  00008001  [05]  14 04 00 13 0D query?

  boost::asio::io_service ios;
  boost::asio::posix::stream_descriptor stream(ios);
  stream.assign(natsock);

  boost::asio::deadline_timer timer(ios, boost::posix_time::milliseconds(10));

  ScheduleCanSend(frame, stream, timer);

//  stream.async_write_some(boost::asio::buffer(&frame, sizeof(frame)),
//                          boost::bind(data_send));
  stream.async_read_some(
      boost::asio::buffer(&rec_frame, sizeof(rec_frame)),
      boost::bind(data_rec, boost::ref(rec_frame), boost::ref(stream)));
  ios.run();
}
