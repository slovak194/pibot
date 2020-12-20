#!/usr/bin/env bash

# sudo ip link set can0 up type can bitrate 1000000   dbitrate 8000000 restart-ms 1000 berr-reporting on fd on
# sudo ip link set can1 up type can bitrate 1000000   dbitrate 8000000 restart-ms 1000 berr-reporting on fd on

sudo ip link set can0 up type can tq 25 prop-seg 13 phase-seg1 12 phase-seg2 14 sjw 5 dtq 25 dprop-seg 3 dphase-seg1 1 dphase-seg2 3 dsjw 3 restart-ms 1000 fd on
sudo ip link set can1 up type can tq 25 prop-seg 13 phase-seg1 12 phase-seg2 14 sjw 5 dtq 25 dprop-seg 3 dphase-seg1 1 dphase-seg2 3 dsjw 3 restart-ms 1000 fd on

#sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 restart-ms 1000 berr-reporting on fd on
#sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 restart-ms 1000 berr-reporting on fd on

sudo ifconfig can0 txqueuelen 65536
sudo ifconfig can1 txqueuelen 65536

ip -details link show can0
ip -details link show can1

#can send 8002 ABCDABCDABCDABCDABCDABCDABCDABCDABCDABCDABCDABCDABCDABCDABCDABCD

