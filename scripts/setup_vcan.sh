#!/usr/bin/env bash

# sudo ip link set can0 up type can bitrate 1000000   dbitrate 8000000 restart-ms 1000 berr-reporting on fd on
# sudo ip link set can1 up type can bitrate 1000000   dbitrate 8000000 restart-ms 1000 berr-reporting on fd on

sudo ip link add dev vcan0 type vcan fd on
sudo ip link set up vcan0
#
#sudo ip link set can0 up type can bitrate 1000000 dbitrate 1000000 restart-ms 1000 berr-reporting on fd on
#sudo ip link set can1 up type can bitrate 1000000 dbitrate 1000000 restart-ms 1000 berr-reporting on fd on
#
#sudo ifconfig can0 txqueuelen 65536
#sudo ifconfig can1 txqueuelen 65536
#
#ip -details link show can0
#ip -details link show can1
