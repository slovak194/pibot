# https://wiki.seeedstudio.com/2-Channel-CAN-BUS-FD-Shield-for-Raspberry-Pi/
# git clone https://github.com/Seeed-Studio/seeed-linux-dtoverlays
# cd seeed-linux-dtoverlays//modules/CAN-HAT
# sudo ./install.sh 
# sudo reboot

# pi@raspberrypi:~ $ dmesg | grep spi
# [    6.178008] mcp25xxfd spi0.0 can0: MCP2517FD rev0.0 (-RX_INT +MAB_NO_WARN +CRC_REG +CRC_RX +CRC_TX +ECC -HD m:20.00MHz r:18.50MHz e:0.00MHz) successfully initialized.
 
# pi@raspberrypi:~ $ ifconfig -a
# can0: flags=128<NOARP>  mtu 16
#         unspec 00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00  txqueuelen 10  (UNSPEC)
#         RX packets 0  bytes 0 (0.0 B)
#         RX errors 0  dropped 0  overruns 0  frame 0
#         TX packets 0  bytes 0 (0.0 B)
#         TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
#         device interrupt 166
 
# can1: flags=128<NOARP>  mtu 16
#         unspec 00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00  txqueuelen 10  (UNSPEC)
#         RX packets 0  bytes 0 (0.0 B)
#         RX errors 0  dropped 0  overruns 0  frame 0
#         TX packets 0  bytes 0 (0.0 B)
#         TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
#         device interrupt 167




# sudo ip link set can0 up type can bitrate 1000000   dbitrate 8000000 restart-ms 1000 berr-reporting on fd on
# sudo ip link set can1 up type can bitrate 1000000   dbitrate 8000000 restart-ms 1000 berr-reporting on fd on
 
# sudo ifconfig can0 txqueuelen 65536
# sudo ifconfig can1 txqueuelen 65536


# sudo ip link set can0 up type can bitrate 500000
# ip -details link show can0


import can
 
can_interface = 'can0'
bus = can.interface.Bus(can_interface, bustype='socketcan_native')
while True:
    message = bus.recv(1.0) # Timeout in seconds.
    if message is None:
            print('Timeout occurred, no message.')
    print(message)




mcp -> fdcanusb 1.604 us

