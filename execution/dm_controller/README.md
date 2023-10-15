# dm_controller

this package is used to control the DM motors

the motors are controlled by CAN communication

each can_frame controlls a single motor

motors have 3 modes: mit, vel, pos_vel

each has different can_frame protocol

## requirements

similar to dji_controller

to enable CAN communication on Orin Nano,

you need to run the following commands:

```
sudo apt-get install busybox

busybox devmem 0x0c303020 w 0x458

modprobe can
modprobe can_raw
modprobe mttcan

ip link set can0 up type can bitrate 500000 dbitrate 1000000 berr-reporting on fd on
```

## reference

https://docs.nvidia.com/jetson/archives/r35.3.1/DeveloperGuide/text/HR/ControllerAreaNetworkCan.html