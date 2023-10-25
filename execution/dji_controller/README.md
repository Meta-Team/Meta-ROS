# dji_controller

## description

this package is used to control the DJI motors

the motors are controlled by CAN communication

each can_frame delivers 4 values (id from 1 to 4, or from 5 to 8). each value is a 16-bit integer, taking 2 bytes

so, 4 motors should be controlled at a time

the bitrate of the CAN communication should be 1M

CAN frames should be continuously sent to the motors to keep them running. in this case, frames are sent at 1000Hz

## requirements

similar to dm_controller

to enable CAN communication on Orin Nano,

you need to run the following commands:

```
sudo apt-get install busybox

sudo busybox devmem 0x0c303018 w 0xc458
sudo busybox devmem 0x0c303010 w 0xc400

modprobe can
modprobe can_raw
modprobe mttcan

ip link set can0 up type can bitrate 1000000 dbitrate 1000000 berr-reporting on fd on
```

## reference

https://docs.nvidia.com/jetson/archives/r35.3.1/DeveloperGuide/text/HR/ControllerAreaNetworkCan.html