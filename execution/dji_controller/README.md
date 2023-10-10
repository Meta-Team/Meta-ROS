# dji_controller

similar to dm_controller

## requirements

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