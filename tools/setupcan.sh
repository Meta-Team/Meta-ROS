setup_can () {
	if [ -d /sys/class/net/$1 ] ; then
		sudo ip link set $1 type can bitrate 1000000
		sudo ip link set $1 up
	fi
}
setup_can can_1
setup_can can_2
setup_can can_3
# fallback to some using standalone usb2can devices
setup_can can0
