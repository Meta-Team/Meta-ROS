#!/bin/bash
meta_ros_dir=$(realpath $(realpath $(dirname $0))/..)
username=$(whoami)
cat << EOF | sudo tee /etc/rc.local
#!/bin/sh
wait_can(){
	if [ -d /sys/class/net/can_1 ] && [ -d /sys/class/net/can_2 ] && [ -d /sys/class/net/can_3 ]; then 
		echo all can device are loaded
	else
		sleep 0.5
		wait_can
	fi
}
wait_can
$meta_ros_dir/tools/setupcan.sh
nohup sudo -u $username $meta_ros_dir/tools/boot_run.sh &
EOF
