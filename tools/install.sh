#!/bin/sh
cd $(dirname $(realpath $0))
sudo cp 01-robomaster-host1ton.rules /etc/udev/rules.d/ 
sudo udevadm control --reload && sudo udevadm trigger
