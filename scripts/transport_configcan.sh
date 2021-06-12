echo "nvidia" | sudo -S ifconfig can0 down
echo "nvidia" | sudo -S ifconfig can1 down
echo "nvidia" | sudo -S ip link set can0 type can bitrate 250000
echo "nvidia" | sudo -S ip link set up can0
echo "nvidia" | sudo -S ip link set can1 type can bitrate 500000
echo "nvidia" | sudo -S ip link set up can1
