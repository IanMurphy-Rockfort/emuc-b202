sudo pkill -2 emucd_64
sleep 0.2
sudo rmmod emuc2socketcan
sudo insmod emuc2socketcan.ko
#sudo cp emuc2socketcan.ko /lib/modules/$(uname -r)/kernel/drivers/net/can
#sudo depmod -a
#sudo modprobe emuc2socketcan
sudo ./emucd_64 -s9 ttyACM0 can0 can1
sudo sudo ip link set can0 up qlen 1000
sudo sudo ip link set can1 up qlen 1000
