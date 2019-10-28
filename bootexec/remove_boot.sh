sudo update-rc.d -f run_emucd remove
sudo rm -f /usr/sbin/emucd_64
sudo rm -f /etc/init.d/run_emucd
sudo rm -f /etc/init.d/emuc2socketcan.ko
#rm /lib/modules/$(uname -r)/kernel/drivers/net/can/emuc2socketcan.ko
