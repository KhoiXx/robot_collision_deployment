echo "Configure usb rules /etc/udev/rules.d"
sudo cp -f ../rules/robot_rules.rules /etc/udev/rules.d

sudo udevadm control --reload
sudo udevadm trigger
echo "Finish!!!"