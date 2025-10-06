The install assumes that the active ROS 2 Humble workspace is named "cda_ws" and is within the user's home directory (~/cda_ws).

```
sudo cp ./micro-ros-startup.sh /usr/bin/
sudo cp ./teleop-joy-startup.sh /usr/bin/
sudo cp ./micro-ros-startup.service /etc/systemd/system/
sudo cp ./teleop-joy-startup.timer /etc/systemd/system/
sudo cp ./teleop-joy-startup.service /etc/systemd/system/
sudo systemctl enable micro-ros-startup.service
sudo systemctl enable teleop-joy-startup.timer

sudo cp ./50-input.rules /etc/udev/rules.d/50-input.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```
