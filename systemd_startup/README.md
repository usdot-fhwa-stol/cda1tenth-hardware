```
cp ./micro-ros-startup.sh /usr/bin/
cp ./teleop-joy-startup.sh /usr/bin/
cp ./micro-ros-startup.service /etc/systemd/system/
cp ./teleop-joy-startup.timer .etc/systemd/system/
cp ./teleop-joy-startup.service /etc/systemd/system/
sudo systemctl enable micro-ros-startup.service
sudo systemctl enable teleop-joy-startup.timer
```
