# **cmr_driver**: Component execution
This package contains launch files to start all basic components (sensors, actuators, social functions) of the robot together with some basic nodes for message conversion.
The main launch files are cotained in the `launch/starters` folder. We use separate launch files to be able to kill and restart specific launch files at runtime.
To manually start all sensors run:
```
roslaunch cmr_driver sensors.launch
```
This will automatically start all sensor drivers (e.g. for IMU, Velodyne, Cameras, Platform). The camera drivers are run remotely on jetson and the MP-500 drivers on the MP-500 computer

To start social functions (Text-to-speech, Speech-to-Text, NLP, LED Stripes, LED Panel) run:
```
roslaunch cmr_driver social.launch
```
