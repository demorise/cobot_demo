# ROS2 Cobot Demo

## Installation
1. Open a terminal and navigate to the src folder of your ROS2 colcon workspace, then run the following commands:
```bash
git clone https://github.com/DoveConsulting/cobot_demo.git
cd ..
colcon build --symlink-install
```

## Running
### Launch Robot in Rviz2
Open a terminal, navigate to the root of your colcon workspace and run the following commands:
```bash
source install/local_setup.bash
ros2 launch cobot_demo demo.launch.py use_fake_hardware:=true
``` 
