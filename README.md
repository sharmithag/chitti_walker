# chitti_walker
## Custom Differential drive bot with Depth Camera
![Screenshot from 2022-12-07 18-55-18](https://user-images.githubusercontent.com/90351952/206322680-adbc5a71-e1bd-45a2-930b-3aa434e64a07.png)

## Custom World

![custom_world](https://user-images.githubusercontent.com/90351952/206322018-ac483ec2-b5e5-4635-9383-fb506395827d.png)

## IMPLEMENTATION

```
git clone https://github.com/sharmithag/chitti_walker.git
cd chitti_walker
source /opt/ros/foxy/setup.bash
<<<<<<< HEAD
gazebo --verbose chitti_custom_world
```

IN OTHER TERMINAL
```
source /opt/ros/foxy/setup.bash
colcon build
. install/setup.bash
ros2 launch chitti_walker chitti_walker.py

```
To check rosbag file in results

```
ros2 bag play rosbag2_2022_12_07-23_58_57
```
