# haru_drums_ros_driver

The haru_drums_ros_driver package is a versatile and user-friendly solution for integrating a drum kit (musical instrument) with the Robot Operating
System (ROS) ecosystem. This package aims to bridge the gap between the musical world and robotics, providing a seamless connection between the drum
kit hardware and ROS-based applications.


## Usage
### Step 1: Drum configuration

Run this command the first time you use the drums

```shell
rosrun haru_drums_ros_driver run_drum_setup.sh
```

### Init drum driver


```shell
roslaunch haru_drums_ros_driver midi_drum_driver.launch
```
