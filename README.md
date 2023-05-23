# haru_drums_ros_driver

The haru_drums_ros_driver package is a versatile and user-friendly solution for integrating a drum kit (musical instrument) with the Robot Operating
System (ROS) ecosystem. This package aims to bridge the gap between the musical world and robotics, providing a seamless connection between the drum
kit hardware and ROS-based applications.


## Usage
### Step 1: Drum configuration

Run this command the first time you use the drums. Follow the steps to set up your default midi port,
sounds and the colors of the drum pads

```shell
rosrun haru_drums_ros_driver run_drum_setup.sh
```

### Step 2: Init drum driver
The drum driver node is responsible for capturing drum hits and publishing
DrumMidiSignal messages.
```shell
roslaunch haru_drums_ros_driver midi_drum_driver.launch
```

## Use your custom sounds
To create a new sound set you will need seven sound tracks, preferably with a duration shorter than half a second
to avoid latency issues.

Place your seven tracks inside a new folder in ``src/sounds`` and select them during the configuration process (Examples: Minecraft and Nintendo
sound tracks).

## Record and play beats
If you want to record and play your own beats you can do it via command line like so:

```shell
rosrun haru_drums_ros_driver record_beat.sh <name_of_your_beat>
```

To play an specific beat, use the next command:
```shell
rosrun haru_drums_ros_driver play_beat.sh <name_of_your_beat>
```

Sound sets and beats are stored in ``src/beats`` folder.

