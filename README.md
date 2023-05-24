# haru_drums_ros_driver

The haru_drums_ros_driver package is a versatile and user-friendly solution for integrating a drum kit (musical instrument) with the Robot Operating
System (ROS) ecosystem. This package aims to bridge the gap between the musical world and robotics, providing a seamless connection between the drum
kit hardware and ROS-based applications.


## Previous steps
### Step 1: Drum configuration

Run this command the first time you use the drums. Follow the steps to set up your default midi port,
sounds and the colors of the drum pads

```shell
rosrun haru_drums_ros_driver run_drum_setup.sh
```

### Step 2: Init drum driver
The drum driver node is responsible for capturing drum hits and publishing.
DrumMidiSignal messages. This node must remain running to be able to execute the rest of the commands that are taught below.
```shell
roslaunch haru_drums_ros_driver midi_drum_driver.launch
```

## Use your custom sounds
To create a new sound set you will need seven sound tracks, preferably with a duration shorter than half a second
to avoid latency issues.

Place your seven tracks inside a new folder in ``src/sounds`` and select them during the configuration process (Examples: Minecraft and Nintendo
sound tracks).

## Record and play beats
`beat_creator.py` module takes charge of Creating and reproducing beat files, which are scripts that contain information about
color hit, color id and the time between last hit.

If you want to record and play your own beats you can do it via command line like so:

```shell
rosrun haru_drums_ros_driver record_beat.sh <name_of_your_beat>
```

To play an specific beat, use the next command:
```shell
rosrun haru_drums_ros_driver play_beat.sh <name_of_your_beat>
```

When recording with this method, the resulting beats will have a predefined length of 20 hits per beatfile.

If you would like to have an specific number of drum hits, first source the virtual environment, then use the CLI of `beat_creator.py` script like so:
```shell
rosrun haru_drums_ros_driver beat_creator.py --help
```
output
```
usage: Beat Creator [-h] [--filename FILENAME] [--play-beat] [--num-hits NUM_HITS]

optional arguments:
  -h, --help           show this help message and exit
  --filename FILENAME  Name of new/existing beat file to play/record.
  --play-beat          Play the selected beat. WARNING: If not declared, will rewrite the file.
  --num-hits NUM_HITS  Define how many hits will be in the beat sequence. Default: 20.
```

Beat files are stored in ``src/beats`` folder.

## Request actions to the player
The `beat_comparator.py` module is in charge of requesting the player to input a specific beat or hit sequence, it will
generate a score of how accurate a player played the beat.

The scoring system depends on the game-mode you are playing which can be:
- Simon says (`simon-says`): The score is only based on color matching
- Rythm game (`rythm-game`): The score is the mean score of color matching and timing accuracy

You can test and visualize the scoring system by calling the script directly:

```shell
rosrun haru_drums_ros_driver beat_comparator.py 
```
