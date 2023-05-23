#!/usr/bin/env python3
import yaml
import rtmidi
import time
import os
import rospkg

package_path = rospkg.RosPack().get_path("haru_drums_ros_driver")
config_path = os.path.join(package_path, "config")
sounds_path = os.path.join(package_path, "src", "sounds")

colors = ["red", "yellow", "blue", "green", "orange", "light-blue", "purple"]


def read_drum_hit(port):
    data = {
        "number": [],
        "count": 0
    }

    def handle(et, data):
        if et[0][2] == 127:
            data["number"].append(et[0][1])
            data["count"] += 1

    midi_in = rtmidi.MidiIn()
    midi_in.open_port(port)
    midi_in.set_callback(handle, data=data)

    while data["count"] < 3:
        time.sleep(0.000000000001)
    midi_in.close_port()

    return data["number"]


def setup_midi_port():
    print("################################")
    print('#       PORT SELECTION         #')
    print('################################\n')
    print("Available midi ports:")

    available_ports = []
    port = 0
    for i, port in enumerate(rtmidi.MidiIn().get_ports()):
        print(i, "-", port)
        available_ports.append(i)

    try:
        port = int(input("Select midi port:"))
    except IndexError:
        setup_midi_port()

    return port


def setup_colors(port):
    print("################################")
    print('#    COLOR CONFIGURATION       #')
    print('################################\n')
    print("For each color press the corresponding pad three times\n")
    drum_note_dict = {}
    drum_color_dict = {}

    for color in colors:
        repeat = True
        print(f"Hit {color}")
        while repeat:
            ans = read_drum_hit(port)
            if len(set(ans)) == 1 and ans[0] not in drum_note_dict.keys():
                repeat = False
                drum_note_dict[ans[0]] = color
                drum_color_dict[color] = ans[0]
            else:
                print(f"Try again with:{color}")
    return drum_note_dict, drum_color_dict


def setup_sound_set():
    print("################################")
    print('#   SOUND EFFECT SELECTION     #')
    print('################################\n')

    print("These are the detected sound effects")

    for i, sound_set in enumerate(os.listdir(sounds_path)):
        print(i, "-", sound_set)

    try:
        sound_set = os.path.join(sounds_path, os.listdir(sounds_path)[int(input("Select a sound set:"))])
    except IndexError:
        setup_sound_set()

    sound_tracks = [os.path.join(sound_set, sound) for sound in os.listdir(sound_set)]

    return sound_tracks


def full_setup():
    port = setup_midi_port()
    number_color, color_number = setup_colors(port)
    sound_tracks = setup_sound_set()
    number_sound = dict(zip(number_color.keys(), sound_tracks))

    drum_data = {
        "default_port": port,
        "color_number": color_number,
        "number_color": number_color,
        "number_sound": number_sound
    }

    with open(os.path.join(config_path, "drum_settings.yaml"), "w") as file:
        yaml.dump(drum_data, file)
    print("Drum config file changed!")


if __name__ == '__main__':
    full_setup()
