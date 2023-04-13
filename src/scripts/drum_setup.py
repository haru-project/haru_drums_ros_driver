#!/usr/bin/env python3
import yaml
import json
import rtmidi
import time
import os
import rospkg

package_path = rospkg.RosPack().get_path("haru_drums_ros_driver")
config_path = os.path.join(package_path, "config")

colors = ["red", "yellow", "blue",
          "green", "orange", "light-blue", "purple"]


def read_drum_hit(port):
    data = {
        "number": [],
        "count": 0
    }

    def handle(event, data):
        if event[0][2] == 127:
            data["number"].append(event[0][1])
            data["count"] += 1

    midi_in = rtmidi.MidiIn()
    midi_in.open_port(port)
    midi_in.set_callback(handle, data=data)

    while data["count"] < 3:
        time.sleep(0.000000000001)
    midi_in.close_port()

    return data["number"]


print('Available ports')
available_ports = []
for i, port in enumerate(rtmidi.MidiIn().get_ports()):
    print(i, "-", port)
    available_ports.append(i)

okay = False
while not okay:
    port = int(input("Select midi port:"))
    if port in available_ports:
        okay = True

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

drum_data = {
    "default_port": port,
    "color_number": drum_color_dict,
    "number_color": drum_note_dict
}

with open(os.path.join(config_path, "drum_settings.yaml"), "w") as file:
    yaml.dump(drum_data, file)

with open(os.path.join(config_path, "drum_settings.json"), "w") as file:
    json.dump(drum_data, file, indent=2)

print("Drum config file changed!")
