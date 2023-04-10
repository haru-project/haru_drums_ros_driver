#!/usr/bin/env python3
"""
This module implements the midi_drums node.
This node will publish and execute all actions related
to the interaction with
"""
import os
import rtmidi
import rospy
import yaml
from std_msgs.msg import String
import rospkg

package_path = rospkg.RosPack().get_path("haru_drums_ros_driver")
config_path = os.path.join(package_path, "config")
settings_path = os.path.join(config_path, "drum_settings.yaml")


class MidiDrum():
    def __init__(self):
        with open(settings_path, "r") as file:
            settings = yaml.safe_load(file)
        self.port = settings["default_port"]
        self.color_to_num_dict = settings["color_number"]
        self.num_to_color_dict = settings["number_color"]
        self.color_pub = rospy.Publisher('midi_drum/hit', String, queue_size=10)

        self.midi_in = rtmidi.MidiIn()
        self.midi_in.open_port(self.port)
        self.midi_in.set_callback(self.handle)

    def handle(self, event, data):
        if event[0][2] == 127 and (event[1] > 0.2 or event[1] == 0.0):
            s = String()
            s.data = self.num_to_color_dict[event[0][1]]
            self.color_pub.publish(s)

    def shutdown(self):
        self.midi_in.close_port()

    def __str__(self):
        s = f"Midi Drum\n-------------------------------------\nListening through port: {self.port}\n-------------------------------------\n" \
            f"Color dictionary:\n\n"
        for key, value in self.num_to_color_dict.items():
            s += f"{key} <-----> {value}\n"
        return s


if __name__ == '__main__':
    rospy.init_node("midi_drum")
    drum = MidiDrum()
    print(drum)
    rospy.spin()
