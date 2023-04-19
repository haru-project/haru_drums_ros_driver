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
import rospkg
import threading
from haru_drums_ros_driver.msg import DrumMidiSignal
from custom_sounds_module import play_sound


package_path = rospkg.RosPack().get_path("haru_drums_ros_driver")
config_path = os.path.join(package_path, "config")
settings_path = os.path.join(config_path, "drum_settings.yaml")


class MidiDrum():
    def __init__(self, custom_sounds=False):
        with open(settings_path, "r") as file:
            settings = yaml.safe_load(file)
        self.port = settings["default_port"]
        self.color_to_num_dict = settings["color_number"]
        self.num_to_color_dict = settings["number_color"]
        self.midi_signal_pub = rospy.Publisher('midi_signal/hit', DrumMidiSignal, queue_size=10)
        self.custom_sounds = custom_sounds
        self.midi_in = rtmidi.MidiIn()
        self.midi_in.open_port(self.port)
        self.midi_in.set_callback(self.handle)

    def play_sound_async(self, midi_code):
        thread = threading.Thread(target=play_sound, args=(midi_code,), daemon=True)
        thread.start()

    def handle(self, event, data):
        if event[0][2] == 127 and (event[1] > 0.15 or event[1] == 0.0):
            midi_signal = DrumMidiSignal()
            midi_signal.color = self.num_to_color_dict[event[0][1]]
            midi_signal.midi_key = event[0][1]
            midi_signal.delta_time = event[1]
            self.midi_signal_pub.publish(midi_signal)

            if self.custom_sounds:
                self.play_sound_async(event[0][1])

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
    drum = MidiDrum(custom_sounds=True)
    print(drum)
    rospy.spin()
