#!/usr/bin/env python3
import rospy
import yaml
import os
import rospkg
import argparse
from haru_drums_ros_driver.msg import DrumMidiSignal

package_path = rospkg.RosPack().get_path("haru_drums_ros_driver")
beats_path = os.path.join(package_path, "src", "beats")


class BeatCreator():
    def __init__(self, filename, play, limit=5):
        self.yaml_file = os.path.join(beats_path, filename) + ".yaml"
        if play:
            self.beat_publisher = rospy.Publisher('midi_signal/record', DrumMidiSignal, queue_size=10)
            self.play_beat()
        else:
            self.hit_suscriber = rospy.Subscriber('midi_signal/hit', DrumMidiSignal, self.record_callback)
            self.beat = {"hit_sequence": []}
            self.limit = limit + 1
            self.save_data(self.beat)

    def load_data(self):
        with open(self.yaml_file, "r") as f:
            return yaml.safe_load(f)

    def save_data(self, dato):
        self.limit -= 1
        with open(self.yaml_file, "w") as f:
            yaml.safe_dump(dato, f)

    def record_callback(self, msg: DrumMidiSignal):
        self.beat = self.load_data()
        if self.beat["hit_sequence"] == []:
            self.beat["hit_sequence"].append({
                "color": msg.color,
                "midi_key": msg.midi_key,
                "delta_time": 0.16
            })
        else:
            self.beat["hit_sequence"].append({
                "color": msg.color,
                "midi_key": msg.midi_key,
                "delta_time": msg.delta_time
            })

        self.save_data(self.beat)

        rospy.loginfo(f"RECORD {msg.color}")

        if self.limit == 0:
            rospy.signal_shutdown(reason="Beat recorded")

    def play_beat(self):
        self.beat = self.load_data()
        for beat in self.beat["hit_sequence"]:
            signal_msg = DrumMidiSignal()
            signal_msg.color = beat["color"]
            signal_msg.midi_key = beat["midi_key"]
            signal_msg.delta_time = beat["delta_time"]
            rospy.sleep(signal_msg.delta_time)
            rospy.loginfo(f"PLAY {signal_msg.color}")
            self.beat_publisher.publish(signal_msg)
        rospy.signal_shutdown("Node ended his purpouse")


if __name__ == '__main__':
    parser = argparse.ArgumentParser("Beat Creator")
    parser.add_argument("--filename", default="", type=str, help="Name of the new beat sequence")
    parser.add_argument("--play-beat", action="store_true", help="Play the recorded beat")
    args = parser.parse_args()
    rospy.init_node("beat_creator")
    beat = BeatCreator(args.filename, args.play_beat)
    rospy.spin()
