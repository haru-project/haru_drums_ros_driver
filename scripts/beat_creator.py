#!/usr/bin/env python3
import rospy
from haru_drums_ros_driver.msg import DrumMidiSignal

class BeatCreator():
    def __init__(self):
        self.hit_suscriber = rospy.Subscriber('midi_signal/hit', DrumMidiSignal, self.callback)
        self.beat = []

    def callback(self, msg: DrumMidiSignal):
        self.beat.append({
            "color": msg.color,
            "midi_key": msg.midi_key,
            "delta_time": msg.delta_time
        })

if __name__ == '__main__':
    rospy.init_node("beat_creator")
    beat = BeatCreator()
    rospy.spin()