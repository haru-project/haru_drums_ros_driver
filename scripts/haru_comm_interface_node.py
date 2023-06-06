#!/usr/bin/env python3

import rospy
import os
from haru_drums_ros_driver.msg import DrumMidiSignal
from idmind_tabletop_msgs.srv import TTSServiceRequest, TTSService
from idmind_tabletop_msgs.msg import LCDCommand
from midi_drum_node import package_path, settings

class HaruCommInterface:
    if settings["real_robot"]:
        videos_path = "/home/haru/drum_demo_videos"
    else:
        videos_path = os.path.join(package_path, "src", "eye_colors")

    video_dict = {
        "blue":     os.path.join(videos_path, "blue.mp4"),
        "green":    os.path.join(videos_path, "green.mp4"),
        "yellow":   os.path.join(videos_path, "yellow.mp4"),
        "orange":   os.path.join(videos_path, "orange.mp4"),
        "red":      os.path.join(videos_path, "red.mp4"),
        "purple":   os.path.join(videos_path, "purple.mp4"),
        "skyblue":  os.path.join(videos_path, "skyblue.mp4")
    }

    def __init__(self):
        self.drum_record_suscriber = rospy.Subscriber("/midi_signal/record", DrumMidiSignal, self.process_record)
        self.drum_hit_suscriber = rospy.Subscriber("/midi_signal/hit", DrumMidiSignal, self.process_hit)
        self.eye_video_publisher = rospy.Publisher("/idmind_tabletop/cmd_lcd", LCDCommand, queue_size=10)
    def process_record(self, msg: DrumMidiSignal):
        self.do_routine(msg.color)
        self.speak(msg.color)
    def process_hit(self, msg: DrumMidiSignal):
        self.do_routine(msg.color)
        # self.speak(msg.color)
    def speak(self, color):
        rospy.wait_for_service('/idmind_tabletop/cmd_tts_blocking_srv')
        srv_proxy = rospy.ServiceProxy('/idmind_tabletop/cmd_tts_blocking_srv', TTSService)
        request = TTSServiceRequest()
        request.message = color
        try:
            response = srv_proxy(request)
            print(response)
        except rospy.ServiceException as e:
            rospy.logerr("Error al llamar al servicio: %s" % str(e))
    def do_routine(self, color):
        lcd_cmd = LCDCommand()
        lcd_cmd.left_eye_file = self.video_dict[color]
        lcd_cmd.right_eye_file = self.video_dict[color]
        self.eye_video_publisher.publish(lcd_cmd)



if __name__ == '__main__':
    rospy.init_node("haru_drums_comm_interface")
    interface = HaruCommInterface()
    rospy.spin()