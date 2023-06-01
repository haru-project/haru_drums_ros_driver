#!/usr/bin/env python3
"""
Accepts service requests to challenge the player
"""

import yaml
import rospy
import random
from midi_drum_node import settings
from haru_drums_ros_driver.srv import SimonSays, SimonSaysResponse
from beat_comparator import request_drum_action_to_player, beat_path

simon_says_beat = "simon_says"

def get_random_hit():
    number = random.choice(list(settings["number_color"].keys()))
    color = settings["number_color"][number]
    return {"color": color, "delta_time": 1, "midi_key": number}

def update_beat():
    with open(beat_path(simon_says_beat), "r") as file:
        simon_file = yaml.safe_load(file)
    hit = get_random_hit()
    simon_file['hit_sequence'].append(hit)
    with open(beat_path(simon_says_beat), "w") as file:
        yaml.dump(simon_file, file)

def reset_game():
    data = {'hit_sequence': []}
    with open(beat_path(simon_says_beat), "w") as file:
        yaml.dump(data, file)



def gameplay_block(req):
    score = request_drum_action_to_player(beat_filename=simon_says_beat, mode="simon-says", plot_eval=False)
    if score == 10:
        update_beat()
        return SimonSaysResponse(True, score)
    else:
        if score < 5:
            reset_game()
            return SimonSaysResponse(False, score)
        else:
            return SimonSaysResponse(True, score)

def simon_says_server():
    rospy.init_node("simon_says_service")
    service = rospy.Service("simon_says_service", SimonSays, gameplay_block)
    rospy.spin()

if __name__ == '__main__':
    reset_game()
    update_beat()
    simon_says_server()