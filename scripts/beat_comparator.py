#!/usr/bin/env python3
import os
import yaml
import rospy
import rospkg
import numpy as np
import multiprocessing
import matplotlib.pyplot as plt
from beat_creator import BeatCreator

package_path = rospkg.RosPack().get_path("haru_drums_ros_driver")
beats_path = os.path.join(package_path, "src", "beats")


def beat_path(string):
    """
    Wrapper for beat_file path
    :param string:
    :return:
    """
    return os.path.join(beats_path, string + ".yaml")

def play_record_factory(beat_filename, play=False):
    """
    Factory of BeatCreator objects.
    Can create objects that are either players or recorders.

    :param beat_filename: Name of the beat to play/replicate
    :param play: If set to True, will play the beat, otherwise will record a beatfile for later comparison
    :return:
    """
    with open(beat_path(beat_filename), "r") as file:
        target = yaml.safe_load(file)

    num_hits = len(target["hit_sequence"])
    rospy.init_node("beat_playing")

    if play:
        BeatCreator(beat_filename, play)
    else:
        BeatCreator(beat_filename + "_user", play, num_hits)
    rospy.spin()


def play_and_request_beat(beat_filename="ejemplo"):
    """
    Plays a beat, then request the player the second beat
    :param beat_filename:
    :return:
    """
    print("HARU'S TURN")
    haru_turn(beat_filename)
    print("PLAYER'S TURN")
    user_turn(beat_filename)

def haru_turn(filename):
    """
    Haru's turn will always play the beat
    :param filename:
    :return:
    """
    record_process = multiprocessing.Process(target=play_record_factory, args=(filename, True))
    record_process.start()
    record_process.join()

def user_turn(filename):
    """
    User's turn will always request a beat
    :param filename:
    :return:
    """
    record_process = multiprocessing.Process(target=play_record_factory, args=(filename, False))
    record_process.start()
    record_process.join()

def compare_results(beat_filename="ejemplo", plot_eval=False):
    """
    Opens both files and compares time and color matching exactitude
    returns both scores
    :param beat_filename:
    :param plot_eval:
    :return:
    """
    with open(beat_path(beat_filename), "r") as file:
        target = yaml.safe_load(file)

    with open(beat_path(beat_filename + "_user"), "r") as file:
        response = yaml.safe_load(file)

    colors = []
    delta_times = []

    for elem in zip(target["hit_sequence"], response["hit_sequence"]):
        target, response = elem
        colors.append([target["color"], response["color"]])
        delta_times.append([target["delta_time"], response["delta_time"]])

    colors = np.array(colors)
    delta_times = np.array(delta_times)

    color_diff = colors[:, 0] == colors[:, 1]
    time_diff = abs(delta_times[:, 0] - delta_times[:, 1])

    color_precision = color_score(color_diff)
    timing_precision = time_score(time_diff)

    if plot_eval:
        plot_results(colors, delta_times, color_precision, timing_precision)

    return color_precision, timing_precision


def time_score(time_diffs):
    """
    Calculates the timing precision score
    :param time_diffs:
    :return:
    """
    res = 1 / np.mean(time_diffs)
    if res >= 10:
        return 10.0
    else:
        return round(res, 2)


def color_score(color_diffs):
    """
    Calculates color precision score
    :param color_diffs:
    :return:
    """
    num_hits = len(color_diffs)
    return (np.sum(color_diffs) * 10) / num_hits


def plot_results(colors, delta_times, score_color, score_time):
    """
    Auxiliar function to visualize how exact was the players repetition
    :param colors:
    :param delta_times:
    :param score_color:
    :param score_time:
    :return:
    """
    plt.title("User evaluation")
    plt.gca().set_facecolor("darkgrey")
    for i, label in enumerate([["Beat to replicate", "mediumblue"], ["User beat", "lime"]]):
        for j in range(colors.shape[0]):
            plt.scatter(j, delta_times[j, i], c=colors[j, i], s=100, zorder=10)
        plt.plot(np.arange(0, colors.shape[0]), delta_times[:, i], label=label[0], c=label[1])
    plt.legend()
    plt.ylabel("Time elapsed since last hit")
    plt.xlabel("Hit count")
    plt.figtext(0.75, 0.95, f"Color score: {score_color} / 10\n"
                            f" Time score: {score_time}  / 10",
                ha='left', va='center', bbox=dict(facecolor='white', edgecolor='black'))
    plt.show()


def request_drum_action_to_player(beat_filename, mode, plot_eval):
    """
    Process of requesting a player to participate in a game-mode round.
    Deletes the auxiliar beat_file created during the process
    :param beat_filename:
    :param mode:
    :param plot_eval:
    :return:
    """
    play_and_request_beat(beat_filename)
    color_precision, timing_precision = compare_results(beat_filename, plot_eval=plot_eval)
    os.remove(beat_path(beat_filename + "_user"))

    match mode:
        case "simon-says":
            return color_precision
        case "rythm-game":
            return np.mean([color_precision, timing_precision])


if __name__ == '__main__':
    print("PLAYING A SIMON SAYS ROUND")
    print(request_drum_action_to_player('ejemplo', "simon-says", True))
    print("PLAYING A RYTHM GAME ROUND")
    print(request_drum_action_to_player('ejemplo', "rythm-game", True))
