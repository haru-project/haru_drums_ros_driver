#!/bin/bash
# shellcheck disable=SC2046
source $(rospack find haru_drums_ros_driver)/venv/bin/activate
python $(rospack find haru_drums_ros_driver)/scripts/midi_drum_node.py
deactivate
