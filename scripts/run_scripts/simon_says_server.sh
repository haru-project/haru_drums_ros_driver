#!/bin/bash
# shellcheck disable=SC2046
source $(rospack find haru_drums_ros_driver)/venv/bin/activate
python3.10 $(rospack find haru_drums_ros_driver)/scripts/simon_says_server.py
deactivate