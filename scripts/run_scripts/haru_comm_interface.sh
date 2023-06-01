#!/bin/bash
# shellcheck disable=SC2046
source $(rospack find haru_drums_ros_driver)/venv/bin/activate
python $(rospack find haru_drums_ros_driver)/scripts/haru_comm_interface_node.py
deactivate
