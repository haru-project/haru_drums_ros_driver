#!/bin/bash
python3 -m venv venv >/dev/null
sudo chown -R "$(whoami)" venv
source venv/bin/activate
pip3 install --upgrade pip
pip3 install wheel
pip install -r requirements.txt
deactivate
