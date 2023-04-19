#!/bin/bash
python3 -m venv venv_drums_drivers >/dev/null
sudo chown -R "$(whoami)" venv_drums_drivers
source venv_drums_drivers/bin/activate
pip3 install --upgrade pip
pip3 install wheel
pip install -r requirements.txt
deactivate
