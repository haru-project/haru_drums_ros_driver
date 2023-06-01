#!/bin/bash
python3.10 -m venv venv
source venv/bin/activate
pip3 install --upgrade pip
pip3 install wheel
pip install -r requirements.txt
deactivate
