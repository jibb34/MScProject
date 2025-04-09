#!/bin/bash

python3 -m venv importenv
source importenv/bin/activate
python -m pip install requests gpxpy tqdm
python3 import.py
rm -rf ./importenv
