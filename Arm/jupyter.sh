#!/bin/sh

source /home/jetson/dofbot_ws/devel/setup.bash
/home/jetson/.local/bin/jupyter lab --port 8888 --ip=0.0.0.0
