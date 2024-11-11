#!/bin/bash
set -e

# setup ros environment
#source "/root/catkin_ws/devel/setup.bash"
cd /workspace/jumping-from-pixels && python3 setup.py build_ext --inplace && pip3 install -e .

eval "bash"

exec "$@"
