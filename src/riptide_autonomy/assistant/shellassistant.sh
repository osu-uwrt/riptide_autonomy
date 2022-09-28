#! /bin/bash

#
# Shell assistant to the Python BehaviorTree assistant.
# This script pretty much exists to do things in shell that would be more annoying / impossible to do in python.
#

if [ "$1" == "rebuild_autonomy" ]; then # REBUILD AUTONOMY
    cd ~/osu-uwrt/riptide_software
    rm -rf build/riptide_autonomy2
    rm -rf install/riptide_autonomy2
    colcon build --packages-select riptide_autonomy2
fi
