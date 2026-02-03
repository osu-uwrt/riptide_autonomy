#!/usr/bin/env bash


set -e
set -o pipefail

WORKSPACE_DIR=$(pwd)


echo "Workspace Dir: $WORKSPACE_DIR"
echo ""


apt update -qq
apt install -y \
    libboost-log-dev \
    python3-colcon-common-extensions \
    python3-rosdep


if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
   
    rosdep init
fi
rosdep update



rosdep install --from-paths . --ignore-src -r -y || true



colcon build \
    --cmake-force-configure \
    --base-paths . \
    --install-base INSTALL_BASE \
    --event-handlers console_direct+




source INSTALL_BASE/setup.bash


set +e
colcon test \
    --return-code-on-test-failure \
    --base-paths . \
    --install-base INSTALL_BASE \
    --event-handlers console_direct+

TEST_STATUS=$?
set -e


if [[ $TEST_STATUS != 0 ]]; then
  
    colcon test-result --test-result-base INSTALL_BASE --all --verbose
    exit $TEST_STATUS
fi


exit 0