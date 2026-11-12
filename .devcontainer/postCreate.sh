#!/usr/bin/env bash
set -e

WORKSPACE_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
cd "$WORKSPACE_DIR"
source /opt/ros/melodic/setup.bash

# Melodic is excluded from rosdep's default update because it is end-of-life.
rosdep update --rosdistro melodic --include-eol-distros
# pybind11 is installed by Dockerfile.dev; ros_pytest installs python-pytest.
# These legacy keys no longer have definitions in the current rosdep index.
rosdep install --from-paths src --ignore-src -r -y --skip-keys "pybind11 python-pytest"
catkin_make

SETUP_COMMAND="source \"$WORKSPACE_DIR/devel/setup.bash\""
if ! grep -Fxq "$SETUP_COMMAND" /root/.bashrc; then
    printf '\n%s\n' "$SETUP_COMMAND" >> /root/.bashrc
fi

