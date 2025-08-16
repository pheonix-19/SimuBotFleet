#!/bin/bash
# Wrapper script to run Gazebo GUI without snap library conflicts

# Remove snap library paths from LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | tr ':' '\n' | grep -v snap | tr '\n' ':' | sed 's/:$//')

# Prepend system library paths
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"

# Clear any snap-related environment variables
unset SNAP
unset SNAP_COMMON
unset SNAP_DATA
unset SNAP_USER_COMMON
unset SNAP_USER_DATA

# Run Gazebo with the cleaned environment
exec /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz "$@"
