#!/bin/bash

ROOT_DIR="$HOME/Formula"

# Check if "--use-cpu-clustering" is passed as an argument
USE_CPU_CLUSTERING=false
for arg in "$@"; do
    if [ "$arg" == "--use-cpu-clustering" ]; then
        USE_CPU_CLUSTERING=true
        break
    fi
done

# Source files from orin-drive and gazebo-sim
source $ROOT_DIR/orin-drive/install/setup.sh
source $ROOT_DIR/gazebo-sim/install/setup.sh
source $ROOT_DIR/pcl-filter/install/setup.sh

# Start a new tmux session and run the commands in separate windows
tmux new-session -d -s simulation "ros2 launch gazebo-sim simulator_launch.py"
tmux split-window -v -t simulation "ros2 launch pcl-filter filter_launch.py"
if [ "$USE_CPU_CLUSTERING" == true ]; then
    tmux split-window -h -t simulation "ros2 launch clustering_plane_finder_cpu clustering_plane_finder_cpu_launch.py"
fi
tmux split-window -v -t simulation "ros2 launch mmr_ekf_odometry simulation.launch.py"
tmux split-window -h -t simulation "ros2 launch local_planner local_planner.launch.py"
tmux split-window -h -t simulation "ros2 launch lap_counter lap_counter.launch.py"

tmux select-layout -t simulation tiled

tmux attach-session -t simulation

tmux wait-for -S detach
tmux kill-session -t simulation
pkill ruby
