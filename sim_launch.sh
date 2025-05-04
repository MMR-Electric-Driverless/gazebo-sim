#!/bin/bash

# Source files from mmr-drive, orin-drive, and gazebo-sim
source ~/Formula/mmr-drive/install/setup.sh
source ~/Formula/orin-drive/install/setup.sh
source ~/Formula/gazebo-sim/install/setup.sh
source ~/Formula/pcl-filter/install/setup.sh

# Start a new tmux session and run the commands in separate windows
tmux new-session -d -s simulation "ros2 launch gazebo-sim simulator_launch.py"
tmux split-window -v -t simulation "ros2 launch pcl-filter filter_launch.py"
tmux split-window -h -t simulation "ros2 launch clustering_plane_finder_cpu clustering_plane_finder_cpu_launch.py"
tmux split-window -v -t simulation "ros2 launch marker_adapter marker_adapter.launch.py"
tmux split-window -h -t simulation "ros2 launch mmr_ekf_odometry simulation.launch.py"
tmux select-layout -t simulation tiled

tmux attach-session -t simulation

tmux wait-for -S detach
tmux kill-session -t simulation
pkill ruby
