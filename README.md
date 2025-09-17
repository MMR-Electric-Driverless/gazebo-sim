<div align="center">
    <h1>Simple Sensor Simulator with Gazebo and Ros2</h1>
</div>

## :open_file_folder: What's in this repo

- **`simulator/`**
  - **`simulator_launch.py`** : Python launch file. Aggregates all .sdf/.stl models and opens the various .yaml for gazebo simulation
  - **`/filter.py`** : Python Ros2 node capable of removing excessive LiDAR rays (v_axis only) to achieve non-uniform rays distributions
- **`params.yaml`** : File with all customisable parameters see below...
- **`models/<file_name>.stl`** : Car and cones .stl models
- **`sdf/`**
    - **`world_template.sdf`** : Contains the template for the gazebo simulation world
    - **`world.sdf`** :  Contains the track for the gazebo simulation world with cones
- **`tracks/race_track.csv`** : Contains the shape of the track with coordinates

## :package: Prerequisite packages
> What we need are ros2 humble, gazebo fortress, ros-gz-bridge, colcon, pandas and numpy.
- **`Ubuntu 22.04`** [[link]](https://releases.ubuntu.com/jammy/)
- **`ros2 humble`** [[link]](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- **`gazebo fortress`** [[link]](https://gazebosim.org/docs/fortress/install/)
- **`Colcon`** [[link]](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

### Venv installation
> for virtual environments use the following two commands. Remember to use pip only inside the virtual environment!
```commandline
sudo apt-get install ros-humble-ros-gz-bridge python3-colcon-common-extensions
```
```commandline
pip install numpy pandas pyyaml catkin_pkg
```

### Systemwide installation (not recommended)
```
sudo apt-get install ros-humble-ros-gz-bridge python3-colcon-common-extensions python3-numpy python3-pandas -y
```
## :gear: How to build & Run
```commandline
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/MMR-Electric-Driverless/gazebo-sim.git
cd ~/ros2_ws
colcon build --packages-select gazebo-sim --symlink-install
source install/setup.bash
```
> if a virtual environment is used, export the path to the virtual environment, my was called .venv and was placed inside ~/ros2_ws alongside src, install, build and log.
```commandline
export PYTHONPATH=$PYTHONPATH:~/ros2_ws/.venv/lib/python3.10/site-packages
```

> To move the car click on the plugins dropdown list in the top right corner of gazebo (vertical ellipsis), select the Key Publisher.
```commandline
ros2 launch gazebo-sim simulator_launch.py
```

> A bridge between ros2 and gazebo to control the car is present and can be use instead of the Key Publisher.
```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: .5}, angular: {z: 0.5}}"
```

A pcl filter is also needed to emulate real OT128 lidar structure: 
https://github.com/MMR-Electric-Driverless/pcl-filter

![Screenshot from 2025-01-31 21-59-21](https://github.com/user-attachments/assets/343c7e04-40dd-4dab-8bff-37520b708268)

![scatter plot race track](https://github.com/user-attachments/assets/1a83dcf5-76ab-4309-8b8f-71cec086830f)

## :abacus: Parameters

The following code explains what every parameter does:

```yaml
# Topic filter
filter:
  fixed_frame: 'track'
  odom_topic: '/odometry'
  imu_topic: '/imu'   # does nothing
  pcl_topic: '/points'

# Vehicle parameters
vehicle:
  abs_pose: [-2.4, 0, 0, 0, 0, -0.7]  # Absolute position and rotation
  linear_speed_f: 0.5 # Frontal car speed
  cmd_vel: '/model/cmd_vel' # Moving input topic


# Lidar parameters
lidar:
  pose: [0.26, 0, 0.475, 0, -0.1570796, 0]  # Lidar position and rotation relative to the car
  hz: 20 # Lidar frequency
  h_samples: 500 # Number of Horizontal Rays
  h_min_angle: 2.356194   # 135° (convert from rads to degrees)
  h_max_angle: 3.926991   # 225° (convert from rads to degrees)
  h_resolution: 0.1 # Param that multiply h_samples
  v_samples: 128 # Number of Vertical Rays
  v_min_angle: -0.3926991 # -22.5° (convert from rads to degrees)
  v_max_angle: 0.3926991  # 22.5° (convert from rads to degrees)
  v_resolution: 0.1 # Param that multiply v_samples
  min_range: 0.01 # Lidar min depth range (THIS IS METERS!)
  max_range: 80.0 # Lidar max depth range (THIS IS METERS!)
  range_resolution: 0.01 # Param that multiply min_range and max_range
  noise_mean: 0.0 # Mean of the gaussian noise applied to Lidar
  noise_std: 0.05 # Std. Dev. of the gaussian noise applied to Lidar

# Cones parameters
cones:
  min_dist: 1.5 # Minimum distance between cones (THIS IS NOT METERS!)
  scale: 10 # Map coordinates scale
  track_width: 3 # Width of the track (THIS IS NOT METERS!)
```
## Todo List

- [ ]  Add coloerd cones with stripes to improve color and contrast reading with lidar (probably a light implementation within gazebo is required)
- [ ]  Fine-Tuning lidar settings
- [ ]  Implement filter presets to achieve faster and smoother operations
- [ ]  Implement a more accurate steering mode
- [ ]  Add ros2 movement control
- [ ]  Apply the true model for the car
- [ ]  Improve performance

🏎️ Made by [**Leonardo Nels**](https://github.com/leonardonels) - 2025
