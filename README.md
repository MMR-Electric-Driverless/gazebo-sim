<div align="center">
    <h1>Simple Lidar Simulator</h1>
</div>

## :open_file_folder: What's in this repo

* Python launch file
* Yaml params file with all customisable parameters
* Models folder for car and cones .stl models
* Track folder with .csv file with the inside line of a test track (no mid-line)

## :package: Prerequisite packages
> What we need are ros2 humble, gazebo fortress, ros-gz-bridge, pandas and numpy.

```commandline
sudo apt-get install ros-humble-ros-gz-bridge python3-numpy python3-pandas -y
```
## :gear: How to build & Run
```commandline
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/leonardonels/LiDAR_simulator_project.git
cd ~/ros2_ws
colcon build --packages-select LiDAR_simulator_project --symlink-install
source install/setup.bash
```
> To move the car click on the plugins dropdown list in the top right corner of gazebo (vertical ellipsis), select the Key Publisher.
```commandline
ros2 launch LiDAR_simulator_project simulator_launch.py
```
![Screenshot from 2025-01-31 21-59-21](https://github.com/user-attachments/assets/343c7e04-40dd-4dab-8bff-37520b708268)

![scatter plot race track](https://github.com/user-attachments/assets/1a83dcf5-76ab-4309-8b8f-71cec086830f)

## Todo List

- [ ]  Fine-Tuning lidar settings
- [ ]  implement filter presets to achieve faster and smoother operations
- [ ]  Implement a more accurate steering mode
- [ ]  Add ros2 movement control
- [ ]  Apply the true model for the car
- [ ]  Improve performance