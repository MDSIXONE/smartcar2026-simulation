# Local WSL Ubuntu 20.04 setup

This repository is a ROS 1 Noetic catkin workspace for Gazebo Classic.  Run
Gazebo and its ROS nodes on the local WSL Ubuntu 20.04 system.  The only ROS
Master for this workflow is `http://192.168.8.197:11311`; never start or use a
ROS Master on the vehicle address `192.168.8.231:11311`.

## Install and build

Clone into the WSL Linux filesystem rather than `/mnt/d`, which avoids slow or
blocked CMake and Gazebo I/O on the Windows-mounted filesystem.

```bash
sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  g++ dos2unix libopencv-dev \
  ros-noetic-gazebo-ros ros-noetic-gazebo-plugins \
  ros-noetic-gazebo-ros-control ros-noetic-navigation \
  ros-noetic-map-server ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher-gui ros-noetic-joint-state-controller \
  ros-noetic-joint-trajectory-controller ros-noetic-position-controllers \
  ros-noetic-control-toolbox

git clone https://github.com/MDSIXONE/smartcar2026-simulation.git \
  ~/smartcar2026-simulation
cd ~/smartcar2026-simulation
find src -type f -name '*.py' -exec dos2unix {} \;
find src/car3/scripts -type f -name '*.py' -exec chmod +x {} \;
chmod +x start_sim_clean.sh

source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
export ROS_IP=192.168.8.197
export DISABLE_ROS1_EOL_WARNINGS=1
catkin_make -j2
```

## Start the local Master

In a dedicated WSL terminal, start the Master at the required address.  When
the computer is not currently assigned `192.168.8.197`, add it only as a local
loopback alias for this simulation session.  Do not add this alias while the
computer already owns that address on the network.

```bash
sudo ip address add 192.168.8.197/32 dev lo 2>/dev/null || true
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
export ROS_IP=192.168.8.197
export DISABLE_ROS1_EOL_WARNINGS=1
roscore
```

## Start and verify the preparation scene

In another WSL terminal, source both environments, then explicitly restore the
same Master URI after every `source` command:

```bash
cd ~/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
source devel/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
export ROS_IP=192.168.8.197
export DISABLE_ROS1_EOL_WARNINGS=1
roslaunch car3 task3_prepare.launch gui:=true rviz:=true
```

Use `gui:=false rviz:=false` for a non-graphical check.  The launch does not
send a navigation goal or start the pickup-and-delivery task.

Run the portable-asset regression checks from the repository root:

```bash
python3 src/car3/test/test_cube_mesh_uri.py
python3 src/car3/test/test_sign_mesh_uri.py
```

After Gazebo starts, visually confirm that all three cubes and the Food,
Daily Necessities, and Electronics wall labels are visible.  The labels depend
on the active camera angle and zoom.  If a model entity exists but its mesh is
not visible, verify that the launch puts `$(find car3)/models` in
`GAZEBO_MODEL_PATH` before `gzserver` is started and that no mesh URI contains
a machine-specific `file:///...` path.
