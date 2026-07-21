# Local WSL Ubuntu 20.04 setup

This repository is a ROS 1 Noetic catkin workspace for Gazebo Classic.  Run
Gazebo and its ROS nodes on the local WSL Ubuntu 20.04 system.  The only ROS
Master for this workflow is `http://192.168.8.197:11311`; never start or use a
ROS Master on the vehicle address `192.168.8.231:11311`.

## Install and build

Clone into the WSL Linux filesystem rather than `/mnt/d`, which avoids slow or
blocked CMake and Gazebo I/O on the Windows-mounted filesystem.

Before building, use a Linux-only `PATH`.  This prevents CMake from probing
Windows-mounted executables through WSL's `9p` filesystem, which can otherwise
make configuration appear to hang.

```bash
sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  g++ dos2unix libopencv-dev python3-opencv \
  ros-noetic-gazebo-ros ros-noetic-gazebo-plugins \
  ros-noetic-gazebo-ros-control ros-noetic-navigation \
  ros-noetic-map-server ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher-gui ros-noetic-joint-state-controller \
  ros-noetic-joint-trajectory-controller ros-noetic-position-controllers ros-noetic-cv-bridge \
  ros-noetic-control-toolbox

git clone https://github.com/MDSIXONE/smartcar2026-simulation.git \
  ~/smartcar2026-simulation
cd ~/smartcar2026-simulation
find src -type f -name '*.py' -exec dos2unix {} \;
find src/car3/scripts -type f -name '*.py' -exec chmod +x {} \;
chmod +x start_v3_clean.sh

export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
export ROS_IP=192.168.8.197
export DISABLE_ROS1_EOL_WARNINGS=1
catkin_make -j2
```

## Install the local YOLOv3-tiny runtime

The detector intentionally keeps the model runtime outside this Git repository.
It only copies the Darknet sources required for inference, the three-class
configuration, class names, and the single `best` checkpoint; it does not copy
training images or intermediate checkpoints. The command below builds the
shared library and persistent `yolo_pipe` for this WSL distribution.

```bash
darknet_source=/mnt/d/WORK/ALLCODE/smartcar2026/darknet_64efa721
darknet_runtime=$HOME/smartcar2026-models/darknet-yolov3

test -f "$darknet_source/cfg/yolov3-tiny-3cls.cfg"
test -f "$darknet_source/data/obj.names"
test -f "$darknet_source/backup/yolov3-tiny-3cls_best.weights"
test -f "$darknet_source/wrapper/yolo_pipe.cpp"

mkdir -p "$darknet_runtime"
tar -C "$darknet_source" -cf - Makefile build.sh include src 3rdparty wrapper | \
  tar -C "$darknet_runtime" -xf -
install -D -m 0644 "$darknet_source/cfg/yolov3-tiny-3cls.cfg" \
  "$darknet_runtime/cfg/yolov3-tiny-3cls.cfg"
install -D -m 0644 "$darknet_source/data/obj.names" \
  "$darknet_runtime/data/obj.names"
install -D -m 0644 "$darknet_source/backup/yolov3-tiny-3cls_best.weights" \
  "$darknet_runtime/models/yolov3-tiny-3cls_best.weights"

cd "$darknet_runtime"
make -j2 GPU=0 CUDNN=0 OPENCV=0 LIBSO=1
(cd wrapper && bash build.sh)
test -x "$darknet_runtime/yolo_pipe"
test -f "$darknet_runtime/libdarknet.so"
```

The CPU build is deliberate for a portable first integration. If WSL GPU
passthrough and a matching CUDA toolkit are available, rebuild this same
runtime with the appropriate `GPU`, `CUDNN`, and `ARCH` settings before raising
the detector rate.

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

## Run the camera and YOLOv3 during a complete task

Start the preparation scene with the detector enabled. This only subscribes to
the RGB camera and publishes JSON detections on `/sim_task3/yolo/detections`;
it does not issue base or arm commands. As with every ROS terminal, restore
the required WSL Master after every `source` command.

```bash
cd ~/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
source devel/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
export ROS_IP=192.168.8.197
export DISABLE_ROS1_EOL_WARNINGS=1
roslaunch car3 task3_prepare.launch gui:=true rviz:=true \
  enable_yolo:=true darknet_dir:=$HOME/smartcar2026-models/darknet-yolov3
```

Before starting a task, verify that the detector is ready and receives camera
frames. These checks are read-only:

```bash
rostopic echo -n 1 /sim_task3/yolo/status
rostopic echo -n 1 /sim_task3/yolo/detections
python3 - <<'PY'
import sys
import time

import rospy
import tf2_ros

rospy.init_node("task3_tf_check", anonymous=True)
buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buffer)
time.sleep(2.0)  # Wall time: works while Gazebo uses /clock.
for parent, child in (("odom", "base_link"), ("map", "base_link")):
    if not buffer.can_transform(parent, child, rospy.Time(0)):
        raise SystemExit("missing TF: %s -> %s" % (parent, child))
print("odom -> base_link and map -> base_link are available")
PY
```

Only after the TF checks are normal may a full task be sent in another terminal:

```bash
cd ~/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
source devel/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
export ROS_IP=192.168.8.197
export DISABLE_ROS1_EOL_WARNINGS=1
roslaunch car3 task3_execute.launch cargo_item:="苹果"
```

The current task controller continues to use its verified Gazebo model/pose
logic for pickup; YOLOv3 runs in parallel for camera and detection validation.
If any future vehicle-side log shows `/odom_raw` or `wheelodom` as `NaN`, or
reports `TF_NAN_INPUT`, publish zero velocity and restart the navigation and
odometry chain before continuing. Do not start a local vehicle `roscore`.
