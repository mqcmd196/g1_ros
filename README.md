# The ROS driver for Unitree Robotics G1 Humanoid

## Setup

Please install ROS 2 and some development tools like rosdep, vcs2l at first.

Then build the packages.

```shell
source /opt/ros/<ROS DISTRO>/setup.bash
mkdir -p <colcon workspace>/src
cd <colcon workspace>/src
wget https://raw.githubusercontent.com/mqcmd196/g1_ros/refs/heads/master/jazzy.repos.yaml -O- | vcs import
sudo apt update && rosdep update && rosdep install -iqry --from-paths .
cd ..
colcon build --symlink-install --packages-up-to g1_bringup
```

## Execute

Please connect your computer to the robot following [official instruction](https://support.unitree.com/home/en/G1_developer/quick_development#heading-7). Please check the network interface name.

If the robot has Inspire RH56DFX hands and you want to move them, run the hand service:
```shell
ssh unitree@192.168.123.164  # Information required for ssh (IP address, user name, password): https://support.unitree.com/home/en/G1_developer/about_G1#heading-6
cd ~/dfx_inspire_service/build
sudo ./inspire_g1 -k -u  # This script must continue to run in the background. Required branch: https://github.com/pazeshun/dfx_inspire_service/tree/pazeshun-devel (https://github.com/unitreerobotics/dfx_inspire_service/pull/4 + https://github.com/unitreerobotics/dfx_inspire_service/pull/5
```

Then launch the nodes.

```shell
source <colcon workspace>/install/setup.bash
ros2 launch g1_bringup g1_bringup.launch.py network_interface:=<network interface name>
```

If you want to move Inspire RH56DFX hands, run the following instead:

```shell
ros2 launch g1_bringup g1_bringup.launch.py network_interface:=<network interface name> hand_type:=inspire_dfq
```

By default, the hands close when their hardware interface is deactivated.
To disable this behavior, run the following instead:

```shell
ros2 launch g1_bringup g1_bringup.launch.py network_interface:=<network interface name> hand_type:=inspire_dfq close_hand_on_deactivate:=false
```

To activate ROS controller

```shell
ros2 control set_controller_state upper_body_controller active
```

To run hand motion demo

```shell
ros2 run g1_bringup inspire_dfx_hand_open_close_demo --ros-args -p hand:=both
```

## gear_sonic (SONIC) locomotion

`g1_hardware` provides `gear_sonic_interface`, a bridge that sends locomotion, VR 3-point upper-body and SMPL-motion commands to the [GR00T-WholeBodyControl](https://github.com/NVlabs/GR00T-WholeBodyControl) (SONIC) deploy stack over ZMQ. Enable it with `use_gear_sonic:=true` on `hardware.launch.py` or `g1_bringup.launch.py`.

`zmq_host` is the address the interface **binds** its ZMQ XPUB socket to; the deploy stack connects to it. The default `127.0.0.1` only accepts a deploy stack running on the same machine (simulation). For the real robot, set `zmq_host:=0.0.0.0`.

### Simulation (MuJoCo)

The MuJoCo simulator of GR00T-WholeBodyControl publishes the same DDS topics as the real robot (`rt/lowstate` etc.) on the loopback interface, so the ros2_control stack (`/joint_states`, TF) works unmodified with `network_interface:=lo`.

```shell
# Terminal 1: MuJoCo simulator (in GR00T-WholeBodyControl, .venv_sim)
python gear_sonic/scripts/run_sim_loop.py
# Terminal 2: SONIC controller (in gear_sonic_deploy)
./deploy.sh --input-type zmq_manager sim
# Terminal 3: ROS driver
ros2 launch g1_hardware hardware.launch.py network_interface:=lo use_gear_sonic:=true
```

### Real robot

```shell
# On this computer
ros2 launch g1_bringup g1_bringup.launch.py network_interface:=<network interface name> hand_type:=inspire_dfq use_gear_sonic:=true zmq_host:=0.0.0.0
# On the robot (in gear_sonic_deploy)
./deploy.sh --input-type zmq_manager --zmq-host <this computer's IP> real
```

### Commanding the robot

Once the deploy stack has connected (see the `gear_sonic_interface` log), enable control and send velocity commands:

```shell
ros2 service call /gear_sonic_interface/enable_control std_srvs/srv/SetBool "{data: true}"
ros2 topic pub /gear_sonic_interface/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" -r 10
```

Locomotion modes (`~/mode/*` services), VR 3-point upper-body targets and full-body SMPL streaming are also available. See the header comment of `g1_hardware/include/g1_hardware/gear_sonic_interface.hpp` for the full interface.

> [!WARNING]
> Do **not** activate `upper_body_controller` while SONIC is in control — its arm commands conflict with the whole-body policy. The Inspire hand controllers use separate DDS topics and can be used together with SONIC.

## Contribution

### Prerequisite

```shell
sudo apt install black isort libxml2-utils pre-commit
```

### Pre-commit

Make sure `pre-commit` is run before commit.

If you want to execute `pre-commit` automatically in every `git commit` :

```shell
cd g1_ros
pre-commit install
```

If you want to execute `pre-commit` manually:

```shell
pre-commit run --all-files
```
