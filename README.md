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
sudo ./inspire_g1
```

Then launch the nodes.

```shell
source <colcon workspace>/install/setup.bash
ros2 launch g1_bringup g1_bringup.launch.py network_interface:=<network interface name>
# If you want to move hands, run the following instead:
# ros2 launch g1_bringup g1_bringup.launch.py network_interface:=<network interface name> hand_type:=inspire_dfq
```

To activate ROS controller

```shell
ros2 control set_controller_state upper_body_controller active
```

To run hand motion demo

```shell
ros2 run g1_bringup inspire_dfx_hand_open_close_demo --ros-args -p hand:=both
```

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
