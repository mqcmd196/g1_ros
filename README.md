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

Then launch the nodes.

```shell
source <colcon workspace>/install/setup.bash
ros2 launch g1_bringup g1_bringup.launch.py network_interface:=<network interface name>
```

To activate ROS controller

```shell
ros2 control set_controller_state upper_body_controller active
```

## Contribution

Make sure `pre-commit` is run before commit.

If you want to execute `pre-commit` automatically:

```shell
cd g1_ros
pre-commit install
```

If you want to execute `pre-commit` manually:

```shell
pre-commit run --all-files
```
