#!/bin/bash -eu

# The BSD License
# Copyright (c) 2018 PickNik Consulting
# Copyright (c) 2014 OROCA and ROS Korea Users Group

#set -x

function usage {
    # Print out usage of this script.
    echo >&2 "usage: $0 [ROS distro] (default: noetic)"
    echo >&2 "          [-h|--help] Print help message."
    exit 0
}

# Parse command line. If the number of argument differs from what is expected, call `usage` function.
OPT=`getopt -o h -l help -- $*`
if [ $# != 1 ]; then
    usage
fi
eval set -- $OPT
while [ -n "$1" ] ; do
    case $1 in
        -h|--help) usage ;;
        --) shift; break;;
        *) echo "Unknown option($1)"; usage;;
    esac
done

ROS_DISTRO=$1
ROS_DISTRO=${ROS_DISTRO:="noetic"}

version=`lsb_release -sc`
echo ""
echo "INSTALLING ROS USING quick_ros_install --------------------------------"
echo ""
echo "Checking the Ubuntu version"
case $version in
  "saucy" | "trusty" | "vivid" | "wily" | "xenial" | "bionic" | "focal")
  ;;
  *)
    echo "ERROR: This script will only work on Ubuntu Saucy(13.10) / Trusty(14.04) / Vivid / Wily / Xenial / Bionic / Focal. Exit."
    exit 0
esac

relesenum=`grep DISTRIB_DESCRIPTION /etc/*-release | awk -F 'Ubuntu ' '{print $2}' | awk -F ' LTS' '{print $1}'`
if [ "$relesenum" = "14.04.2" ]
then
  echo "Your ubuntu version is $relesenum"
  echo "Intstall the libgl1-mesa-dev-lts-utopic package to solve the dependency issues for the ROS installation specifically on $relesenum"
  sudo apt-get install -y libgl1-mesa-dev-lts-utopic
else
  echo "Your ubuntu version is $relesenum"
fi
sudo apt install curl -y
echo "Add the ROS repository"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${version} main\" > /etc/apt/sources.list.d/ros-latest.list"
fi

echo "Download the ROS keys"
roskey=`apt-key list | grep "ROS Builder"` && true # make sure it returns true
if [ -z "$roskey" ]; then
  echo "No ROS key, adding"
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
fi

echo "Updating & upgrading all packages"
sudo add-apt-repository universe
sudo add-apt-repository multiverse
sudo apt-get update
sudo apt-get dist-upgrade -y

echo "Installing ROS"
sudo apt install -y \
     ros-$ROS_DISTRO-desktop \
     git \
     curl \
     doxygen \
     libtbb-dev \
     python3-pip \
     python3-rosdep \
     python3-rosinstall \
     python3-bloom \
     python3-wstool \
     python3-catkin-lint \
     python3-catkin-tools \
     python3-osrf-pycommon \
     python3-rosinstall \
     python3-rosinstall-generator \
     build-essential \
     ros-noetic-tf2-geometry-msgs \
     ros-noetic-tf2-tools \
     ros-noetic-pcl-ros \
     ros-noetic-tf2-sensor-msgs \

pip install opencv-contrib-python

# Only init if it has not already been done before
if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
#source /opt/ros/melodic/setup.bash
echo "Done installing ROS"
#Create catkin workspace
mkdir -p ~/udacity_ws/src
cd ~/udacity_ws
catkin init # Init Catkin workspace
catkin config --extend /opt/ros/noetic  # exchange noetic for your ros distro if necessary
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release # To enable release mode compiler optimzations
# Move to catkin workspace /src and clone the project
cd ~/udacity_ws/src
# Clon repo
git clone https://github.com/alsarmie/3D-MAV-Path-Planner.git --recurse-submodules
# Power tool to address any remaining missing dependency.
cd ~/udacity_ws/src/3D-MAC-Path-Planner/path_planner
rosdep install --from-paths src --ignore-src --rosdistro noetic -y -r
cd ~/udacity_ws/src/3D-MAC-Path-Planner/pointcloud_publisher
rosdep install --from-paths src --ignore-src --rosdistro noetic -y -r
cd ~/udacity_ws/src/
source ~/.bashrc
catkin build
echo "source ~/udacity_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
exit 0
