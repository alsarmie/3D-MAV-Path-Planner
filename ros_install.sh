#!/bin/bash -eu

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
     python3-wstool \
     python3-catkin-lint \
     python3-catkin-tools \
     python3-osrf-pycommon \
     python3-roslaunch \
     python3-rosinstall-generator \
     build-essential \
     ros-noetic-tf2-geometry-msgs \
     ros-noetic-tf2-tools \
     ros-noetic-pcl-ros \
     ros-noetic-tf2-sensor-msgs \

pip install opencv-contrib-python
echo "Done installing ROS & project dependencies"

exit 0
