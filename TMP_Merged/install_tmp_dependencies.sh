#!/bin/bash
#
# Authors:
#   Deepak Kala Vasudevan<dkalavas@asu.edu>
#
# Description:
#   OpenRAVE Installation Script
# just an extra comment
echo "INFO: This script will modify your ~/.bashrc file!!"
UBUNTU_VER=$(lsb_release -sr)
if [ ${UBUNTU_VER} != '16.04' ] && [ ${UBUNTU_VER} != '18.04' ]; then
	echo "ERROR: Unsupported Ubuntu version: ${UBUNTU_VER}"
	echo "  Supported versions are: 14.04, 16.04 and 18.04"
	exit 1
fi

# Ros melodic installation
if [ ${UBUNTU_VER} = '16.04' ]; then
	echo "INFO: Installing ROS kinetic..........."
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
	sudo apt-get update
	sudo apt-get install ros-kinetic-desktop-full
	sudo rosdep init
	rosdep update
	echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
	source ~/.bashrc
	sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
elif [ ${UBUNTU_VER} = '18.04' ]; then
	echo "INFO: Installing ROS Melodic..........."
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
	sudo apt update
	sudo apt install ros-melodic-desktop-full -y
	sudo rosdep init
	rosdep update
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
	source ~/.bashrc
	sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential -y
fi

# Install ros dependencies
echo "INFO: Installing ROS packages..........."
if [ ${UBUNTU_VER} = '16.04' ]; then
	sudo apt-get install ros-kinetic-chomp-motion-planner ros-kinetic-sbpl* ros-kinetic-ompl ros-kinetic-trac-ik-* ros-kinetic-srdfdom* -y
elif [ ${UBUNTU_VER} = '18.04' ]; then
	sudo apt-get install ros-melodic-chomp-motion-planner ros-melodic-sbpl* ros-melodic-ompl ros-melodic-trac-ik-* ros-melodic-srdfdom*  -y
fi
# Install Python & other Linux packages
echo "INFO: Installing python and linux dependencies..........."
sudo apt-get install liblapacke-dev libnewmat10*  libgsl-dev -y
sudo python2.7 -m pip install pydot==1.2.3
sudo python2.7 -m pip install matplotlib==2.2.0 --user
sudo python2.7 -m pip install enum34 --user
sudo python2.7 -m pip install networkx==2.2 --user
sudo pip install --upgrade sympy==0.7.1

# Install Openrave
echo "INFO: Installing OpenRAVE..........."
cd ~
git clone https://github.com/AAIR-lab/or_catkin
if [ ${UBUNTU_VER} = '16.04' ]; then
	cd  or_catkin/
	git checkout kinetic
	cd ..
fi
cd or_catkin/openrave-installation/
sudo ./install-dependencies.sh
sudo ./install-osg.sh
sudo ./install-fcl.sh
sudo ./install-openrave.sh

cd ~
rm -rf or_catkin/openrave-installation/

# Creating Catkin worskpace for openrave plugins
echo "INFO: Installing OpenRAVE Plugins in at ~/tmp_catkin_ws/  workspace..........."
mkdir -p tmp_catkin_ws/src/
mv or_catkin/* tmp_catkin_ws/src/
rm -rf or_catkin/
cd tmp_catkin_ws/
catkin_make
echo "source ~/tmp_catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
cd ~
