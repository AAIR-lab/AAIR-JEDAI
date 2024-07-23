The setup is tested for **Ubuntu 16.04** and **ROS Kinetic**.

Step 1: Install openrave depnedancies

```
sudo apt-get install cmake g++ git ipython minizip python-dev python-h5py python-numpy python-scipy python-sympy qt4-dev-tools

sudo apt-get install libassimp-dev libavcodec-dev libavformat-dev libavformat-dev libboost-all-dev libboost-date-time-dev libbullet-dev libfaac-dev libglew-dev libgsm1-dev liblapack-dev liblog4cxx-dev libmpfr-dev libode-dev libogg-dev libpcrecpp0v5 libpcre3-dev libqhull-dev libqt4-dev libsoqt-dev-common libsoqt4-dev libswscale-dev libswscale-dev libvorbis-dev libx264-dev libxml2-dev libxvidcore-dev

```
Step 2: Install collada-dom 

```
git clone https://github.com/rdiankov/collada-dom.git
cd collada-dom && mkdir build && cd build
cmake ..
make -j4
sudo make install
```

Step 3: Install OpenSceneGraph

```
sudo apt-get install libcairo2-dev libjasper-dev libpoppler-glib-dev libsdl2-dev libtiff5-dev libxrandr-dev
git clone --branch OpenSceneGraph-3.4 https://github.com/openscenegraph/OpenSceneGraph.git
cd OpenSceneGraph && mkdir build && cd build
cmake .. -DDESIRED_QT_VERSION=4
make -j4
sudo make install
```

Step 4: Install FCL collision checker 

```
sudo apt-get install libccd-dev
git clone https://github.com/flexible-collision-library/fcl.git
cd fcl
git checkout 0.5.0  # use FCL 0.5.0
mkdir build && cd build
cmake ..
make -j4
sudo make install
sudo ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen
```

Step 5: Install sympy version 0.7.1

```
sudo pip install --upgrade sympy==0.7.1
```

Note: Ubuntu 16 and 18 users would need to downgrade the mpmath library to 0.11 for openrave iksolvers and sympy 0.7.1 to work.

Step 6: Install Openrave
```
git clone --branch latest_stable https://github.com/rdiankov/openrave.git
git checkout 9c79ea260e1c009b0a6f7c03ec34f59629ccbe2c
cd openrave && mkdir build && cd build
cmake .. -DOSG_DIR=/usr/local/lib64/
make -j4
sudo make install

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(openrave-config --python-dir)/openravepy/_openravepy_
export PYTHONPATH=$PYTHONPATH:$(openrave-config --python-dir)
```
(You can (should) add this to your .bashrc to make it easy to run openrave on new terminals).

Step 7: Test Openrave.
```
openrave.py --example graspplanning
```

This should run the graspplanning tutorial from the openrave.

Step 8: Install ROS Kinetic (If not already installed):

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

Step 10: Setup the catkin workspace (if not already)

```
cd ~
mkdir -p catkin_ws/src
cd catkin_ws
catkin_make
source devel/setup.bash
```

Add "source ~/catkin_ws/devel/setup.bash" to your .bashrc and source it again.

Step 11: Install openrave_catkin 

```
cd ~/catkin_ws/src
git clone https://github.com/personalrobotics/openrave_catkin.git
cd ../
catkin_make
source devel/setup.bash
```

Step 12: Install or_urdf

```
cd ~/catkin/src
git clone https://github.com/personalrobotics/or_urdf.git
cd or_urdf
git checkout kinetic_fixes
cd ../../
catkin_make
source devel/setup.bash
```

Step 13: Install prpy

```
sudo apt-get install liblapacke-dev
sudo apt-get install libnewmat10*
sudo apt install libgsl-dev
sudo apt-get install ros-kinetic-chomp-motion-planner
sudo apt-get install ros-kinetic-sbpl*
sudo apt-get install ros-kinetic-ompl
python2 -m pip install enum34 --user
python2 -m pip install networkx==2.2 --user

cd ~/catkin_ws/src/
git clone https://github.com/personalrobotics/or_ompl
git clone https://github.com/personalrobotics/or_sbpl
git clone https://github.com/personalrobotics/or_cdchomp
git clone https://github.com/personalrobotics/comps
git clone https://github.com/personalrobotics/prpy
git clone https://github.com/personalrobotics/tsr
cd ..
catkin_make
source devel/setup.bash
```
