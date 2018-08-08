# dji_ws

`catkin` workspace for the ROS implementation of the landing site search and landing from an input rgb-d data using DJI matrice 100 drone.



# Setup Instructions

## Dependency ROS Packages

The packages can now be built using [catkin tools](https://catkin-tools.readthedocs.io/en/latest/). Following the installation instructions [here](http://catkin-tools.readthedocs.io/en/latest/installing.html):
```bash
# First you must have the ROS repositories which contain the .deb for catkin_tools:
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
# Once you have added that repository, run these commands to install catkin_tools:
sudo apt-get update
sudo apt-get install python-catkin-tools
```

To install other dependency packages required to run this workspace:
```bash


# To-Do list
sudo apt-get install ros-kinetic-octomap-mapping \  
                     ros-kinetic-depth-image-proc \
                     ros-kinetic-octomap-rviz-plugins \
                     ros-kinetic-octomap-ros \
                     ros-kinetic-openni-camera \
                     ros-kinetic-costmap-2d -y
```

## Building `glog` for ORB-SLAM2

Follow the steps below to install [`glog`](https://github.com/google/glog):
```bash
# install autoconf
sudo apt-get install autoconf
# setup directory and install
cd ~
git clone https://github.com/google/glog.git
cd glog
./autogen.sh && ./configure && make
sudo make install
```

# Build Instructions

1. Load git submodules
```bash
git submodule update --init
# if there are nested submodules:
git submodule update --init --recursive
```

2. Build the package using [`catkin build`](http://catkin-tools.readthedocs.io/en/latest/quick_start.html)
```bash
cd ~/catkin_ws
# To maximize performance, build the workspace in Release mode
catkin build
```


## Packages:
* [**costmap_search**](src/costmap_search) for searching landing sites from the input RGB-D data at a given instant.

* [**landing_site_search**](src/landing_site_search) for searching landing sites from the input RGB-D data at a given instant. This package is similar to the [costmap_search](src/costmap_search) package, however, it implements a ROS Service/Client for landing sites query. 

* [**voxel_downsampler**](src/voxel_downsampler) for downsampling (reduce the number of points in a point cloud dataset), using a voxelized grid approach. It represents the underlying surface more accurately.

* [**dataset_publisher**](src/dataset_publisher) for publishing data from the stereo camera. The package provides two possible modes of doing so. One is by using the ros-wrapper for zed camera, other is to publish the image files read from a `txt` file in the [`cfg`](cfg) directory.

* [**process_depth_data**](src/process_depth_data) for processing depth image from the camera and publishing the processed depth data and corresponding point cloud.

* [**fount_bringup**](src/fount_bringup) as a master layer to launch nodes from various packages together.

* [**dji_ws**](src/fount_bringup) as a master layer to launch nodes from various packages together for working with DJI matrice 100.

* [**landing_markers**](src/landing_markers) for visualizing landing sites as interactive markers on rviz.

## Submodules:

* [**mav_trajectory_generation**](https://github.com/himanshu-erol/mav_trajectory_generation) for finding optimised path from current position to clicked landing site.

* [**waypoint_navigator**](https://github.com/himanshu-erol/waypoint_navigator) for generating path from given waypoint in ENU/GPS frame.

* [**mav_dji_ros_interface**](https://github.com/himanshu-erol/mav_dji_ros_interface) for interfacing DJI matrice 100 with ros.

* [**grid_map**](https://github.com/himanshu-erol/grid_map) for using grid_map package to compute cost maps.

* [**octomap_mapping**](https://github.com/himanshu-erol/octomap_mapping) added parameters for using only defined ranges of dynamic point cloud to update octomap.




__NOTE:__ Each package has there own included `README` files. Kindly read them for better understanding of the code and parameters used.


# Usage

## connecting to DJI Matrice 100 through WIFI

1. ssh to your on-board TX2 computer with the following IP:

```

# if you are using 'deeplanding' wifi [netgear deeplanding router] (by default TX2 automatically connects to this on bootup)
ssh -X nvidia@192.168.168.123

# if you are using 'ais-robots' wifi in hall
ssh -X nvidia@192.168.167.123
```
__Note__: to change which wifi TX2 connects automatically TX2, follow this [tutorial](https://weworkweplay.com/play/automatically-connect-a-raspberry-pi-to-a-wifi-network) 


2. make sure you always run these commands to ensure that rosmaster is running on TX2 and ground computer can connect to it.

```
# On TX2
$source ~/.bashrc
$export ROS_IP=192.168.168.123; 
$export ROS_MASTER_URI=http://192.168.168.123:11311
# or you can simply run following: 
# (already included in ~/.bashrc file)
#$rosconfig_nvidia 

# on remote computer
$source ~/.bashrc
$export ROS_IP=192.168.168.100; 
$export ROS_MASTER_URI=http://192.168.168.123:11311
# or you can simply run following: 
# (already included in ~/.bashrc file)
#$rosconfig_deepnote_nvidia
 
```


## Starting DJI Matrice 100
* Make sure UART connection from TX2 and matrice 100 is up and running and remote control is set to 'F' mode.
To check the connection run following:  
```
roslaunch dji_interface dji_interface.launch
```
This will start publishing sensor messages from drone and commands on ros topics.




## Running the nodes

Clone this repo both on on-board computer and remote computer.

### Flying the Drone

1. ssh to your on-board TX2 computer and run the launch file:
```
$roslaunch dji_interface matrice100_indoor_MPC_trj.launch
```
2. Check the gear switch is down (i.e. in serial off mode).
3. Switch to 'F' mode.
4. Turn on motors.
5. push the gear switch switch up (i.e. serial on mode).
6. Now all modules are up and running in position control mode (i.e., you send position commands via the transmitter not attitude commands). 
7.  You can send a goal pose message using rqt (rqt can be opened by just typing rqt in a terminal). The figure below shows how to add a message publisher and the command mode. Example test can be to, set x=0, y=0, z=1 for position and x=0, y=0, z=0, w=1 for orientation.

![alt text](https://camo.githubusercontent.com/0d522181fad7dfd0aa63d1ce77c961d69993e95a/687474703a2f2f692e696d6775722e636f6d2f34784e304667472e706e67)

8. Various services for landing, takeoff, following path in lawn mover can be called from rqt.( For information about various services refer readme of [**mav_dji_ros_interface**](src/mav_dji_ros_interface) node.


For detailed steps you can refer to this [WIKI](https://github.com/ethz-asl/mav_dji_ros_interface/wiki).

__Note__: rqt and rviz should always be opened on remote computer with rosmaster running on on-board computer for preventing the slowing down of wifi connection from fetching graphics.



### Create octomap and publish landing sites
On on-board computer launch:
```
roslaunch dji_nvidia.launch
```

### Visualize landing sites
On remote computer launch:
```
roslaunch dji_remote.launch
```

## Overall working
After running the launch files as described above, call 'execute/lawn_mower_path' from rqt to start exploration with top landing sites visible as interactive markers on rviz. Once clicked, the path is generated from curent odometry position to clicked landing site. To initiate the autonomous landing call the 'execute/planned_path' service from rqt.

