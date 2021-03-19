# gtsam_ros
ROS wrapper for GTSAM/iSAM2
## Notes
this is the version after integrated with robotx planning module

## Getting Started
- Install ROS melodic.
- Install [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page).
```bash
$ git clone https://gitlab.com/libeigen/eigen.git
$ cd eigen
$ mkdir build
$ cmake ..
$ sudo make install
```
- Install boost libraries, >=1.58 (Ubuntu: `sudo apt-get install libboost-all-dev`).
- Install CMake, >=3.0.2 (Ubuntu: `sudo apt-get install cmake`).
- Install TBB, >=4.4 (Ubuntu: `sudo apt-get install libtbb-dev`).
- Install [GTSAM](https://github.com/borglab/gtsam). Newest version is not compatible. Use version [4.0.3](https://github.com/borglab/gtsam/tree/4.0.3) instead.
```bash
$ git clone https://github.com/borglab/gtsam.git
$ cd gtsam
$ mkdir build
$ cmake ..
$ sudo make install
```
- Install [landmark_detection](https://github.com/MichiganRobotX/3d_projection) ROS package.
- Clone this repo into ROS workspace and make:
```bash
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ilnehc/gtsam_ros.git
$ cd ~/catkin_ws/
$ catkin_make
```

# gtsam_ros
