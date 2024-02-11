# FlyWheel

Hardware and Software for FlyWheel, a mobile robot for testing models of Fruit Fly Vision.

## Project Organization
- /hardware: Bill of materials and stl files for manufacturing FlyWheel
- /sns_flywheel: ROS package for controlling FlyWheel
- camera_overrides.isp: Preference file for proper color calibration of the cameras, from https://jonathantse.medium.com/fix-pink-tint-on-jetson-nano-wide-angle-camera-a8ce5fbd797f. See Installation for details
- robot_driver.ino: Arduino sketch to control the motors as a ros node.

## Installation

### Hardware Configuration
Please refer to the hardware directory for more information.

### Software Configuration
- Flash the sd card and set up the Jetson following the directions at: https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit#intro . This project is built using Jetpack 5.1.2, available at: https://developer.nvidia.com/embedded/jetpack-sdk-512
- Install ROS-Noetic following the instructions at: http://wiki.ros.org/noetic
- Set up a catkin workspace following the instructions at: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
- Clone this repository into ~/catkin_ws/src
- Install the joy package using "sudo apt-get install ros-noetic-joy"
- Install pip: sudo apt install python3-pip
- Install libopenblas: sudo apt-get install libopenblas-dev
- Upgrade numpy: pip install --upgrade numpy
- Export the NVIDIA Jetson torch url using: export TORCH_INSTALL=https://developer.download.nvidia.cn/compute/redist/jp/v511/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
- Install torch: pip install --no-cache $TORCH_INSTALL 
- Correct the camera color profiles using camera_overrides.isp, following the directions at https://jonathantse.medium.com/fix-pink-tint-on-jetson-nano-wide-angle-camera-a8ce5fbd797f
