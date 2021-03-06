# Coordinate control of mobile robots in an unknown environment
This work was developed as my Final Project in Robots Engineering Degree in the University of Alicante. It proposes a Multi-Robot System to perform environment efficient exploration an mapping, as well as real-time object detection. The code contains the implementation of the developed application to solve these tasks.

## Index
- [1.   Project background](#p1)
- [2.   Installation](#p2)
- [2.1. Hardware and software requirements](#p3)
- [2.2  Previous needed libraries](#p4)
- [2.3. Project installation](#p5)
- [3.   How to use](#p6)
- [4.   Demo](#p7)


## Project background <a name="p1"/>
The aim of this work is to design and develop a Multi-Robot based solution to solve a determinate task: identifying dangerous objects in an unknown environment. Specifically, the aim is to launch a group of robots into a specific place where there is a terrorist threat (or a danger of this kind, in general), to search for objects suspected of being explosives, such as backpacks or suitcases.

To this end, SLAM and efficient exploration algorithms are needed, as well as an object detection method to perform object recognition in real time. In this project, some state-of-the-art methods are applied to carry out such tasks.


## Followed approach 

To solve the task presented above through a Multi-Robot System, a state machine and a graphical interface were desgined to control the group of robots and monitorize its state.

The state machine desgined looks like this:
<p align="center">
  <img src="display/state_machine.png" alt="animated"/>
</p>

When the application is executed, the robots are on "Wait" state, until initializing order is received. When in "Initialize", the user must introduce the pose where the robots will begin their exploration. Thus, a "guided exploration" is achieved, accelerating exploration by sending each robot to a different region of the environment. While robots explore, they run an object detector, performing object recognition of the areas they visit. Once exploration ends or user tells robots to return to launching area, all of them return to Home position and application ends.


## Installation <a name="p2"/>
In order to execute the developed application using the above codes, one needs to install some required frameworks and libraries previously.


### Hardware and software requirements <a name="p3"/>
Due to the usage of neural network models to perform object detection in several images in real time, a GPU is required, as is a quite powerful processor.
Below are the main specs of the PC used to develop this project (these are the minimum neccessary requirements, more powerful GPU is recommended):

- CPU: 12 core Intel(R) Core(TM) i7-8750H CPU 2.20GHz
- GPU: NVIDIA GeForce GTX 1070 8 GB GDDR5
- 16 GB RAM

The entire project has been developed using ROS Noetic in Ubuntu 20.04.4, and hasn't been tested in another Linux distros nor ROS versions, so it is recommended using these versions for a suitable functioning. ROS Noetic installing instructions can be found [here](http://wiki.ros.org/noetic/Installation/Ubuntu) (```desktop-full``` version must be installed to acquire all the graphical tools needed).


### Previous needed libraries <a name="p4"/>

This project requires some packages or libraries to be installed before running the application. 

First of all, TurtleBot3 ROS package was used to simulate the envornment and the robots in Gazebo. Here are the instructions followed for its installation:
```
$ sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

$ sudo apt install ros-noetic-dynamixel-sdk
$ sudo apt install ros-noetic-turtlebot3-msgs
$ sudo apt install ros-noetic-turtlebot3
$ sudo apt install ros-noetic-turtlebot3-simulations
```

After installing TurtleBot3 simulator, execute the following line in command terminal to set ROS environment variables:
```
$ export TURTLEBOT3_MODEL=waffle > ~/.bashrc
```

TurtleBot3 simulation package includes SLAM, map merging and frontier exploration packages.


For object detection, ```darknet_ros``` package was used to process incoming images from robots cameras with YOLO detector. Installing instructions can be found at [darknet_ros Github repository](https://github.com/leggedrobotics/darknet_ros). As suggested there, using ```catkin build``` instead of ```catkin_make``` provides a better ROS package building pipeline. Clone the Github repo in ```~/catkin_ws/src``` directory


As mentioned before, CUDA framework must be used to perform object detection in real time. To use GPU parallel computing using CUDA, you need to install:

- NVIDIA CUDA Toolkit. The most recent version (```11.7```) was used. Follow these [instructions](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html) provided by NVIDIA.
- cuDNN (NVIDIA CUDA Deep Neural Networks Library). Installation guida can be found [here](https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html)

After installing CUDA Toolkit and cuDNN, execute this command in Linux terminal (I had problems if I didnt't execute it when pretending to use CUDA with ```darknet_ros```):
```
export PATH=/usr/local/cuda-11.7/bin${PATH:+:${PATH}} > ~/.bashrc
```

You also need OpenCV library to extract the image provided by the robots cameras from ROS topics. OpenCV will be used in background to forward pass the images through YOLO to perform object detection. Furthermore, using CUDA requires some OpenCV features to run correctly. This is why installing OpenCV 4.4.0 versi??n is highly recommended, as I had some incompatibility problems with 4.2.0 version and CUDA modules. Installing instructions can be found in this [website](https://vitux.com/opencv_ubuntu/). However, the ```cmake``` in Step 3 changes slightly to build CUDA modules. This is the command I used (from ``` ~/opencv/build``` directory):
```
$ cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D INSTALL_PYTHON_EXAMPLES=ON -D	INSTALL_C_EXAMPLES=OFF -D OPENCV_ENABLE_NONFREE=ON -D WITH_CUDA=ON -D WITH_CUDNN=ON -D OPENCV_DNN_CUDA=ON -D 			ENABLE_FAST_MATH=1 -D CUDA_FAST_MATH=1 -D CUDA_ARCH_BIN=6.1 -D WITH_CUBLAS=1 -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules -D HAVE_opencv_python3=ON ..
```

Once you have installed these features, check your CUDA architecture [here](https://developer.nvidia.com/cuda-gpus) (in my case, it is 6.1). Then, go to ```~/catkin_ws/src/darknet_ros/darknet_ros/CMakeLists.txt``` file and build the workspace after adding a line like this (it is already mentioned in ```darknet_ros``` repo, but I had some issues with that setup):
```
-O3 -gencode arch=compute_61,code=sm_61
```

At this point, you should have completely configured your system to run YOLO detector on GPU using ```darknet_ros```.


Only 2 more steps are left to end the installation. To code the explained state machine, Smach libray was used. To design and develop the visual interface, PyQt5 library for graphical applications was used. Below are the commands to respectively install such libraries.

```
$ sudo apt-get install ros-noetic-smach-ros
$ pip install PyQt5
```

### Project installation <a name="p5"/>
After configuring the needed libraries and packages with the above instructions, you are ready to try the application developed. The first step is cloning this repo in ```~/catkin_ws/src``` (the same workspace where you installed ```darknet_ros```) and builing the new package:
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/andgomyel/TFG-Project.git
$ cd .. & catkin build
```

To avoid updating workspace dependencies every time you open a new terminal, I suggest including the following line in ```~/.bashrc``` file:
```
source ~/catkin_ws/devel/setup.bash
```

Finally, copy the files inside folder ```~/catkin_ws/src/tfg/yolo_network_config``` of this package into ```~/catkin_ws/src/darknet_ros/darknet_ros/config```.


## How to use <a name="p6"/>
By this time, the project is ready to be executed. It takes some steps.
1. Launch Gazebo environment and spawn 3 TurtleBot3 robots:
```
# Terminal window 1
$ roslaunch tfg spawn_robots.launch
```

2. Start SLAM and map merging algorithms and open RViz visualization:
```
# Terminal window 2
$ roslaunch tfg robots_multi_slam.launch
```

3. Start ROS navigation stack:
```
# Terminal window 3
$ roslaunch tfg robots_move_base.launch
```

4. Load pretrained YOLO complete network model for each TurtleBot camera:
```
# Terminal window 4
$ ROS_NAMESPACE=tb3_0 roslaunch tfg darknet_ros_tfg.launch image:=/tb3_0/camera/rgb/image_raw param_file:=ros0.yaml

# Terminal window 5
$ ROS_NAMESPACE=tb3_1 roslaunch tfg darknet_ros_tfg.launch image:=/tb3_1/camera/rgb/image_raw param_file:=ros1.yaml

# Terminal window 6
$ ROS_NAMESPACE=tb3_2 roslaunch tfg darknet_ros_tfg.launch image:=/tb3_2/camera/rgb/image_raw param_file:=ros2.yaml
```

5. Execute state machine and open graphical interface:
```
# Terminal window 7
$ python ~/catkin_ws/src/tfg/src/my_interface.py
```

When all of these commands have been executed, your screen should look like this (Gazebo window would be in background, too):
<p align="center">
  <img src="display/screen.png" alt="animated"/>
</p>

Step 4 may be skkiped if you don't need to use object detection, and bounding boxes won't appear in the interface images.


## Demo <a name="p7"/>
You can find a video demo of how does the developed application works [here](https://www.youtube.com/watch?v=QfGfMR_f1rI).




