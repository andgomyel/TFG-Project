# Coordinate control of mobile robots in an unknown environment
This work was developed as my Final Project in Robots Engineering Degree in the University of Alicante. It proposes a Multi-Robot System to perform environment efficient exploration an mapping, as well as real-time object detection. The code contains the implementation of a Multi-Robot System

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
- Explicar pòr encima lo que se ha pensado (máquina + interfaz), y lo que se ha necesitado para ello.

- Foto de la máquina de estados y la interfaz
Decir aquí que necesitamos:
- Algoritmo de SLAM Gmapping
- Algoritmo de map merging
- Algoritmo de frontier exploration
- Detector de objetos YOLO

## Installation <a name="p2"/>
In order to execute the developed application using the above codes, one needs to install some required frameworks and libraries previously.


### Hardware and software requirements <a name="p3"/>
Due to the usage of neural network models to perform object detection in several images in real time, a GPU is required, as is a quite powerful processor.
Below are the main specs of the PC used to develop this project (these are the minimum neccessary requirements, more powerful GPU is recommended):

- CPU: 12 core Intel(R) Core(TM) i7-8750H CPU 2.20GHz
  - GPU: NVIDIA GeForce GTX 1070 8 GB GDDR5
- 16 GB RAM

The entire project has been developed using ROS Noetic in Ubuntu 20.04.4, and hasn't been tested in another distros or ROS versions, so it is recommended using these versions for a suitable functioning. ROS Noetic installing instructions can be found [here](http://wiki.ros.org/noetic/Installation/Ubuntu). (```desktop-full``` version must be installed to acquire all the graphical tools needed)


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

You also need OpenCV library to extract the image provided by the robots cameras from ROS topics. OpenCV will be used in background to forward pass the images through YOLO to perform object detection. Furthermore, using CUDA requires some OpenCV features to run correctly. This is why installing OpenCV 4.4.0 versión is highly recommended, as I had some incompatibility problems with 4.2.0 version and CUDA modules. Installing instructions can be found in this [website](https://vitux.com/opencv_ubuntu/). However, the ```cmake``` in Step 3 changes slightly to build CUDA modules. This is the command I used (from ``` ~/opencv/build``` directory):
```
$ cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D INSTALL_PYTHON_EXAMPLES=ON -D	INSTALL_C_EXAMPLES=OFF -D OPENCV_ENABLE_NONFREE=ON -D WITH_CUDA=ON -D WITH_CUDNN=ON -D OPENCV_DNN_CUDA=ON -D 			ENABLE_FAST_MATH=1 -D CUDA_FAST_MATH=1 -D CUDA_ARCH_BIN=6.1 -D WITH_CUBLAS=1 -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules -D HAVE_opencv_python3=ON ..
```

Once you have installed these features, check your CUDA architecture [here](https://developer.nvidia.com/cuda-gpus) (in my case, it is 6.1). Then, check ```~/catkin_ws/src/darknet_ros/darknet_ros/CMakeLists.txt``` and add a line like this (it is already mentioned in ```darknet_ros``` repo, but I had some issues with that setup):
```
-O3 -gencode arch=compute_61,code=sm_61
```

At this time, you should have completely configured your system to run YOLO detector on GPU using ```darknet_ros```.


Only 2 more steps are left to end the installation. To code the explained state machine, Smach libray was used. To design and develop the visual interface, PyQt5 library for graphical applications was used. Below are the commands to respectively install such libraries.

```
$ sudo apt-get install ros-noetic-smach-ros
$ pip install PyQt5
```

### Project installation <a name="p5"/>



## How to use <a name="p6"/>
Once you have all the required features installed, ...



- Hardware and software requirements
- Uso de paquetes de ros para la navegación, mapeado... (poner los roslaunch en plan bonito y explicar qué lanza cada uno, quizá en un apartado de cómo usar)

## Related features installation <a name="p1"/>
- darknet_ros

```
$ source devel/setup.bash
```

# Previous needed libraries
- ROS Noetic
- Todo el paquete del TB3 (robots móviles)
- OpenCV, en su versión 4.4 a ser posible, pues es la más actualizada, para evitar incompatibilidades con los módulos de CUDA.



# Used PC datasheet
- Nvidia GTX 1070
- Intel Core i7


# Environment setup
- Se ha creado un entorno (.world) basado en turtlebot3_house.world, que añade las mochilas y tal
- El modelo 3D de las mochilas se ha descargado de X y modificado luego (esto no hace falta pq no lo he mencionado en el TFG)


Explicar en algún lado que la carpeta tfg hay que clonarla e introducirla en el workspace donde se haya instalado darknet

En el .bashrc, meter el export... y el source devel del workspace de darknet.


## How to use

