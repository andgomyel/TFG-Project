# Coordinate control of mobile robots in an unknown environment
This work was developed as my Final Project in Robots Engineering Degree in the University of Alicante. It proposes a Multi-Robot System to perform environment efficient exploration an mapping, as well as real-time object detection. The code contains the implementation of a Multi-Robot System

## Index
- [1.   Project background](#p1)
- [2.   Installation](#p2)
- [2.1. Software and hardware requirements](#p3)
- [2.2  Previous needed libraries](#p4)
- [2.3. Project installation](#p5)
- [3.   How to use](#p6)
- [4.   Demo](#p7)


## Project background <a name="p1"/>
The aim of this work is to design and develop a Multi-Robot based solution to solve a determinate task: identifying dangerous objects in an unknown environment. Specifically, the aim is to launch a group of robots into a specific place where there is a terrorist threat (or a danger of this kind, in general), to search for objects suspected of being explosives, such as backpacks or suitcases.

To this end, SLAM and efficient exploration algorithms are needed, as well as an object detection method to perform object recognition in real time.

## Installation <a name="p2"/>



## Software and hardware requirements <a name="p3"/>



## Previous needed libraries <a name="p4"/>



## Project installation <a name="p5"//>



## How to use <a name="p6"//>
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

