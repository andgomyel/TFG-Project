# Coordinate control of mobile robots in an unknown environment
This work was developed as my Final Project in Robots Engineering Degree in the University of Alicante. It proposes a Multi-Robot System to perform environment efficient exploration an mapping, as well as real-time object detection. The code contains the implementation of a Multi-Robot System

## Index
- [1.   Software and hardware reuirements](#p1)


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
