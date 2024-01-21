# TFG-Robotics
TFG 2023-2024 Víctor Gallardo

Control de una formación  de *platooning* usando un equipo de robots.
 Este es el repositorio oficial del TFG de Víctor Gallardo.

Este trabajo pretende realizar una implementación de un control de *platooning* basándose en el artículo https://ieeexplore.ieee.org/document/10113481 en el que se consigue un *platooning* de robots con ordenación espontánea. Es una adaptación por lo que tiene algunos cambios
con respecto al análisis matemático que ahí hacen.

Resumiendo, este repositorio propone cosas como trayectorias circulares de N robots en un *platooning*...

![Imagen de trayectoria circular](https://github.com/viictorgallardo/TFG-Robotics/blob/main/catkin_ws/src/robot_core/assets/3NCirc-Trayec.png)

Trayectorias *Lissajous* de N robots...


Incluso pruebas en real...




Se puede encontrar todo el código dentro de *robot_core* donde lo más importante es:

-simular.launch : lanza una simulacion en STAGE para ver una ordenacion de trayectoria circular o *Lissajous*

-circular2DTrajectory: es el calculador de puntos de la trayectoria circular

-Lissajous2DTrajectory: es el calculador de puntos de la trayectoria Lissajous

-moveRobot.cpp: es el nodo de movimiento de los robots y detección de obstáculos.




