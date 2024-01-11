# TFG-Robotics
TFG 2023-2024 Víctor Gallardo

Control de una formación  de "platooning" usando un equipo de robots.
 Este es el repositorio oficial del TFG de Víctor Gallardo

Este trabajo pretende realizar una implementación del artículo _ en el que se consigue un platooning de robots con ordenación espontánea. Es una adaptación por lo que tiene algunos cambios
con respecto al análisis matemático que ahí hacen.

Se puede encontrar todo el código dentro de robot_core donde lo más importante es:

simular.launch : lanza una simulacion en STAGE para ver una ordenacion de trayectoria circular o Lissajous

circular2DTrajectory: es el calculador de puntos de la trayectoria circular

Lissajous2DTrajectory: es el calculador de puntos de la trayectoria Lissajous

moveRobot.cpp: es el nodo de movimiento de los robots y detección de obstáculos.
