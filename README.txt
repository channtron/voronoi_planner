Proyecto de 4º GIERM de la asignatura Ampliación de Robótica.
El objetivo de este proyecto es realizar un planificador basado en vornoi utilizando mapas de GRID para un quadrotor.
El simulador usado es el grvc_ual con el backend de gazebo_light.

Se prueba primero con un modelo en 2D simulado en Stage
COMANDOS PARA SIMULAR EL PLANIFICADOR EN 2D
MODELO EN 2D
1. roscore
2. roscd planner
3. rosrun stage_ros stageros worlds/simulation.world
4. rosrun voronoi_planner go_to_goal.py
5. rosrun voronoi_planner voronoi2D.py
