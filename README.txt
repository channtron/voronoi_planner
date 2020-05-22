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


MODELO EN 3D
1. roslaunch ual_backend_gazebo_light simulation.launch (lanza roscore y gazebo con el dron)
2. rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/voronoi_planner/worlds/Plano/model.sdf -sdf -x 0 -y 0 -z 0 -model Plano
(lanza el mapa y lo coloca en el (0,0,0)
