#!/usr/bin/env python
import rospy
from uav_abstraction_layer.srv import TakeOff, GoToWaypoint, Land
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import VoronoiPoints3D
import numpy as np
import matplotlib.pyplot as plt
from math import pow, sqrt
import copy

# Creamos una nueva clase celda, que contenga las coordenadas,
# los valores de G, H y F, un puntero a la celda padre

class celda:

    def __init__(self, x, y, z, start_cell, goal_cell, parent, heur):
        self.z = round(z, 0)
        self.x = round(x, 0)
        self.y = round(y, 0)
        if heur == 0:  # Heuristica = distancia Manhattan
            self.g = abs(round(start_cell[1], 0) - self.x) + abs(round(start_cell[0], 0) - self.y) \
                     + abs(round(start_cell[2], 0) - self.z)  # Distancia Manhattan al origen
            self.h = abs(round(goal_cell[1], 0) - self.x) + abs(round(goal_cell[0], 0) - self.y) \
                     + abs(round(goal_cell[2], 0) - self.z) # Distancia Manhattan al destino

        elif heur == 1:  # Heuristica = distancia Euclidea
            self.g = sqrt(pow((start_cell[1] - self.x), 2) + pow((start_cell[0] - self.y), 2) + pow((start_cell[2] - self.z), 2)) # Distancia euclidea al origen
            self.h = sqrt(pow((goal_cell[1] - self.x), 2) + pow((goal_cell[0] - self.y), 2) + pow((goal_cell[2] - self.z), 2)) # Distancia euclidea al destino

        self.f = self.g + self.h
        self.parent = parent


class Planner:

    def __init__(self):

        # A subscriber to the topic '/odom'. self.update_pose is called
        # when a message of type Pose is received.

        # self.pose_subscriber = rospy.Subscriber('/odom',
        #                                         Odometry, self.update_pose)
        #
        # self.pose = Pose()

        # Load map
        self.map = VoronoiPoints3D.GenerateMap()

    def compute_path(self, start_cell, goal_cell, heur):
        """Compute path."""

        path = []
        w_map = self.map
        n_map = copy.deepcopy(self.map)

        ######################
        #  Implementar algoritmo
        ######################
        lista_abierta = []
        lista_cerrada = []
        fin = False
        iteraciones = 0  # Variable que cuenta las iteraciones
        en_lista = False  # en_lista = True si la celda actual se encuentra en la lista abierta o cerrada

        # objeto = celda(x, y, start_cell, goal_cell, parent)
        inicio = celda(start_cell[1], start_cell[0], start_cell[2], start_cell, goal_cell, None, heur)

        # Anadimos la celda origen a la lista abierta
        # lista_abierta.append(inicio)

        # Calculamos los puntos de voronoi
        voronoi_points = VoronoiPoints3D.Voronoi3D(w_map)

        celdas_voronoi = []
        # Los hacemos objetos de clase celda y los guardamos en una lista
        for point in voronoi_points:
            celda_aux = celda(point[2], point[1], point[0], start_cell, goal_cell, None, heur)
            celdas_voronoi.append(celda_aux)

        # Obtenemos la celda de voronoi mas cercana al origen y al final

        celdas_voronoi.sort(key=lambda aux: aux.g,
                            reverse=False)  # Ordenamos los puntos de voronoi por distancia al origen
        celda_v_origen = celdas_voronoi[0]
        for celdasaux in celdas_voronoi:
            # Dentro de las que esten a la misma distancia elegimos la mas cercana al punto final
            if (celdasaux.g == celda_v_origen.g and celdasaux.h < celda_v_origen.h):
                celda_v_origen = celdasaux

        celdas_voronoi.sort(key=lambda aux: aux.h,
                            reverse=False)  # Ordenamos los puntos de voronoi por distancia al final
        celda_v_final = celdas_voronoi[0]
        for celdasaux2 in celdas_voronoi:
            # Dentro de las que esten a la misma distancia elegimos la mas cercana al punto final
            if (celdasaux2.h == celda_v_origen.h and celdasaux2.g < celda_v_origen.g):
                celda_v_final = celdasaux2

        lista_abierta.append(celda_v_origen)

        while fin == False:
            # Ordenamos la lista abierta segun la menor f
            lista_abierta.sort(key=lambda aux: aux.f, reverse=False)

            # Sacamos el primer elemento de la lista abierta y lo introducimos en la cerrada
            lista_cerrada.append(lista_abierta[0])
            celda_actual = lista_abierta[0]
            lista_abierta.remove(celda_actual)

            # path.append([celda_actual.x, celda_actual.y])

            # Comprobamos si hemos llegado al final:
            # Si es el final recorremos la cadena de padres hasta el origen
            if celda_actual.x == celda_v_final.x and celda_actual.y == celda_v_final.y and celda_actual.z == celda_v_final.z:
                # path = []
                path.append([round(goal_cell[1]), round(goal_cell[0]), round(goal_cell[2])])
                path.append([celda_actual.x, celda_actual.y, celda_actual.z])
                celda_padre = celda_actual.parent
                if celda_padre is not None:  # Si el destino esta a una casilla del origen, sin esto daria fallo
                    while celda_padre.parent is not None:
                        path.append([celda_padre.x, celda_padre.y, celda_actual.z])
                        celda_padre = celda_padre.parent
                path.append([celda_v_origen.x, celda_v_origen.y, celda_actual.z])
                path.append([inicio.x, inicio.y, inicio.z])
                path.reverse()
                print(path)
                fin = True
                break


            # Si no es el final calculamos las celdas vecinas, comprobamos si esta en la lista o hay obstaculo y repetimos el bucle
            else:
                # Calculamos celdas vecinas a una distancia r que pertenezcan a celdas de voronoi
                r = 5
                celdas_vecinas = []
                atraviesa_pared = 0

                for celdas in celdas_voronoi:
                    dist_x = celdas.x - celda_actual.x
                    dist_y = celdas.y - celda_actual.y
                    dist_z = celdas.z - celda_actual.z

                    if dist_x >= 0:
                        sign_x = 1
                    else:
                        sign_x = -1

                    if dist_y >= 0:
                        sign_y = 1
                    else:
                        sign_y = -1

                    if dist_z >= 0:
                        sign_z = 1
                    else:
                        sign_z = -1

                    if abs(dist_x) <= r and abs(dist_y) <= r and abs(dist_z) <= r:
                        # if(celdas.x == 28 and celdas.y ==3 and celdas.z == 8):
                        #     print("u")
                        # Comprobamos que entre la celda vecina y la actual no hay obstaculos
                        for z in range(0, int(dist_z), sign_z):
                            for x in range(0, int(dist_x), sign_x):
                                for y in range(0, int(dist_y), sign_y):
                                    if self.map[
                                        (celda_actual.z + z), (celda_actual.y + y), (celda_actual.x + x)] == 1.0:
                                        atraviesa_pared = 1

                        if atraviesa_pared == 0:
                            celda_aux_2 = celda(celdas.x, celdas.y, celdas.z, start_cell, goal_cell, celda_actual, heur)
                            celdas_vecinas.append(celda_aux_2)

                iteraciones = iteraciones + 1

                for vecino in celdas_vecinas:

                    # Si el vecino no es un obstaculo
                    if self.map[int(vecino.z), int(vecino.y), int(vecino.x)] != 1:

                        # Si ya estaba en la lista abierta o cerrada
                        for elemento in lista_abierta:
                            if elemento.x == vecino.x and elemento.y == vecino.y and elemento.z == vecino.z:
                                en_lista = True
                                if vecino.g < elemento.g:  # Si la variable actual tiene mejor g que la ya almacenada
                                    lista_abierta[elemento] = vecino

                        for elemento in lista_cerrada:
                            if elemento.x == vecino.x and elemento.y == vecino.y and elemento.z == vecino.z:
                                en_lista = True
                                if vecino.g < elemento.g:  # Si la variable actual tiene mejor g que la ya almacenada
                                    lista_cerrada[elemento] = vecino

                        # Si no estaba en ninguna de las listas, se anade a la lista abierta
                        if en_lista == False:
                            lista_abierta.append(vecino)

                        lista_abierta.sort(key=lambda aux: aux.f, reverse=False)
                        en_lista = False

            # En caso de fallo, no hace infinitas iteraciones
            if iteraciones > 600:
                print("Mas de 100 iteraciones, algo ha fallado :(")
                fin = True

        ######################
        # End Algoritmo            #
        ######################

        # Print path
        x = []
        y = []
        z = []

        Z = self.map.shape[0]

        for point in path:
            x.append(point[0])
            y.append(point[1])
            z.append(point[2])

        # fig = plt.figure()
        # ax = fig.gca(projection='3d')
        # ax.voxels(n_map[1:10][:][:], facecolors='blue', edgecolor='k')

        # for point in path:
        #     ax.scatter(point[2], point[1], point[0])
            # print(point[0], point[1], point[2])
        # plt.show()

        return path


    def goto(self):

        current_cell = [10, 20, 1]
        print("-7, 4, 4 funciona bien")

        goal_cell_x = input("Set your x goal: ")
        goal_cell_y = input("Set your y goal: ")
        goal_cell_z = input("Set your z goal: ")

        goal_cell=[2*(5 - goal_cell_y), 2*(goal_cell_x + 10), 2*(goal_cell_z - 0.5)]

        while 1:

            heur = 0

            path = self.compute_path(current_cell, goal_cell, heur)

            # Hacemos la conversion de puntos del planificador a puntos del dron
            # El planificador esta al doble de tamano para mas precision (hay que dividir entre 2)
            for points in path:
                points[0] = points[0]/2 - 10 # Conversion x
                points[1] = 5 - points[1]/2 # Conversion y
                points[2] = points[2]/2 + 0.5 # Conversion z
                print(points)

            takeoff = rospy.ServiceProxy('ual/take_off', TakeOff)
            ready = takeoff(0.5,1)

            for point in path:

                goto = rospy.ServiceProxy('ual/go_to_waypoint', GoToWaypoint)
                waypoint = PoseStamped()
                waypoint.header.seq = 0
                waypoint.header.stamp.secs = 0
                waypoint.header.stamp.nsecs = 0
                waypoint.header.frame_id = ''
                waypoint.pose.position.x = point[0]
                waypoint.pose.position.y = point[1]
                waypoint.pose.position.z = point[2]
                waypoint.pose.orientation.x = 0
                waypoint.pose.orientation.y = 0
                waypoint.pose.orientation.z = 0
                waypoint.pose.orientation.w = 0

                #blocking = false
                ready = goto( waypoint , 1 )

            landing = rospy.ServiceProxy ('ual/land', Land)
            ready = landing(1)

            # La nueva celda actual sera la antigua celda final
            current_cell = [goal_cell[0], goal_cell[1], 1]

            # Proximo destino
            goal_cell_x = input("Set your x goal: ")
            goal_cell_y = input("Set your y goal: ")
            goal_cell_z = input("Set your z goal: ")

            goal_cell = [2 * (5 - goal_cell_y), 2 * (goal_cell_x + 10), 2 * (goal_cell_z - 0.5)]

if __name__ == '__main__':

    try:
        rospy.init_node('Dron_goto', anonymous=True)

        x = Planner()
        x.goto()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
