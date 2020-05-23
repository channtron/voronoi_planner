#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from planner.srv import GoTo
import tf.transformations
import VoronoiPoints
import numpy as np
import matplotlib.pyplot as plt
from math import pow, sqrt

# Creamos una nueva clase celda, que contenga las coordenadas,
# los valores de G, H y F, un puntero a la celda padre

class celda:

    def __init__(self, x, y, start_cell, goal_cell, parent, heur):
        self.x = round(x, 0)
        self.y = round(y, 0)
        if heur == 0:  # Heuristica = distancia Manhattan
            self.g = abs(round(start_cell[1], 0) - self.x) + abs(
                round(start_cell[0], 0) - self.y)  # Distancia Manhattan al origen
            self.h = abs(round(goal_cell[1], 0) - self.x) + abs(
                round(goal_cell[0], 0) - self.y)  # Distancia Manhattan al destino

        elif heur == 1:  # Heuristica = distancia Euclidea
            self.g = sqrt(pow((start_cell[1] - self.x), 2) + pow((start_cell[0] - self.y), 2)) # Distancia euclidea al origen
            self.h = sqrt(pow((goal_cell[1] - self.x), 2) + pow((goal_cell[0] - self.y), 2)) # Distancia euclidea al destino

        self.f = self.g + self.h
        self.parent = parent


class Planner:

    def __init__(self):

        # A subscriber to the topic '/odom'. self.update_pose is called
        # when a message of type Pose is received.

        self.pose_subscriber = rospy.Subscriber('/odom',
                                                Odometry, self.update_pose)

        self.pose = Pose()

        # Load map
        self.map = np.genfromtxt('worlds/map2.csv', delimiter=',')
        self.height = self.map.shape[0]
        self.width = self.map.shape[1]
        self.resolution = 1

        # Print map
        # image = np.flipud(self.map)
        # plt.figure()
        # plt.imshow(image, cmap=plt.get_cmap('binary'), interpolation="nearest")
        # plt.show()

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""

        # para mellodic
        self.pose.x = round(data.pose.pose.position.x, 4)
        self.pose.y = round(data.pose.pose.position.y, 4)
        # para kinetic
        # self.pose.x = round(data.pose.pose.position.x - 7.5, 4)
        # self.pose.y = round(data.pose.pose.position.y - 4.5, 4)

        orientation = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
                       data.pose.pose.orientation.w]
        (_, _, yaw) = tf.transformations.euler_from_quaternion(orientation)

        self.pose.theta = yaw

    def compute_path(self, start_cell, goal_cell, heur):
        """Compute path."""
        path = []

        ######################
        # TODO: Implement A* # CON CUIDADO, ESTA SEGUN LAS COORDENADAS DEL CSV (0,0) ARRIBA A LA IZQUIERDA Y CON FORMATO (Y,X)
        ######################
        lista_abierta = []
        lista_cerrada = []
        fin = False
        iteraciones = 0  # Variable que cuenta las iteraciones
        en_lista = False  # en_lista = True si la celda actual se encuentra en la lista abierta o cerrada

        # objeto = celda(x, y, start_cell, goal_cell, parent)
        inicio = celda(start_cell[1], start_cell[0], start_cell, goal_cell, None, heur)

        # Anadimos la celda origen a la lista abierta
        # lista_abierta.append(inicio)

        # Calculamos los puntos de voronoi
        voronoi_points = VoronoiPoints.voronoi('worlds/map2.csv')

        celdas_voronoi=[]
        # Los hacemos objetos de clase celda y los guardamos en una lista
        for point in voronoi_points:
            celda_aux = celda(point[1], point[0], start_cell, goal_cell, None, heur)
            celdas_voronoi.append(celda_aux)

        # Obtenemos la celda de voronoi mas cercana al origen y al final

        celdas_voronoi.sort(key=lambda aux: aux.g, reverse=False) #Ordenamos los puntos de voronoi por distancia al origen
        celda_v_origen = celdas_voronoi[0]
        for celdasaux in celdas_voronoi:
            # Dentro de las que esten a la misma distancia elegimos la mas cercana al punto final
            if (celdasaux.g == celda_v_origen.g and celdasaux.h < celda_v_origen.h):
                celda_v_origen = celdasaux

        celdas_voronoi.sort(key=lambda aux: aux.h, reverse=False)  # Ordenamos los puntos de voronoi por distancia al final
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
            if celda_actual.x == celda_v_final.x and celda_actual.y == celda_v_final.y:
                # path = []
                path.append([round(goal_cell[1]), round(goal_cell[0])])
                path.append([celda_actual.x, celda_actual.y])
                celda_padre = celda_actual.parent
                while celda_padre.parent != None:
                    path.append([celda_padre.x, celda_padre.y])
                    celda_padre = celda_padre.parent
                path.append([celda_v_origen.x, celda_v_origen.y])
                path.append([inicio.x, inicio.y])
                path.reverse()
                print(path)
                fin = True
                break


            # Si no es el final calculamos las celdas vecinas, comprobamos si esta en la lista o hay obstaculo y repetimos el bucle
            else:
                # Calculamos celdas vecinas que pertenezcan a celdas de voronoi
                celdas_vecinas=[]
                for celdas in celdas_voronoi:
                    if abs(celdas.x - celda_actual.x) <= 1 and abs(celdas.y - celda_actual.y) <= 1:
                        celda_aux_2 = celda(celdas.x, celdas.y, start_cell, goal_cell, celda_actual, heur)
                        celdas_vecinas.append(celda_aux_2)

                # path.append([celda_actual.x, celda_actual.y])

                iteraciones = iteraciones + 1

                for vecino in celdas_vecinas:

                    # Si el vecino no es un obstaculo
                    if self.map[int(vecino.y), int(vecino.x)] == 0.0:

                        # Si ya estaba en la lista abierta o cerrada
                        for elemento in lista_abierta:
                            if elemento.x == vecino.x and elemento.y == vecino.y:
                                en_lista = True
                                if vecino.g < elemento.g:  # Si la variable actual tiene mejor g que la ya almacenada
                                    lista_abierta[elemento] = vecino

                        for elemento in lista_cerrada:
                            if elemento.x == vecino.x and elemento.y == vecino.y:
                                en_lista = True
                                if vecino.g < elemento.g:  # Si la variable actual tiene mejor g que la ya almacenada
                                    lista_cerrada[elemento] = vecino

                        # Si no estaba en ninguna de las listas, se anade a la lista abierta
                        if en_lista == False:
                            lista_abierta.append(vecino)

                        lista_abierta.sort(key=lambda aux: aux.f, reverse=False)
                        en_lista = False

            # En caso de fallo, no hace infinitas iteraciones
            if iteraciones > 100:
                print("Mas de 100 iteraciones, algo ha fallado :(")
                fin = True

        ######################
        # End A*             #
        ######################

        # Print path
        x = []
        y = []

        for point in path:
            x.append(point[0])
            y.append(point[1])

        image = np.flipud(self.map)
        plt.figure()
        plt.imshow(image, cmap=plt.get_cmap('binary'), interpolation="nearest")
        plt.plot(x,y, 'ro-', linewidth=2, markersize=5)
        plt.show()

        return path

    def goto(self):
        """Moves the robot to the goal."""

        goal_pose = Pose()
        # Get the input from the user.
        # Test with -0.5,3.5 meters
        goal_pose_x = int(input("Set your x goal: "))
        goal_pose_y = int(input("Set your y goal: "))
        tolerance = input("Set the tolerance: ")
        heur = int(input(" 0 = distancia Manhattan \n 1 = distancia Euclidea \n Introduce 0 o 1:"))

        # Compute current and goal cell
        # TODO: compute automatically

        """ La transformacion a metros es X + 10 / 8 - Y	"""

        current_cell = [8.5 - self.pose.y, self.pose.x + 10]
        goal_cell = [8.5 - goal_pose_y, goal_pose_x + 10]

        path = self.compute_path(current_cell, goal_cell, heur)

        for point in path:
            # TODO: Call service GoTo
            """rospy.wait_for_service('goto')"""
            fn = rospy.ServiceProxy('goto', GoTo)
            answ = fn(point[0] - 9.5, 7.5 - point[1], tolerance)  # Rehacemos el cambio de coordenadas
            pass


if __name__ == '__main__':
    try:
        rospy.init_node('robot_planner', anonymous=True)

        x = Planner()
        x.goto()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

