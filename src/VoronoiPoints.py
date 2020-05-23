#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt

def voronoi(path):
    # Se llama a la funcion voronoi y devuelve una lista de puntos de voronoi

    # Load map
    map = np.genfromtxt(path, delimiter=',')
    M = map.shape[0]
    N = map.shape[1]
    resolution = 1

    # Print map
    # image = np.flipud(map)
    # plt.figure()
    # plt.imshow(image, cmap=plt.get_cmap('binary'), interpolation="nearest")
    # plt.show()

    w_map = map
    n_map = map
    p_voronoi = []

    for i in range(0, M - 1):
        for j in range(0, N - 1):
            if w_map[i, j] == 0:
                dist_obs = [0, 0, 0, 0]

                # Miramos los vecinos hacia la derecha
                k = i;
                while w_map[k, j] != 1:
                    k = k + 1
                dist_obs[0] = abs(k - i)

                # Miramos los vecinos hacia la izquierda
                k = i
                while w_map[k, j] != 1:
                    k = k - 1
                dist_obs[1] = abs(k - i)

                # Miramos vecinos hacia arriba
                k = j
                while w_map[i, k] != 1:
                    k = k + 1
                dist_obs[2] = abs(k - j)

                # Miramos vecinos hacia abajo
                k = j
                while w_map[i, k] != 1:
                    k = k - 1
                dist_obs[3] = abs(k - j)

                cont = 0
                d_min = min(dist_obs)
                for w in dist_obs:
                    if w == d_min:
                        cont = cont + 1

                # Para evitar errores en habitaciones pares
                if cont < 2:
                    for w in dist_obs:
                        if w == d_min + 1:
                            cont = cont + 1

                if cont >= 2:
                    p_voronoi.append([i, j])
                    w_map[i, j] = 2;

    # Para terminar de cerrar el diagrama trazamos bisectrices entre los obstaculos

    for i in range(1, M - 2):
        for j in range(1, N - 2):
            if w_map[i, j] == 2:
                if w_map[i, j - 1] == 1 and w_map[i, j + 1] == 1:  # Detectar puertas horizontales
                    w_map[i + 1, j] = 2
                    w_map[i - 1, j] = 2
                    p_voronoi.append([i + 1, j])
                    p_voronoi.append([i - 1, j])

                    k = 2
                    while w_map[i + k, j] != 2 and w_map[i + k, j - 1] != 2 and w_map[i + k, j + 1] != 2: # Bisectriz hacia arriba
                        w_map[i + k, j] = 2
                        p_voronoi.append([i + k, j])
                        k = k + 1

                    k = 2
                    while w_map[i - k, j] != 2 and w_map[i - k, j - 1] != 2 and w_map[i - k, j + 1] != 2: # Bisectriz hacia abajo
                        w_map[i - k, j] = 2
                        p_voronoi.append([i - k, j])
                        k = k + 1

                elif w_map[i - 1, j] == 1 and w_map[i + 1, j] == 1:  # Detectar puertas verticales
                    w_map[i, j + 1] = 2
                    w_map[i, j - 1] = 2
                    p_voronoi.append([i, j + 1])
                    p_voronoi.append([i, j - 1])

                    k = 2
                    while w_map[i, j + k] != 2 and w_map[i - 1, j + k] != 2 and w_map[i + 1, j + k] != 2:  # Bisectriz hacia la derecha
                        w_map[i, j + k] = 2
                        p_voronoi.append([i, j + k])
                        k = k + 1

                    k = 2
                    while w_map[i, j - k] != 2 and w_map[i - 1, j - k] != 2 and w_map[i + 1, j - k] != 2:  # Bisectriz hacia la izquierda
                        w_map[i, j - k] = 2
                        p_voronoi.append([i, j - k])
                        k = k + 1

    image = np.flipud(map)
    plt.figure()
    plt.imshow(image, cmap=plt.get_cmap('binary'), interpolation="nearest")

    for point in p_voronoi:
        plt.plot(point[1], point[0], 'ro')
    plt.show()

    return p_voronoi


if __name__ == "__main__":
    path = 'worlds/map2.csv'
    #path = 'map.csv'
    puntos = voronoi(path)
