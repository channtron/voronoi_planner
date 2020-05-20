#!/usr/bin/env python3
#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import copy

def GenerateMap():
    # Cargamos los mapas de las distintas alturas

    suelo = np.genfromtxt('worlds/Plano/suelo + techo.csv', delimiter=',')
    M = suelo.shape[0]
    N = suelo.shape[1]
    image_suelo = np.flipud(suelo)

    alt1 = np.genfromtxt('worlds/Plano/Altura1.csv', delimiter=',')
    image_alt1 = np.flipud(alt1)

    alt3 = np.genfromtxt('worlds/Plano/Altura3.csv', delimiter=',')
    image_alt3 = np.flipud(alt3)

    alt4_5 = np.genfromtxt('worlds/Plano/Altura4,5.csv', delimiter=',')
    image_alt4_5 = np.flipud(alt4_5)

    Z=11
    map3d = np.zeros((Z,M,N))
    print(map3d.shape)

    # Para z=0, el plano vale suelo
    map3d[0][:][:]=image_suelo

    # De z=0.5 a z = 3 el plano vale alt1
    for j in range (1,6):
        map3d[j][:][:]=image_alt1

    # De z=3.5 a z = 4.5 el plano vale alt1
    for j in range (6,9):
        map3d[j][:][:]=image_alt3

    # De z=4.5 a z = 5 el plano vale alt1
    for j in range(9, 10):
        map3d[j][:][:] = image_alt4_5

    # Ultimo nivel de techo
        map3d[10][:][:] = image_suelo

    # fig = plt.figure()
    # ax = fig.gca(projection='3d')
    # ax.voxels(map3d[1:10][:][:],facecolors='blue' ,edgecolor='k')

    # plt.show()

    return map3d

def Voronoi3D(map):
    w_map = copy.deepcopy(map)
    n_map = copy.deepcopy(map)
    Z=map.shape[0]
    M=map.shape[1]
    N=map.shape[2]

    p_voronoi = []
    for z in range(1, Z - 2):
        for i in range(1, M - 2):
            for j in range(1, N - 2):
                
                if w_map[z, i, j] == 0:
                    None
                    if not (w_map[z-1, i, j] == 1 or w_map[z+1, i, j] == 1 or w_map[z, i-1, j] == 1 or w_map[z, i+1, j] == 1 or w_map[z, i, j-1] == 1 or w_map[z, i, j+1] == 1):
                        dist_obs = [0, 0, 0, 0, 0, 0]

                        # Miramos los vecinos hacia la derecha
                        k = i
                        while w_map[z, k, j] != 1:
                            k = k + 1
                        dist_obs[0] = abs(k - i)

                        # Miramos los vecinos hacia la izquierda
                        k = i
                        while w_map[z, k, j] != 1:
                            k = k - 1
                        dist_obs[1] = abs(k - i)

                        # Miramos vecinos hacia arriba
                        k = j
                        while w_map[z, i, k] != 1:
                            k = k + 1
                        dist_obs[2] = abs(k - j)

                        # Miramos vecinos hacia abajo
                        k = j
                        while w_map[z, i, k] != 1:
                            k = k - 1
                        dist_obs[3] = abs(k - j)

                        # Miramos vecinos encima
                        k = z
                        while w_map[k, i, j] != 1:
                            k = k + 1
                        dist_obs[4] = abs (k - z)

                        # Miramos vecinos debajo
                        k = z
                        while w_map[k, i, j] != 1:
                            k = k - 1
                        dist_obs[5] = abs(k - z)

                        cont = 0
                        d_min = min(dist_obs)
                        for w in dist_obs:
                            if w == d_min:
                                cont = cont + 1

                        if cont >= 3:
                            p_voronoi.append([z, i, j])
                            w_map[z, i, j] = 2


    # fig = plt.figure()
    # ax = fig.gca(projection='3d')
    # ax.voxels(map[1:Z-2][:][:], facecolors='blue', edgecolor='k')

    # for point in p_voronoi:
    #     ax.scatter(point[0], point[1], point[2])
    #     # print(point[0], point[1], point[2])
    # plt.show()

    return p_voronoi


if __name__ == "__main__":
    map = GenerateMap()
    points = Voronoi3D(map)
