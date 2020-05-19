import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

def GenerateMap():
    # Cargamos los mapas de las distintas alturas

    suelo = np.genfromtxt('Plano/suelo + techo.csv', delimiter=',')
    M = suelo.shape[0]
    N = suelo.shape[1]
    image_suelo = np.flipud(suelo)

    alt1 = np.genfromtxt('Plano/Altura1.csv', delimiter=',')
    image_alt1 = np.flipud(alt1)

    alt3 = np.genfromtxt('Plano/Altura3.csv', delimiter=',')
    image_alt3 = np.flipud(alt3)

    alt4_5 = np.genfromtxt('Plano/Altura4,5.csv', delimiter=',')
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

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.voxels(map3d[1:10][:][:],facecolors='blue' ,edgecolor='k')

    plt.show()

    return map3d

def Voronoi3D(map):
    w_map = map
    n_map = map
    Z=map.shape[0]
    M=map.shape[1]
    N=map.shape[2]

    p_voronoi = []
    for z in range(0, Z - 1):
        for i in range(0, M - 1):
            for j in range(0, N - 1):
                if w_map[z, i, j] == 0:
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

                    if cont >= 2:
                        p_voronoi.append([z, i, j])
                        w_map[z, i, j] = 2;

    for z in range(0, Z - 1):
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.voxels(map[z:z+1][:][:], facecolors='blue', edgecolor='k')

        for point in p_voronoi:
            if point[0] == z:
                ax.scatter(point[0], point[1], point[2])
                # print(point[0], point[1], point[2])
        plt.show()

    return p_voronoi


if __name__ == "__main__":
    map = GenerateMap()
    points = Voronoi3D(map)
