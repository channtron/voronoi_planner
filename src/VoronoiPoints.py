#!/usr/bin/env python
import numpy as np
import cv2 # libreria para el manejo de imagens
import matplotlib.pyplot as plt
import copy

def voronoi(path):
    # Se llama a la funcion voronoi y devuelve una lista de puntos de voronoi

    # Load map
    map = np.genfromtxt(path, delimiter=',')
    Y = map.shape[0]
    X = map.shape[1]
    map_amp = cv2.resize(map, dsize=(Y*3, X*3), interpolation=cv2.INTER_NEAREST)
    resolution = 1/3
    
    M = map_amp.shape[0]
    N = map_amp.shape[1]

    # print(map_amp)
    # image = np.flipud(map_amp)
    # plt.figure()
    # plt.imshow(image, cmap=plt.get_cmap('binary'), interpolation="nearest")
    # plt.show()

    w_map = copy.deepcopy(map_amp)
    n_map = copy.deepcopy(map_amp)
    p_voronoi = []
        
    vecinos=[[0, 1], [1, 0], [0, -1], [-1, 0]]

    for q in range(15):
        for i in range(1, M - 2):
            for j in range(1, N - 2):
                if w_map[i, j] == 0:
                    for k in vecinos:
                        if w_map[i + k[0], j + k[1]] != 0:
                            n_map[i,j] = w_map[i + k[0], j + k[1]]
                            break
        w_map = copy.deepcopy(n_map)                

    n_map = np.zeros((M,N))

    for i in range(1, M - 2):
        for j in range(1, N - 2):
            if map_amp[i, j] == 0:
                w_cell = w_map[i, j]
                for k in vecinos:
                    if w_map[i + k[0], j + k[1]] != w_cell:
                        n_map[i,j] = 1
                        p_voronoi.append([i* Y/M, j* X/N]) # [i, j] - Y/2 - X/2
                        break

    # image = np.flipud(n_map)
    # plt.figure()
    # plt.imshow(image, cmap=plt.get_cmap('binary'), interpolation="nearest")
    # plt.show()

    
    # print(len(p_voronoi))
    pvoronoi=[]
    for punto in p_voronoi:
        if punto not in pvoronoi:
            pvoronoi.append(punto)
    p_voronoi = pvoronoi
    # print(len(p_voronoi))

    # print(p_voronoi)

    image = np.flipud(map)
    plt.figure()
    plt.imshow(image, cmap=plt.get_cmap('binary'), interpolation="nearest")

    for point in p_voronoi:
        plt.plot(point[1], point[0], 'ro')
    plt.show()

    return p_voronoi # devuelve lista de coordenadas [y, x]


if __name__ == "__main__":
    path = 'worlds/map_coloreado.csv'
    puntos = voronoi(path)
