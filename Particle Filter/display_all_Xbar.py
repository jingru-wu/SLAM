import numpy as np
import sys
import pdb
import os
from MapReader import MapReader
import math
from matplotlib import pyplot as plt
from matplotlib import figure as fig
import time
from main_para import binary_map,visualize_map,visualize_timestep,visul_ray, init_particles_random,init_particles_freespace

def main():
    src_path_map = '../data/map/wean.dat'
    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map()
    visualize_map(occupancy_map)
    occupancy_map=binary_map(occupancy_map)

    xbar_path = 'Xbar3/'
    file_names=os.listdir(xbar_path)
    timestamp_list=[]
    for file in file_names:
        timestamp_list.append(int(file.split('.')[0]))
    timestamp_list=np.asarray(timestamp_list)
    index_list=np.argsort(timestamp_list)
    for index in index_list:
        X=np.load(xbar_path+file_names[index])
        print(X.shape)
        visualize_timestep(X, timestamp_list[index],occupancy_map)
    plt.pause(0.01)
if __name__=="__main__":
    main()
