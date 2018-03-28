import argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

def main():
    parser = argparse.ArgumentParser(
        description = 'Plot the reconstructed 3D points.')
    parser.add_argument('initial_points', type = str,
        help = 'path to a file containing the points used to initialize bundle adjustment.')
    parser.add_argument('reconstructed_points', type = str,
        help = 'path to a file containing the 3D points reconstructed by bundle adjustment.')

    args = parser.parse_args()

    # Min and max values for trimmed pointcloud
    x_min = -13627500
    x_max = -13627000
    y_min = -4546000
    y_max = -4544000

    # Plot full pointclouds
    adj_pc = np.genfromtxt(args.reconstructed_points, delimiter = ',')
    init_pc = np.genfromtxt(args.initial_points, delimiter = ',')
    print(adj_pc.shape)
        
    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(adj_pc[:,0],adj_pc[:,1],adj_pc[:,2])
    ax.scatter(init_pc[:,0],init_pc[:,1],init_pc[:,2], c='r')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    plt.show()
    
    # Create trimmed point cloud
    x_indicator = np.logical_and(adj_pc[:,0] > x_min, adj_pc[:,0] < x_max)
    y_indicator = np.logical_and(adj_pc[:,1] > y_min, adj_pc[:,1] < y_max)
    indices = np.where(np.logical_and(x_indicator,y_indicator))[0]
    trimmed_pc = adj_pc[indices,:]
    print np.sum(y_indicator)
    print np.sum(x_indicator)
    print trimmed_pc.shape
    
    # Plot trimmed point cloud
    fig2 = plt.figure(2)
    ax2 = fig2.add_subplot(111, projection='3d')
    ax2.scatter(trimmed_pc[:,0],trimmed_pc[:,1],trimmed_pc[:,2])
    
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    
    plt.show()
        

if __name__ == '__main__':
    sys.exit(main())
