import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

def main():
    adj_pc = np.genfromtxt('adjusted_points.csv',delimiter=',')
    init_pc = np.genfromtxt('initial_points.csv',delimiter=',')
    print(adj_pc.shape)
        
    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(adj_pc[:,0],adj_pc[:,1],adj_pc[:,2])
    ax.scatter(init_pc[:,0],init_pc[:,1],init_pc[:,2], c='r')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    plt.show()
    
    fig2 = plt.figure(2)
    ax2 = fig.add_subplot(111, projection='3d')
    ax2.scatter(adj_pc[:,0],adj_pc[:,1],adj_pc[:,2])
    ax2.scatter(init_pc[:,0],init_pc[:,1],init_pc[:,2], c='r')
    
    ax2.set_xlim(np.min(init_pc[:,0]), np.max(init_pc[:,0]))
    ax2.set_ylim(np.min(init_pc[:,1]), np.max(init_pc[:,1]))
    ax2.set_zlim(-2000, 2000)
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    
    print(np.min(init_pc[:,0]), np.max(init_pc[:,0]))
    print(np.min(init_pc[:,1]), np.max(init_pc[:,1]))
    
    plt.show()
    
    
        
if __name__ == '__main__':
    sys.exit(main())
