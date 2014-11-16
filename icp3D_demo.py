from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np
from math import cos, sin, sqrt
from random import randint

from copy import copy
from copy import deepcopy

def generate3DCloud():
    arr = []
    for x in range(2):
        arr_x = []
        for y in range(9):
            arr_x.append(randint(y, y+1))
        arr.append(arr_x)
    arr_x = []
    for y in range(9):
        arr_x.append(randint(3, 5))
    arr.append(arr_x)
    return arr[0], arr[1], arr[2] 

def drawASphere():
    #draw sphere
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_aspect("equal")
    u, v = np.mgrid[0:np.pi:20j, 0:np.pi/2:10j]
    x=np.cos(u)*np.sin(v)
    y=np.sin(u)*np.sin(v)
    z=np.cos(v)
    ax.plot_wireframe(x, y, z, color="r")
    
    #and make another one by shifting it a bit
    y = y + 2
    z = z - 1
    ax.plot_wireframe(x, y, z, color="b")
    plt.show()

def getASphereCloud(n):
    '''
    u, v = np.mgrid[0:10:2, 0:20:1]
    x=u#np.cos(u)*np.sin(v)
    y=v
    z=u
    '''
    u, v = np.mgrid[0:np.pi:20j, 0:np.pi/2:10j]
    x=np.cos(u)*np.sin(v)
    y=np.sin(u)*np.sin(v)
    z=np.cos(v)
    #print "generate x array: ", v.shape
    w, h = v.shape
    data = []
    for i in range(w):
        for j in range(h):
            data.append([x[i][j], y[i][j], z[i][j]])
    #print "data 0: ", data[0]
    return data[0:n-1]

def getNeighbors(a_p, a_cloud, n_neighbors = 10):
    pnt_count = 0
    pnts_copy = deepcopy(a_cloud)
    rs = []
    while (pnt_count < n_neighbors and len(pnts_copy) > 0):
        # select 3 closest points
        minDist = 1e100
        minPoint = []
        for p in pnts_copy:
            dist = sqrt((p[0]-a_p[0])**2 + (p[1]-a_p[1])**2 + (p[2]-a_p[2])**2)
        
            if dist < minDist:
                minDist = dist
                minPoint = p

        rs.append(minPoint)
        pnts_copy.remove(minPoint)
        pnt_count += 1
        
    return rs;
    
def findNormalVector(a_neighbors, b_neighbors):#in fact this would be icp, from here: http://stackoverflow.com/questions/20528094/computing-the-3d-transformation-between-two-sets-of-points
    #constitute the matrix of Point
    '''a_neighbors is a matrix of (len(a_neighbors), 3)'''
    
    #calculate the centroid of a_neighbors
    a_centroid = np.mean(a_neighbors, axis=0)
    a_neighbors = a_neighbors - a_centroid
    #print "a_neighbors: ", a_neighbors
    #calculate the centroid of b_neighbors
    b_centroid = np.mean(b_neighbors, axis=0)
    b_neighbors = b_neighbors - b_centroid
    #print "shape: ", a_neighbors.shape, b_neighbors.shape
    #covariance matrix
    covariance = np.dot(np.transpose(a_neighbors), b_neighbors) 
    
    #covariance = np.transpose(covariance)
    #Singular Value Decomposition
    U, s, V = np.linalg.svd(covariance, full_matrices=True)
    
    #rotation matrix: R = V * U.t
    R = np.dot(V, np.transpose(U))
    #print "Rotation: ", R
    
    #Translation matrix: T = C_b - R * C_a
    T = b_centroid - np.dot(R, a_centroid)
    #print "translation: ", T
    
    return R, T
    
def convertTransformToOffset(offset, R, T):
    #offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    offset[0] += T[0]
    offset[1] += T[1]
    offset[2] += T[2]
    offset[3] += R[2, 1]
    offset[4] += -R[2, 0]
    offset[5] += R[1,0]
    print "final offset: ",offset
    return offset

def convertOffsetToTransform(offset):
    xd = offset[0]
    yd = offset[1]
    zd = offset[2]
    alpha = offset[3]
    beta = offset[4]
    theta = offset[5]
    R = np.matrix([    
            [cos(theta) * cos(beta), -sin(theta)*cos(alpha) + cos(theta)*sin(beta)*sin(alpha), sin(alpha)*sin(theta) + cos(theta)*sin(beta)*cos(alpha)],
            [sin(theta)*cos(beta)  , cos(theta)*cos(alpha) + sin(theta)*sin(beta)*sin(alpha) , -cos(theta)*sin(alpha) + sin(theta)*sin(beta)*cos(alpha)],
            [-sin(beta)            , cos(beta)*sin(alpha)                                    , cos(beta)*cos(alpha)                                    ],
            ])
    T = [xd, yd, zd]
    
    return R, T

def displacePoint(pts, offset):
    '''shifting the point cloud to another position
    using affine transformation'''
    xd = offset[0]
    yd = offset[1]
    zd = offset[2]
    alpha = offset[3]
    beta = offset[4]
    theta = offset[5]
    T = np.matrix([    
            [cos(theta) * cos(beta), -sin(theta)*cos(alpha) + cos(theta)*sin(beta)*sin(alpha), sin(alpha)*sin(theta) + cos(theta)*sin(beta)*cos(alpha) , xd],
            [sin(theta)*cos(beta)  , cos(theta)*cos(alpha) + sin(theta)*sin(beta)*sin(alpha) , -cos(theta)*sin(alpha) + sin(theta)*sin(beta)*cos(alpha), yd],
            [-sin(beta)            , cos(beta)*sin(alpha)                                    , cos(beta)*cos(alpha)                                    , zd],
            [0.0, 0.0, 0.0, 1.0]
            ])
    temp = [0.0, 0.0, 0.0, 1.0]
    #print pts[0]
    temp[0] = T[0,0]*pts[0] + T[0,1]*pts[1] + T[0,2]*pts[2] + T[0,3]
    temp[1] = T[1,0]*pts[0] + T[1,1]*pts[1] + T[1,2]*pts[2] + T[1,3]
    temp[2] = T[2,0]*pts[0] + T[2,1]*pts[1] + T[2,2]*pts[2] + T[2,3]
    temp[3] = T[3,0]*pts[0] + T[3,1]*pts[1] + T[3,2]*pts[2] + T[3,3] 
    #print temp
    p_off = [temp[0],temp[1],temp[2]]

    return p_off

def transformPoint(pts, R, T):
    temp = [0.0, 0.0, 0.0, 1.0]
    #print pts[0]
    temp[0] = R[0,0]*pts[0] + R[0,1]*pts[1] + R[0,2]*pts[2] + T[0]
    temp[1] = R[1,0]*pts[0] + R[1,1]*pts[1] + R[1,2]*pts[2] + T[1]
    temp[2] = R[2,0]*pts[0] + R[2,1]*pts[1] + R[2,2]*pts[2] + T[2]
    #temp[3] = T[3,0]*pts[0] + T[3,1]*pts[1] + T[3,2]*pts[2] + T[3,3] 
    #print temp
    p_off = [temp[0],temp[1],temp[2]]

    return p_off
    
def transformData(data, R, T):
    rs = []
    for d in data:
        rs.append(transformPoint(d, R, T))
    
    return rs

def displaceData(data, offset):
    '''shifting the point cloud to another position
    using affine transformation'''
    rs = []
    for d in data:
        rs.append(displacePoint(d, offset))
    
    return rs

def drawData(data, fig, c, m):
    #fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_aspect("equal")
    x = []
    y = []
    z = []
    for d in data:
        x.append(d[0])
        y.append(d[1])
        z.append(d[2])
    ax.scatter(x, y, z, color=c, marker=m)
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    
    
def icp_fitstep(a_pts, b_pts):
    #get the main point
    
    idx = randint(0, len(a_pts)-1)
    n_neighbors = 200
    d_neighbors = getNeighbors(b_pts[idx], b_pts, n_neighbors = n_neighbors)
    s_neighbors = getNeighbors(a_pts[idx], a_pts, n_neighbors = n_neighbors)
    R, T = findNormalVector(s_neighbors, d_neighbors)
    return R, T

def getCloudDistance(a_pts, b_pts):
    d = 0
    for i in range(len(a_pts)):
        a_p = a_pts[i]
        b_p = b_pts[i]
        d += sqrt((a_p[0]-b_p[0])**2 + (a_p[1]-b_p[1])**2 + (a_p[2]-b_p[2])**2)
    return d / len(a_pts)

init_off = [0.0,0.0,0.0,0.0,0.0,0.0]

if __name__ == '__main__':
    
    src = getASphereCloud(300)
    #fig = plt.figure()
    R, T = convertOffsetToTransform(init_off)
    #src = transformData(src, R, T)
    
    dest= getASphereCloud(300)
    #dest = transformData(src, R, T)
    R, T = convertOffsetToTransform([10.0,10.0,0.0,1.1,0.2,1.0])
    dest = transformData(dest, R, T)
    
    src1 = src
    min_dist = getCloudDistance(src1, dest)
    print "distance: ", min_dist
    fig = plt.figure()
    drawData(src, fig, "r", 'o')
    drawData(dest, fig, "b", 'x')
    drawData(src1, fig, "y", 'd')
    i = 0  
    while (min_dist > 0.01 and i < 100):
        i +=1
        
        R, T = icp_fitstep(src1, dest)
        d = getCloudDistance(transformData(src1, R, T), dest)
        if d < min_dist:
            #pass
            fig = plt.figure()
            src1 = transformData(src1, R, T)
            drawData(src, fig, "r", 'o')
            drawData(dest, fig, "b", 'x')
            drawData(src1, fig, "y", 'd')
            min_dist = d
            print "distance: ", getCloudDistance(src1, dest)
    print "stop at step ", i
    print "min_dist: ", min_dist
    plt.show()



