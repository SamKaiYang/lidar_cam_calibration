#!/usr/bin/env python2
import numpy as np
from numpy.linalg import inv, qr
import math 
import pandas as pd
import matplotlib.pyplot as plt
import datetime
import scipy
import time
import socket
import datetime
import cv2

from velodyne_capture_v3 import init_velo_socket, get_pointcloud, get_cam_pointcloud
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle
from utils_data import *
from utils_data import CalibFields

# Init sockets
PORT = 2368
soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
soc.bind(('', PORT))

cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FPS, 30)
"""
# 0817lidar to cam Final rotation is:
-0.029497  -0.998977 -0.0342695
-0.0972901  0.0369909  -0.994568
0.994819 -0.0260027 -0.0982818

camera projection 0817 1280*720
projection 1280*720
769.534729 0.000000 653.262129 0.000000
0.000000 788.671204 360.453779 0.000000
0.000000 0.000000 1.000000 0.000000
"""
# TODO: Conversion matrix test 
camera_matrix = np.matrix([[755.469543,0.000000,621.616852],[0.000000,763.467896,386.262318],[0.000000,0.000000,1.000000]])
final_rotation = np.matrix([[-0.858299,0.51315,0.000899662],[0.0780764,0.132324,-0.988127],[-0.507176,-0.848038,-0.153638]])
rotation_inv = inv(final_rotation)
print("rotation_inv",rotation_inv)
print("camera_matrix",camera_matrix)
""" 
<Coordinate system (Lidar)>
x: forward 
y: left
z: up

<Coordinate system (Camera/image)>
x: down 720
y: right 1280
"""
# Average translation is:
t=np.array([[0.00627183],[0.0100153],[0.0256176],[1]])

zero = [0,0,0]
final_rotation_test = np.vstack((final_rotation,zero))
h_test = inv(np.hstack((final_rotation_test,t)))
print("h_test:",h_test)
t=np.array([[h_test[0,3] ],[h_test[1,3]],[h_test[2,3]]]) 
h=np.hstack((rotation_inv.T,t))
print("rotation_inv.T:",rotation_inv)
if not cap.isOpened():
	raise IOError("cannot open webcam")
while 1:
    pcl = get_pointcloud(soc)
	# Get frame
    flag, frame = cap.read()
    cv2.imshow('frame', frame)	
	# Practical vlp16 manual xyz frame
    X= pcl[:,0] 
    Y= pcl[:,1]
    Z= pcl[:,2]
    distance = pcl[:,3]

    
    # make A matrix (x y z)
    size= len(X)

    X1= np.matrix.transpose(X)
    Y1= np.matrix.transpose(Y)
    Z1= np.matrix.transpose(Z)
    W= np.ones(size)
    W1= np.matrix.transpose(W)
    A=[X1,Y1,Z1]
    pcl_matrix= np.matrix([X1,Y1,Z1,W1])
#----------------0818
    # Convert to vlp16 ros coordinate system output
    A=[X1,Y1,Z1,W1]
    real_vlp_to_ros = np.matrix([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]])
    pcl_matrix = np.matmul((real_vlp_to_ros),(A))
    # pcl_matrix = np.hstack([pcl_matrix,W1])
    # print("ducccccck",pcl_matrix)
    #-------------
    F = np.matmul((h),(pcl_matrix))
    cv_points = np.matmul((camera_matrix),(F))/F[2,:]

    imPoints=h.dot(pcl_matrix)        # transforming points from world frame to camera frame
    imPoints=camera_matrix.dot(imPoints)        # projecting points to image plane
    imPoints=imPoints/imPoints[2,:] 

    # cv2.circle(frame, (640,10), 20, (255,0,0), thickness=-1)
    for x in np.nditer(cv_points, flags = ['external_loop'], order = 'F'): 
        # print((x[0]))
        if int(x[0])<1280 and int(x[1])<720 :
            # 1280 -> right/u # 720 down/v  
            cv2.circle(frame, (int(x[0]),int(x[1])), 1, (255,0,0), thickness=-1)
            # print(x)
    cv2.imshow('frame', frame)

    c = cv2.waitKey(1)
    if c == 27:
        break

cap.release()
cap.destroyAllWindows()