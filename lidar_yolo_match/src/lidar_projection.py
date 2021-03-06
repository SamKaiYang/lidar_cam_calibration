#!/usr/bin/env python2
import numpy as np
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

# Init sockets
PORT = 2368
soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
soc.bind(('', PORT))

# For matrix values
xr = 95 * math.pi/180
yr = 10 * math.pi/180  
zr = 0 * math.pi/180


# start z by 90 y by -90
# Matrices: (current projection seems to be off, needs to be fixed?)
Xr = np.matrix([[1,0,0],[0,math.cos(xr),-1*math.sin(xr)],[0,math.sin(xr),math.cos(xr)]])
Yr = np.matrix([[math.cos(yr),0,math.sin(yr)],[0,1,0],[-1*math.sin(yr),0,math.cos(yr)]])
Zr = np.matrix([[math.cos(zr),-1*math.sin(zr),0],[math.sin(zr),math.cos(zr),0],[0,0,1]])

# F = np.matrix([[935,0,0],[0,935,0],[225,375,1]])
F = np.matrix([[746.757273,0,0],[0,750.804686,0],[611.527923,430.316650,1]])

# np.matrix([[769.539734,0.000000,560.541864],[0.000000,799.693787,451.0666283],[0.000000,0.000000,1.000000]])
R = np.matmul(Zr,Yr)
R= np.matmul(R,Xr)
print(R)

'''
Average rotation is:
0.99898   0.033474 -0.0302925
-0.0362764   0.994598 -0.0972613
0.0268731   0.098261   0.994798
Final rotation is:
-0.029497  -0.998977 -0.0342695
-0.0972901  0.0369909  -0.994568
0.994819 -0.0260027 -0.0982818
'''
# transformation = np.matrix([[-0.029497,-0.998977,-0.0342695],[-0.0972901,0.0369909,-0.994568],[0.994819,-0.0260027,-0.0982818]])
# transformation = np.matrix([[0.99898,0.033474,-0.0302925,-0.00030335],[-0.0362764,0.994598,-0.0972613,0.0610432],[0.0268731,0.098261,0.994798,0.0157042]])
# transformation = np.matrix([[0.99898,0.033474,-0.0302925],[-0.0362764,0.994598,-0.0972613],[0.0268731,0.098261,0.994798]])

# R = transformation
# print(R)
"""
Final ypr is:
1.27642
-1.67264
0.258647
Average translation is:
-0.00030335
0.0610432
0.0157042
"""
T = np.matrix([[3],[3],[-2.32]]) #origin

# T = np.matrix([[1.27642],[-1.67264],[0.258647]])
# T = np.matrix([[-0.00030335],[0.0610432],[0.0157042]])
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FPS, 30)
# fig, ax = plt.subplots(1)
# plt.ion()
# plt.show()


#0813 Average transformation is:
# 0.99898    0.033474  -0.0302925 -0.00030335
# -0.0362764    0.994598  -0.0972613   0.0610432
# 0.0268731    0.098261    0.994798   0.0157042
# 0           0           0           1

# F = np.matrix([[0.99898,0.033474,-0.0302925],[-0.0362764,0.994598,-0.0972613],[0.0268731,0.098261,0.994798]])
# T = np.matrix([[-0.00030335],[0.0610432],[0.0157042]])
if not cap.isOpened():
	raise IOError("cannot open webcam")

while 1:
	# Get pointcloud
	# pcl = get_pointcloud(soc)
	# test cam num lidar match 
	pcl = get_cam_pointcloud(soc, 0)
	# Get frame
	flag, frame = cap.read()
	cv2.imshow('frame', frame)	
	
	X= pcl[:,0]
	Y= pcl[:,1]
	Z= pcl[:,2]
	distance = pcl[:,3]
	# make A matrix (x y z)
	size= len(X)
	X1= np.matrix.transpose(X)
	Y1= np.matrix.transpose(Y)
	Z1= np.matrix.transpose(Z)
	A=[X1,Y1,Z1]
	A= np.matrix([X1,Y1 ,Z1])

	T1=np.matrix.transpose(T)
	T2= np.repeat(T1,size,axis=0)

	T2= np.matrix.transpose(T2)
	
	c2 = np.matmul((F), (R))
	
	c2 = 0.25*np.matmul((c2),(A+T2))
	# print(c2)
	# cv2.circle(frame, ,int(x[1])), 3, (0,255,0), thickness=-1)
	# Plot points on frame
	for x in np.nditer(c2, flags = ['external_loop'], order = 'F'): 
		cv2.circle(frame, (int(x[0]),int(x[1])), 1, (255,0,0), thickness=-1)
		#print(x)
	cv2.imshow('frame', frame)

	c = cv2.waitKey(1)
	if c == 27:
		break

cap.release()
cap.destroyAllWindows()
