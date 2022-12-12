#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec  9 16:44:12 2022

@author: vgrefa
"""
import matplotlib.pyplot as plt
import pandas as pd
import math
import numpy as np

with open('data_analysis/aptEnv.txt') as f:
    lines = f.readlines()

lines2=lines.copy()
for i in range(0,len(lines)):
    new=lines[i].split(',')
    lines[i]=new
    
for i in range(0,len(lines)):
    new2=lines[i][1].split(' ')
    lines[i][1]=(float(new2[1]),float(new2[2]),float(new2[3]))
    new3 = lines[i][2].split(' ')
    lines[i][2]=(float(new3[3]),float(new3[4]))
    new4=lines[i][3].split(' ')
    lines[i][3]=(float(new4[1]),float(new4[2]),float(new4[3]))

#position(3), map input velocity (-1,1)(4), map turning(5), heading angle rad(6), apf direction(7-8), spot speed(9), sim turning(10)}
# with open('data_analysis/wallAvoidance2.csv') as f:
# with open('data_analysis/aptNav.csv') as f:
with open('data_analysis/wallStop.csv') as f:
    Wall = f.readlines()

posx = []
posy = []
mspeed = []
mturning = []
hangle = []
apfx = []
apfy = []
sspeed = []
sturning = []
forceangle = []
time = []

for i in range(0,len(Wall)):
    new=Wall[i].split(',')
    posx.append(float(new[0]))
    posy.append(float(new[1]))
    mspeed.append(float(new[3]))
    mturning.append(float(new[4]))
    hangle.append(np.rad2deg(float(new[5]))) #radians psi
    apfx.append(float(new[6]))
    apfy.append(float(new[7]))
    sspeed.append(float(new[8]))
    sturning.append(float(new[9]))
    forceangle.append(np.rad2deg(math.atan2(apfy[i],apfx[i])))
    time.append(float(new[10]))
    
fig, ax = plt.subplots(figsize =(16, 9))
plt.grid(color='grey', linestyle='-.', linewidth=0.5,which='both')

plt.plot(time,mspeed)
plt.plot(time,sspeed)
plt.xlabel("Time [s]")
plt.ylabel("Speed [scalar]")
plt.legend(['Input Speed','Spot Speed'])
plt.xlim([0, 9])
plt.show()

fig, ax = plt.subplots(figsize =(16, 9))
plt.grid(color='grey', linestyle='-.', linewidth=0.5)

plt.plot(time,mturning)
plt.plot(time,sturning)
plt.xlabel("Time [s]")
plt.ylabel("Turning Rate [scalar]")
plt.legend(['Input Turn Rate','Spot Turn Rate'])
plt.xlim([0, 9])
plt.ylim([-1.5, 1.5])

plt.show()    

fig, ax = plt.subplots(figsize =(16, 9))
plt.grid(color='grey', linestyle='-.', linewidth=0.5)

plt.plot(time,hangle)
plt.plot(time,forceangle)
plt.xlabel("Time [s]")
plt.ylabel("Angle [deg]")
plt.legend(['Commanded Heading','APF Resultant Force Angle'])
plt.xlim([0, 9])

plt.show() 
    
fig, ax = plt.subplots(figsize =(16, 16))

for i in range(len(lines)):
    color='b'
    dash = 'dashed'
    listax = []
    listay = []
    angle = lines[i][2][1]
    xd = 0.5*lines[i][3][0]
    yd = 0.5*lines[i][3][1]
    if abs(angle)==1.5708:
        aux=xd
        xd=yd
        yd=aux
    c1x = lines[i][1][0]-xd
    c1y = lines[i][1][1]+yd
    listax.append(c1x)
    listay.append(c1y)
    c1x = lines[i][1][0]+xd
    c1y = lines[i][1][1]+yd
    listax.append(c1x)
    listay.append(c1y)
    
    c1x = lines[i][1][0]+xd
    c1y = lines[i][1][1]-yd
    listax.append(c1x)
    listay.append(c1y)
    
    c1x = lines[i][1][0]-xd
    c1y = lines[i][1][1]-yd
    listax.append(c1x)
    listay.append(c1y)
    
    c1x = lines[i][1][0]-xd
    c1y = lines[i][1][1]+yd
    listax.append(c1x)
    listay.append(c1y)
    if lines[i][0]== 'Wall':
        color='k'
        dash='solid'
    if lines[i][0]== 'Window':
        color='grey'
        dash='dashdot'
    plt.plot(listax,listay,color,linestyle=dash)
    plt.xlim([-14, 4])
    plt.ylim([-14, 4])

plt.plot(posx,posy,color='red')
plt.show()    
    