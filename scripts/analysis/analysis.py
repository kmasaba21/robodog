import os
import pickle
import numpy as np 
import matplotlib.pyplot as plt

from os.path import expanduser
from collections import Counter
from math import pi



'''

CODE THAT CREATES X,Y PLOT OF SCANNER DATA
ASSUME THAT ROBOT IS LOCATED AT (0,0)
RIGHT IS FROM 0 TO PI/2, LEFT IS FROM PI/2 TO PI


FOR EACH DATA POINT (SCANNER READING) WE CREATE A SEPARATE PLOT AND SCALE THE AXIS ACCORDINGLY
THE PLOTS ARE THEN USED TO GENERATE VIDEO USING video.py

'''

folder = os.path.dirname(os.path.abspath(__file__))
folder =  folder[:-len('scripts/analysis')] + 'data'

with open(folder + '/md_karim_4.pickle', 'rb') as f:
    data = pickle.load(f)



mintime = min([y['time'] for y in data])
times = [y['time'] - mintime for y in data]
cnt = Counter(times)


plt.figure()



def swap(x):
    if x != np.inf:
        return x
    else:
        return mx*2
oldt =  0
vel = 0

base_vel = 4

scale_y = 20


zz = [z['ranges'] for z in data]

zzz = zz[0]

angles = sorted(zzz.keys())
angles_proper = list(1/np.array(angles))


angles = [(x,y) for x,y in zip(angles, angles_proper)]

angles = sorted(angles, key=lambda xx: xx[1])


angles_to_print = [xx[0] for xx in angles]


scale_y_low = 0


glob_pos = 0
for ii in range(len(data)):

    print(ii)

    zzz = zz[ii]


    t = times[ii]

    dt = t - oldt

    vel = base_vel/cnt[t]



    glob_pos += vel

    print(glob_pos)


    values = []

    x = []
    y = []
    yy =[]
    bad_vals_x = []
    bad_vals_y = []
    for i, val in enumerate(angles_to_print):
        angle = pi/2 + pi/6 - i*(pi/180)

        valueee = zzz[val]


        if valueee != np.inf:
            x.append(valueee*np.cos(angle) )
            y.append(valueee*np.sin(angle) + glob_pos)



    if max(y) > scale_y:
        scale_y += 5
        scale_y_low = scale_y_low + 5


    plt.figure(ii)


    plt.scatter([-3,3], [scale_y, scale_y_low], c='g')

    plt.scatter(x, y, c='r')



    if not os.path.exists(folder + '/snaps/'):
        os.mkdir(folder + '/snaps/')

    plt.savefig(folder + '/snaps/snapshot_' + str(ii) +'.png')


    if not os.path.exists(folder + '/video/'):
        os.mkdir(folder + '/video/')

    if dt > 0:
        plt.savefig(folder + '/video/snapshot_' + str(ii) +'.png')


    oldt = t

