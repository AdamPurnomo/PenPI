#from pykalman import KalmanFilter
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import lfilter, lfilter_zi, filtfilt, butter
from mpl_toolkits.mplot3d import Axes3D

f=open("data.txt","r")
Time=list()
timestep=list()
AccX_Value=list()
AccY_Value=list()
AccZ_Value=list()
t=0
for line in f.readlines():
  x=line.split(',')
  #print x[0]
  t+=float(x[0])
  Time.append(t)
  timestep.append(float(x[0]))
  AccX_Value.append(float(x[1]))
  AccY_Value.append(float(x[2]))
  AccZ_Value.append(float(x[3]))
timestep=np.array(timestep)
AccX_Value=9.81*np.array(AccX_Value)
AccY_Value=9.81*np.array(AccY_Value)
AccZ_Value=9.81*np.array(AccZ_Value)

#IIR filter
b, a = butter(5, 0.0088,btype='high')
AccX_Value=filtfilt(b,a,AccX_Value)
#b, a = butter(5, 0.0088,btype='high')
AccY_Value=filtfilt(b,a,AccY_Value)
AccZ_Value=filtfilt(b,a,AccZ_Value)

pos=[0,0,0]
vec=[0,0,0]
posX_list=list()
posY_list=list()
posZ_list=list()

steps=AccX_Value.shape[0]
for i in range(steps):
	dt=timestep[i]
	pos[0]+=vec[0]*dt+0.5*AccX_Value[i]*dt**2
	posX_list.append(pos[0])
	vec[0]+=              AccX_Value[i]*dt
	pos[1]+=vec[1]*dt+0.5*AccY_Value[i]*dt**2
	posY_list.append(pos[1])
	vec[1]+=              AccY_Value[i]*dt
	pos[2]+=vec[2]*dt+0.5*AccZ_Value[i]*dt**2
	posZ_list.append(pos[2])
	vec[2]+=              AccZ_Value[i]*dt	

fig = plt.figure()
ax  = fig.gca(projection='3d')
ax.plot(posX_list,posY_list,posZ_list)
#plt.plot(posX_list,posY_list)
plt.show()