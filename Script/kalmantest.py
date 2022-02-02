from pykalman import KalmanFilter
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import lfilter, lfilter_zi, filtfilt, butter

f=open("data.txt","r")
Time=list()
timestep=list()
AccX_Value=list()
AccY_Value=list()
t=0
for line in f.readlines():
  x=line.split(',')
  #print x[0]
  t+=float(x[0])
  Time.append(t)
  timestep.append(float(x[0]))
  AccX_Value.append(float(x[1]))
  AccY_Value.append(float(x[2]))
timestep=np.array(timestep)
AccX_Value=np.array(AccX_Value)
AccY_Value=np.array(AccY_Value)
AccX_Value_raw=AccX_Value
AccY_Value_raw=AccY_Value

#IIR filter
b, a = butter(5, 0.0088,btype='high')
AccX_Value=filtfilt(b,a,AccX_Value)
AccY_Value=filtfilt(b,a,AccY_Value)
#end
#the rest of the code is kalman filter and plot

print timestep
print AccX_Value
# Data description
#  Time
#  AccX_HP - high precision acceleration signal
#  AccX_LP - low precision acceleration signal
#  RefPosX - real position (ground truth)
#  RefVelX - real velocity (ground truth)

# switch between two acceleration signals
use_HP_signal = 0

if use_HP_signal:
    #AccX_Value = AccX_HP
    AccX_Variance = 0.0007
else:    
    #AccX_Value = AccX_LP
    AccX_Variance = 10e-5


# time step
dt = 0.005

# transition_matrix  
F = [[1, dt, 0.5*dt**2], 
     [0,  1,       dt],
     [0,  0,        1]]

# observation_matrix   
H = [0, 0, 1]

# transition_covariance 
Q = [[  0,   0,  0], 
     [  0,   0,  0],
     [  0,   0,  10e-5]]

# observation_covariance 
R = AccX_Variance

# initial_state_mean
X0 = [0,
      0,
      0]

# initial_state_covariance
P0 = [[  0,    0,               0], 
      [  0,    0,               0],
      [  0,    0,   AccX_Variance]]

n_timesteps = AccX_Value.shape[0]
n_dim_state = 3
filtered_state_means = np.zeros((n_timesteps, n_dim_state))
filtered_state_covariances = np.zeros((n_timesteps, n_dim_state, n_dim_state))

kf = KalmanFilter(transition_matrices = F, 
                  observation_matrices = H, 
                  transition_covariance = Q, 
                  observation_covariance = R, 
                  initial_state_mean = X0, 
                  initial_state_covariance = P0)

# iterative estimation for each new measurement
for t in range(n_timesteps):
    if t == 0:
        filtered_state_means[t] = X0
        filtered_state_covariances[t] = P0
    else:
    	dt=timestep[t]
    	F = [[1, dt, 0.5*dt**2], 
    		[0,  1,       dt],
     		[0,  0,        1]]
        filtered_state_means[t], filtered_state_covariances[t] = (
        kf.filter_update(
            filtered_state_means[t-1],
            filtered_state_covariances[t-1],
            AccX_Value[t]
        )
    )


f, axarr = plt.subplots(6, sharex=True)
#Time=[0.005*x for x in range(0,n_timesteps)]
axarr[0].plot(Time, AccX_Value_raw, label="Input AccX")
axarr[0].plot(Time, filtered_state_means[:, 2], "r-", label="Estimated AccX")
axarr[0].set_title('Acceleration X')
axarr[0].grid()
axarr[0].legend()
#axarr[0].set_ylim([-1, 1])

#axarr[1].plot(Time, RefVelX, label="Reference VelX")
axarr[1].plot(Time, filtered_state_means[:, 1], "r-", label="Estimated VelX")
axarr[1].set_title('Velocity X')
axarr[1].grid()
axarr[1].legend()
#axarr[1].set_ylim([-0.05, 0.05])

#axarr[2].plot(Time, RefPosX, label="Reference PosX")
axarr[2].plot(Time, filtered_state_means[:, 0], "r-", label="Estimated PosX")
axarr[2].set_title('Position X')
axarr[2].grid()
axarr[2].legend()
#axarr[2].set_ylim([-0.03, 0.03])

for t in range(n_timesteps):
    if t == 0:
        filtered_state_means[t] = X0
        filtered_state_covariances[t] = P0
    else:
      dt=timestep[t]
      F = [[1, dt, 0.5*dt**2], 
        [0,  1,       dt],
        [0,  0,        1]]
      filtered_state_means[t], filtered_state_covariances[t] = (
      kf.filter_update(
            filtered_state_means[t-1],
            filtered_state_covariances[t-1],
            AccY_Value[t]
        )
    )

axarr[3].plot(Time, AccY_Value_raw, label="Input AccY")
axarr[3].plot(Time, filtered_state_means[:, 2], "r-", label="Estimated AccY")
axarr[3].set_title('Acceleration Y')
axarr[3].grid()
axarr[3].legend()
#axarr[3].set_ylim([-1, 1])

#axarr[1].plot(Time, RefVelX, label="Reference VelX")
axarr[4].plot(Time, filtered_state_means[:, 1], "r-", label="Estimated VelY")
axarr[4].set_title('Velocity Y')
axarr[4].grid()
axarr[4].legend()
#axarr[4].set_ylim([-0.05, 0.05])

#axarr[2].plot(Time, RefPosX, label="Reference PosX")
axarr[5].plot(Time, filtered_state_means[:, 0], "r-", label="Estimated PosY")
axarr[5].set_title('Position Y')
axarr[5].grid()
axarr[5].legend()
#axarr[5].set_ylim([-0.03, 0.03])

plt.show()