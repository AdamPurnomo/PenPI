import sys, getopt

sys.path.append('.')
import RTIMU
import os.path
import time
import math
import numpy as np

SETTINGS_FILE = "RTIMULib"

print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
  print("Settings file does not exist, will be created")

s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)

print("IMU Name: " + imu.IMUName())

if (not imu.IMUInit()):
    print("IMU Init Failed")
    sys.exit(1)
else:
    print("IMU Init Succeeded")

def normalize(v, tolerance=0.00001):
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = sqrt(mag2)
        v = tuple(n / mag for n in v)
    return v

def norm(v):
    mag2 = sum(n * n for n in v)
    return sqrt(mag2)

def q_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z

def q_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)

def qv_mult(q1, v1):
    #v1 = normalize(v1)
    v1= tuple(n for n in v1)
    q2 = (0.0,) + v1
    return q_mult(q_mult(q1, q2), q_conjugate(q1))[1:]

# this is a good time to set any fusion parameters

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

poll_interval = imu.IMUGetPollInterval()
print("Recommended Poll Interval: %dmS\n" % poll_interval)

accel_offset=[0,0,0]
init_QPose  =[0,0,0,0]

i=0
while i<100:
  if imu.IMURead():
    i+=1
    data = imu.getIMUData()
    accel= data["accel"]
    #print accel
    QPose= data["fusionQPose"]
    for j in range(3):
	accel_offset[j]+=0.01*accel[j]
        init_QPose[j]+= 0.01* QPose[j]
    init_QPose[3]+=0.01*QPose[3]
    time.sleep(0.004)
rot_accel_offset=qv_mult(init_QPose,accel_offset)


last=time.time()

while True:
  now=time.time()
  dt=now-last
  if dt>0.004999:
    last=now
    imu.IMURead()
    # x, y, z = imu.getFusionData()
    # print("%f %f %f" % (x,y,z))
    data = imu.getIMUData()
    fusionQPose = data["fusionQPose"]
    #print fusionQPose
    #print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]), 
       # math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
    #print accel_offset
    accel = [ x for x in data["accel"]]
    rot_accel=[x for x in qv_mult(fusionQPose,accel)]

    for i in range(3):
        rot_accel[i]-=rot_accel_offset[i]

    print accel
    print rot_accel
    print dt
    #time.sleep(poll_interval*1.0/1000.0)
