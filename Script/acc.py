#!/usr/bin/python -u
# -*- coding: utf-8 -*-

import smbus
import time
from math import sin,cos,sqrt
import numpy as np
from pykalman import KalmanFilter,UnscentedKalmanFilter

channel=1
address = 0x68
bus     = smbus.SMBus(channel)

def u2s(unsigneddata):
    if unsigneddata & (0x01 << 15) :
        return -1 * ((unsigneddata ^ 0xffff) + 1)
    return unsigneddata

def normalize(v, tolerance=0.00001):
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = sqrt(mag2)
        v = tuple(n / mag for n in v)
    return v

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

bus.write_i2c_block_data(address, 0x6B, [0x80])
time.sleep(0.1)

bus.write_i2c_block_data(address, 0x6B, [0x00])
time.sleep(0.1)

bus.write_i2c_block_data(address, 0x1C, [0x08])

print "Accel calibration start"
_sum    = [0,0,0]


for _i in range(1000):
    data    = bus.read_i2c_block_data(address, 0x3B ,6)
    _sum[0] += (8.0 / float(0x8000)) * u2s(data[0] << 8 | data[1])
    _sum[1] += (8.0 / float(0x8000)) * u2s(data[2] << 8 | data[3])
    _sum[2] += (8.0 / float(0x8000)) * u2s(data[4] << 8 | data[5])

offsetAccelX    = -1.0 * _sum[0] / 1000
offsetAccelY    = -1.0 * _sum[1] / 1000
offsetAccelZ    = -1.0 * _sum[2] / 1000
print "Accel calibration complete"

bus.write_i2c_block_data(address, 0x1B, [0x10])

gyroCoefficient = 1000 / float(0x8000)

print "Gyro calibration start"
_sum    = [0,0,0]

for _i in range(1000):
    data    = bus.read_i2c_block_data(address, 0x43 ,6)
    _sum[0] += gyroCoefficient * u2s(data[0] << 8 | data[1])
    _sum[1] += gyroCoefficient * u2s(data[2] << 8 | data[3])
    _sum[2] += gyroCoefficient * u2s(data[4] << 8 | data[5])

offsetGyroX     = -1.0 * _sum[0] / 1000
offsetGyroY     = -1.0 * _sum[1] / 1000
offsetGyroZ     = -1.0 * _sum[2] / 1000
print "Gyro calibration complete"

pos=[0,0,0]
vec=[0,0,0]
start_t=time.time()
d_teta=[0,0,0]
sin_ang=[0,0,0]
cos_ang=[0,0,0]
Qn=[1,0,0,0]

kfX = KalmanFilter([1],[1])
kfY = KalmanFilter([1],[1])
kfZ = KalmanFilter([1],[1])

accX_fil,accX_cov,accY_fil,accY_cov,accZ_fil,accZ_cov=[0],[0],[0],[0],[0],[0]

while True:
    data1   = bus.read_i2c_block_data(address, 0x3B ,6)
    data2   = bus.read_i2c_block_data(address, 0x43 ,6)
    end_t   = time.time()
    time_dif= end_t-start_t
    start_t = end_t
    rawX    = (8.0 / float(0x8000)) * u2s(data1[0] << 8 | data1[1]) + offsetAccelX
    rawY    = (8.0 / float(0x8000)) * u2s(data1[2] << 8 | data1[3]) + offsetAccelY
    rawZ    = (8.0 / float(0x8000)) * u2s(data1[4] << 8 | data1[5]) + offsetAccelZ
    accel   = [9.81*rawX,9.81*rawY,9.81*rawZ]
    rawX    = gyroCoefficient * u2s(data2[0] << 8 | data2[1]) + offsetGyroX
    rawY    = gyroCoefficient * u2s(data2[2] << 8 | data2[3]) + offsetGyroY
    rawZ    = gyroCoefficient * u2s(data2[4] << 8 | data2[5]) + offsetGyroZ
    ang_v   = [rawX,rawY,rawZ]
    output  = accel + ang_v

    for i in range(3):
	d_teta[i]= ang_v[i]*time_dif
	sin_ang[i]=sin(0.5*d_teta[i])
	cos_ang[i]=cos(0.5*d_teta[i])

    dQ=[cos_ang[0]*cos_ang[1]*cos_ang[2],
	sin_ang[0]*cos_ang[1]*cos_ang[2],
	cos_ang[0]*sin_ang[1]*cos_ang[2],
	cos_ang[0]*cos_ang[1]*sin_ang[2]]
    Qn=q_mult(Qn,normalize(dQ))
    accel_rot = qv_mult(Qn,accel)
    #Get filtered acceleration
    #accX_fil,accX_cov= kfX.filter_update(accX_fil,accX_cov,accel_rot[0])
    #accY_fil,accY_cov= kfY.filter_update(accY_fil,accY_cov,accel_rot[1])
    #accZ_fil,accZ_cov= kfZ.filter_update(accZ_fil,accZ_cov,accel_rot[2])
    #accel_fil=np.concatenate((accX_fil,accY_fil,accZ_fil))
    #print accel_fil


    for i in range(3):
	# replace accel_rot by accel_fil to use filtered acceleration
        pos[i]+=vec[i]*time_dif+0.5*accel_rot[i]*time_dif**2
        vec[i]+=accel_rot[i]*time_dif
    print pos
