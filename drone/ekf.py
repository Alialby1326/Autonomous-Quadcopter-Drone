import serial
import numpy as np

ser = serial.Serial(port='COM4', baudrate=115200)

gbias = np.array([0.0, 0.0, 0.0], dtype=float)

mag_offset = np.load("magOffset.npy")
mag_scale = np.load("magScale.npy")

gxyz = None
axyz = None
raw_mxyz = None


def removerns(L):
    return str(L.replace('\\r', ' ').replace('\\n', ' ').strip())


ser.reset_input_buffer()
while True:
    line = ser.readline()
    strline = str(line).strip('b\'').strip()
    if 'GYROSCOPE' in strline:
        gvals = strline.strip('GYROSCOPE:').split(',')
        gx = np.radians(float(gvals[0].split(':')[1].strip()))
        gy = np.radians(float(gvals[1].split(':')[1].strip()))
        gz = np.radians(float(removerns(gvals[2].split(':')[1].strip().split()[0])))
        gxyz = np.array([gx, gy, gz])
        #print('GYROSCOPE: X: ', gx, ' Y: ', gy, ' Z: ', gz)
    elif 'ACCELEROMETER' in strline:
        avals = strline.strip('ACCELEROMETER:').split(',')
        ax = float(avals[0].split(':')[1].strip())
        ay = float(avals[1].split(':')[1].strip())
        az = float(removerns(avals[2].split(':')[1].strip().split()[0]))
        axyz = np.array([ax, ay, az])
        #print('ACCELEROMETER: X: ', ax, ' Y: ', ay, ' Z: ', az)
    elif 'MAGNETOMETER' in strline:
        mvals = strline.strip('MAGNETOMETER:').split(',')
        mx = float(mvals[0].split(':')[1].strip())
        my = float(mvals[1].split(':')[1].strip())
        mz = float(removerns(mvals[2].split(':')[1].strip()))
        raw_mxyz = np.array([mx, my, mz])
        mxyz = (raw_mxyz - mag_offset) * mag_scale
        #print("raw:", raw_mxyz)
        #print("calibrated", mxyz)
        #print('MAGNETOMETER: X: ', mx, ' Y: ', my, ' Z: ', mz)

    
    


    