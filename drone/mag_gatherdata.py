import serial
import numpy as np
import time

ser = serial.Serial(port='COM4', baudrate=115200)
mag_readings = []

start = time.time()

def removerns(L):
    return str(L.replace('\\r', ' ').replace('\\n', ' ').strip())

print("Begin rotating board in all directions")
while time.time() - start < 45:
    line = ser.readline()
    strline = str(line).strip('b\'').strip()
    if 'MAGNETOMETER' in strline:
        mvals = strline.strip('MAGNETOMETER:').split(',')
        mx = float(mvals[0].split(':')[1].strip())
        my = float(mvals[1].split(':')[1].strip())
        mz = float(removerns(mvals[2].split(':')[1].strip()))
        mxyz = np.array([mx, my, mz])
        mag_readings.append(mxyz)

mag_readings = np.array(mag_readings)
np.save("magdata.npy", mag_readings)
print("Saved", len(mag_readings), "magnetometer readings.")