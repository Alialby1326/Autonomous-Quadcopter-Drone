import serial
import numpy as np

ser = serial.Serial(port='COM4', baudrate=115200)

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
        #print('GYROSCOPE(dps): RollRate(x): ', gx, ' PitchRate(y): ', gy, ' YawRate(z): ', gz)

    elif 'ACCELEROMETER' in strline:
        avals = strline.strip('ACCELEROMETER:').split(',')
        ax = float(avals[0].split(':')[1].strip())
        ay = float(avals[1].split(':')[1].strip())
        az = float(removerns(avals[2].split(':')[1].strip().split()[0]))

        apitch = np.atan(-ax/(np.sqrt(ay * ay + az * az)))
        aroll = np.atan(ay/np.sqrt(ax * ax + az * az))

        print('Pitch(rads): ', apitch, 'Roll(rads): ', aroll)
        #print('ACCELEROMETER(g): X: ', ax, ' Y: ', ay, ' Z: ', az)

    elif 'MAGNETOMETER' in strline:
        mvals = strline.strip('MAGNETOMETER:').split(',')
        mx = float(mvals[0].split(':')[1].strip())
        my = float(mvals[1].split(':')[1].strip())
        mz = float(removerns(mvals[2].split(':')[1].strip()))
        #print('MAGNETOMETER: X: ', mx, ' Y: ', my, ' Z: ', mz)

        



    
    


    