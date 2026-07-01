import time
import zmq
import numpy as np

#publisher
context = zmq.Context()
publisher = context.socket(zmq.PUB)
publisher.bind("tcp://127.0.0.1:5555")

def removerns(L):
    return str(L.replace('\\r', ' ').replace('\\n', ' ').strip())

print("sending simulated data...")

while True:
    with open("sim_imu.log", "r") as sim:
        for line in sim:
            line = str(line).strip()
            if not line:
                continue

            if 'GYROSCOPE' in line:
                gvals = line.strip('GYROSCOPE:').split(',')
                gx = np.radians(float(gvals[0].split(':')[1].strip()))
                gy = np.radians(float(gvals[1].split(':')[1].strip()))
                gz = np.radians(float(removerns(gvals[2].split(':')[1].strip().split()[0])))
                try:
                    data = {
                        'gx': gx,
                        'gy': gy,
                        'gz': gz
                    }
                    publisher.send_string('GYRO', flags=zmq.SNDMORE)
                    publisher.send_json(data)
                except:
                    pass
            elif 'ACCELEROMETER' in line:
                avals = line.strip('ACCELEROMETER:').split(',')
                ax = float(avals[0].split(':')[1].strip())
                ay = float(avals[1].split(':')[1].strip())
                az = float(removerns(avals[2].split(':')[1].strip().split()[0]))
                try:
                    data = {
                        'ax': ax,
                        'ay': ay,
                        'az': az
                    }
                    publisher.send_string('ACCEL', flags=zmq.SNDMORE)
                    publisher.send_json(data)
                except:
                    pass
            elif 'MAGNETOMETER' in line:
                mvals = line.strip('MAGNETOMETER:').split(',')
                mx = float(mvals[0].split(':')[1].strip())
                my = float(mvals[1].split(':')[1].strip())
                mz = float(removerns(mvals[2].split(':')[1].strip().split()[0]))
                try:
                    data = {
                        'mx': mx,
                        'my': my,
                        'mz': mz
                    }
                    publisher.send_string('MAG', flags=zmq.SNDMORE)
                    publisher.send_json(data)
                except:
                    pass            
            #accounting for HAL_DELAY   
            time.sleep(0.1)