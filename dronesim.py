import serial

ser = serial.Serial(port='COM4', baudrate=115200)

while True:
    line = ser.readline()
    strline = str(line)
    print(strline)

