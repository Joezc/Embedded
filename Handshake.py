import serial

ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)

def handshake():
    ser.write("hello")
    response = ser.readline()

def sendData(direction, distance):

ser.write("testing serial connection\n")
ser.write("sending via RPi\n")
try:
    while 1:
        response = ser.readline()
        print response
except KeyboardInterrupt:
    ser.close()
