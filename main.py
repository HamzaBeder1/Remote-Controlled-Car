import serial
import time as t
serialPort = serial.Serial(
   port="COM4", baudrate=38400, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE
)

def readData():
    serialLine = serialPort.readline()
    print(serialLine.decode('ascii'))

def main():
    while(1):
        direction = input()
        direction = direction.encode("ascii")
        serialPort.write(direction)
        t.sleep(2)

if __name__ == "__main__":
    main()