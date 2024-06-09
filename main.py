import serial
import time as t
serialPort = serial.Serial(
   port="COM4", baudrate=38400, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE
)

def main():
    while(1):
        serialLine = serialPort.readline()
        print(serialLine.decode('ascii'))
        t.sleep(2)
        s = b"A"
        serialPort.write(s)

if __name__ == "__main__":
    main()