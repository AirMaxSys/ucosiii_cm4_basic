import sys
import glob
import serial
import serial.tools.list_ports

def ser_ports():
    ports_info = serial.tools.list_ports.comports()
    idx = 1
    port_coms = []
    for port in ports_info:
        port_coms.append(str(port).split()[0])
        print(f"{idx} | {port}")
        idx += 1
    return port_coms

def main():
    coms = ser_ports()
    sel_idx = int(input("select ports idndex:"))
    dev_com = coms[sel_idx - 1]
    ser = serial.Serial(dev_com, 9600)
    while True:
        print(ser.readline().decode("utf-8"), end='')

if __name__ == "__main__":
    main()
