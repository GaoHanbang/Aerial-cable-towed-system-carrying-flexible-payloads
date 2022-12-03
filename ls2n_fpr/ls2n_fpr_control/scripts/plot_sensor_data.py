#!/usr/bin/env python3
import csv
from tkinter import Tk
from tkinter import filedialog
import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt


def main():
    print("Selecting csv file...")
    root = Tk()
    root.withdraw()
    root.filename = filedialog.askopenfilename(initialdir = "/home/user/logs/fpr/sensor")
    print("File selected: ", root.filename)
    
    sensor_data = genfromtxt(root.filename, delimiter=',')
    plt.figure("Sensor Data")
    plt.plot(sensor_data[:,0], sensor_data[:,1])
    plt.grid()
   
    root.destroy()

    while(True):
        try:
            plt.show()
        except KeyboardInterrupt:
            print("Interrupt by user, closing all windows...")
        finally:
            plt.close()
            exit(1)

if __name__ == '__main__':
    main()
