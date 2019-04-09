#!/usr/bin/env python
# Log data from serial port

# Author: Diego Herranz

import argparse
import serial
import datetime
import time
import os

parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("-d", "--device", help="device to read from", default="/dev/ttyUSB0")
parser.add_argument("-s", "--speed", help="speed in bps", default=9600, type=int)
args = parser.parse_args()

outputFilePath = os.path.join(os.path.dirname(__file__),
                 datetime.datetime.now().strftime("%Y-%m-%dT%H.%M.%S") + ".txt")

with serial.Serial(args.device, args.speed) as ser, open(outputFilePath, mode='wb') as outputFile:
    print("Logging started. Ctrl-C to stop.") 
    try:
        while True:
            time.sleep(1)
            outputFile.write((ser.read(ser.inWaiting())))
            outputFile.flush()
    except KeyboardInterrupt:
        print("Logging stopped")
