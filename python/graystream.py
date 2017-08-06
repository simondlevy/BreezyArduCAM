#!/usr/bin/env python3
'''
graystream.py : Use OpenCV to display streaming grayscale images from ArduCAM Mini

Copyright (C) Simon D. Levy 2017

This file is part of BreezyArduCAM.

BreezyArduCAM is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
BreezyArduCAM is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with BreezyArduCAM.  If not, see <http://www.gnu.org/licenses/>.
'''

import time
import serial
from sys import stdout
import numpy as np
import cv2

from helpers import *

# Modifiable params -------------------------------------------------------------------

PORT = '/dev/ttyACM0' # Ubuntu
#PORT = 'COM4'         # Windows

SCALEDOWN = 0          # logarithm of 2 (e.g., SCALEDOWN=3 gives 1/8 width, 1/8 height)

BAUD = 921600   # Arduino Uno

# helpers  --------------------------------------------------------------------------

def num2bytes(n):
    return [n&0xFF, (n>>8)&0xFF, (n>>16)&0xFF, (n>>24)&0XFF]

def dump(msg):
    stdout.write(msg)
    stdout.flush()

# main ------------------------------------------------------------------------------

if __name__ == '__main__':

    image = np.zeros((240>>SCALEDOWN, 320>>SCALEDOWN)).astype('uint8')

    # Open connection to Arduino with a timeout of two seconds
    port = serial.Serial(PORT, BAUD, timeout=2)

    dump('Starting capture ...')

    # Validate startup message
    ackcheck(port, 'SPI interface OK.')

    # Wait a spell
    time.sleep(0.2)

    # Send "start capture" message
    sendbyte(port, 1)

    # Read bytes from serial and write them to file
    for j in range(240>>SCALEDOWN):
        for k in range(320>>SCALEDOWN):
            bl = ord(port.read())  # low byte
            bh = ord(port.read())  # high byte
            rgb = (bh<<8)+bl
            r = (rgb & 0xF800) >> 11
            g = (rgb & 0x07E0) >> 5
            b = rgb & 0x001F
            image[j,k] = int(0.21*r + 0.72*g + 0.07*b)

    # Send "stop" message
    sendbyte(port, 0)

    print('\nDone')

    while True:

       cv2.imshow("ArduCAM", image)
       if cv2.waitKey(1) == 27:
           break

