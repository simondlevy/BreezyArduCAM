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

SCALEDOWN = 1          # logarithm of 2 (e.g., SCALEDOWN=3 gives 1/8 width, 1/8 height)

BAUD = 921600   # 115200 for Due

# helpers  --------------------------------------------------------------------------

def num2bytes(n):
    return [n&0xFF, (n>>8)&0xFF, (n>>16)&0xFF, (n>>24)&0XFF]

def dump(msg):
    stdout.write(msg)
    stdout.flush()

# main ------------------------------------------------------------------------------

if __name__ == '__main__':

    # Base is 320x240 image
    w = 320 >> SCALEDOWN
    h = 240 >> SCALEDOWN

    # Create an empty grayscale image
    image = np.zeros((h, w)).astype('uint8')

    dump('Connecting to camera...')

    # Open connection to Arduino with a timeout of two seconds
    port = serial.Serial(PORT, BAUD, timeout=2)

    # Report acknowledgment from camera
    getack(port)

    # Wait a spell
    time.sleep(0.2)

    # Send "start capture" message
    sendbyte(port, 1)

    dump('\nStarting capture ...')

    while True:

        # Grab image bytes and put them in image array
        for j in range(h):
            for k in range(w):
                image[j,k] = ord(port.read())

        # Restart the capture
        sendbyte(port, 1)

        # Display the image
        cv2.imshow("ArduCAM", image)
        if cv2.waitKey(1) == 27:
            break

    sendbyte(port, 0)

