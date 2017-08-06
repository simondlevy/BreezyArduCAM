#!/usr/bin/env python3
'''
bmpsnap.py : save a single frame of BMP data from Arducam Mini

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

from helpers import *

# Modifiable params -------------------------------------------------------------------

PORT = '/dev/ttyACM0' # Ubuntu
#PORT = 'COM4'         # Windows

SCALEDOWN = 0          # logarithm of 2 (e.g., SCALEDOWN=3 gives 1/8 width, 1/8 height)

BAUD = 921600   # Arduino Uno

OUTFILENAME = 'test.bmp'

# BMP header for 320x240 imagel ------------------------------------------------------

header = [
    0x42, 0x4D, 0x36, 0x58, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x00, 0x28, 0x00,
    0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x58, 0x02, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x1F, 0x00,
    0x00, 0x00
]

# helpers  --------------------------------------------------------------------------

def dump(msg):
    stdout.write(msg)
    stdout.flush()

# main ------------------------------------------------------------------------------

if __name__ == '__main__':

    # Open connection to Arduino with a timeout of two seconds
    port = serial.Serial(PORT, BAUD, timeout=2)

    dump('Starting capture ...')

    # Validate startup message
    ackcheck(port, 'SPI interface OK.')

    # Wait a spell
    time.sleep(0.2)

    # Send "start capture" message
    sendbyte(port, 1)

    dump('\nWriting file %s (will appear upside-down) ...' % OUTFILENAME)

    # Open output file
    outfile = open(OUTFILENAME, 'wb')

    # Write BMP header
    outfile.write(bytearray(header))

    # Read bytes from serial and write them to file
    for k in range((320>>SCALEDOWN)*(240>>SCALEDOWN)*2):
        c = outfile.write(port.read())

    # Send "stop" message
    sendbyte(port, 0)

    # Close output file
    outfile.close()

    print('\nDone')

