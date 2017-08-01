#!/usr/bin/env python3
'''
jpgstream.py : display live ArduCAM JPEG images using OpenCV

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
import cv2

# Modifiable params --------------------------------------------------------------------

PORT = '/dev/ttyACM0' # Ubuntu
#PORT = 'COM4'         # Windows

#BAUD = 115200  # Arduino Due
BAUD = 921600   # Arduino Uno

#SIZE = 0  # 160x120
#SIZE = 1  # 176x144
SIZE = 2  # 320x240
#SIZE = 3  # 352x288
#SIZE = 4  # 640x480
#SIZE = 5  # 800x600
#SIZE = 6  # 1024x768
#SIZE = 7  # 1280x960
#SIZE = 8  # 1600x1200

# Helpers ------------------------------------------------------------------------------

def ackcheck(port, msg):
    assert(msg in port.readline().decode())

def sendbyte(port, value):
    port.write(bytearray([value]))

# main ------------------------------------------------------------------------------

if __name__ == '__main__':

    # Open connection to Arduino
    port = serial.Serial(PORT, BAUD)

    # Validate startup messages
    ackcheck(port, 'SPI interface OK.')

    # Wait a spell
    time.sleep(0.2)

    # Request continuous read
    sendbyte(port, 0x20)

    # We'll report frames-per-second
    start = time.time()
    count = 0

    # Loop over images user hits ESC
    done = False
    while not done:

        # Open a temporary file that we'll write to and read from
        tmpfile = open("tmp.jpg", "wb")

        # Loop over bytes from Arduino for a single image
        written = False
        prevbyte = None
        while not done:

            # Read a byte from Arduino
            currbyte = port.read(1)

            # If we've already read one byte, we can check pairs of bytes
            if prevbyte:

                # Start-of-image sentinel bytes: write previous byte to temp file
                if ord(currbyte) == 0xd8 and ord(prevbyte) == 0xff:
                    tmpfile.write(prevbyte)
                    written = True

                # Inside image, write current byte to file
                if written:
                    tmpfile.write(currbyte)

                # End-of-image sentinel bytes: close temp file and display its contents
                if ord(currbyte) == 0xd9 and ord(prevbyte) == 0xff:
                    tmpfile.close()
                    img = cv2.imread("tmp.jpg")
                    cv2.imshow("ArduCAM [ESC to quit]", img)
                    if cv2.waitKey(1) == 27:
                        done = True
                        break
                    count += 1
                    break

            # Track previous byte
            prevbyte = currbyte

    # Report FPS
    elapsed = time.time() - start
    print('%d frames in %2.2f seconds = %2.2f frames per second' % (count, elapsed, count/elapsed))

    # Close the window
    cv2.destroyAllWindows()

    # End continuous read
    port.write(bytearray([0x21]))
