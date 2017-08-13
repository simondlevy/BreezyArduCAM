'''
helpers.py : Python helper functions for BreezyArduCAM

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

from sys import stdout

def getack(port):
    stdout.write(port.readline().decode())

def sendbyte(port, value):
    port.write(bytearray([value]))

def ackcheck(port, msg):
    line = port.readline().decode()
    assert(msg in line)
