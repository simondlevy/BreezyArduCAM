/*
BMPSnap.ino : grab a single frame and send it to host as a BMP file

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
*/

#include <Wire.h>
#include <BreezyArduCAM.h>
#include <SPI.h>

// set pin 10 as the slave select for the digital pot:
static const int CS = 10;

Serial_ArduCAM_FrameGrabber fg;

/* Choose your camera */
//ArduCAM_Mini_2MP myCam(CS, &fg);
ArduCAM_Mini_5MP myCam(CS, &fg);

void setup(void) 
{
    // ArduCAM Mini uses both I^2C and SPI buses
    Wire.begin();
    SPI.begin();

    // Fastest baud rate (change to 115200 for Due)
    Serial.begin(921600);

    // Begin capturing in  QVGA mode
    myCam.beginQvga();
}

void loop(void) 
{
    myCam.capture();
}
