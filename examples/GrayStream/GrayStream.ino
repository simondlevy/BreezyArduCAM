/*
GrayStream.ino : stream grayscale image bytes to host

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

// Use scale-down factor 2^2 = 4
static const uint8_t SCALEDOWN = 1;

// set pin 10 as the slave select for the digital pot:
static const int CS = 10;

Serial_ArduCAM_FrameGrabber fg;
ArduCAM_Mini_2MP myCam(CS, &fg);

void setup(void) 
{
    // ArduCAM Mini uses both I^2C and SPI buses
    Wire.begin();
    SPI.begin();

    // For Due use 115200
    Serial.begin(921600);

    // Begin capturing in  QVGA mode, scale-down factor 2^2 = 4, grayscale flagged
    myCam.beginQvga(SCALEDOWN, true);
}

void loop(void) 
{
    myCam.capture();
}
