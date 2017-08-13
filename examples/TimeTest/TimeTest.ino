/*
TimeTest.ino : frames-per-second capture test for BreezyArduCAM

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

#include <SPI.h>
#include <BreezyArduCAM.h>
#include <Wire.h>

class Timed_ArduCAM_FrameGrabber : public ArduCAM_FrameGrabber {

    protected:

        virtual bool gotStartRequest(void) override 
        {
            return true;
        }

        virtual bool gotStopRequest(void) override 
        {
            return false; // never stop
        }

        virtual void sendByte(uint8_t b) override 
        {
            (void)b; // ignore byte
        }
};


static const int CS = 10;


Timed_ArduCAM_FrameGrabber fg;
ArduCAM_Mini_2MP myCam(CS, &fg);

static uint32_t count;
static uint32_t start;

void setup(void) 
{
    // ArduCAM Mini uses both I^2C and SPI buses
    Wire.begin();
    SPI.begin();

    // Talk to Arduino 
    Serial.begin(115200);

    // Begin capturing in  QVGA mode
    myCam.beginQvga();

    // Start millisecond timing
    start = millis();
}

void loop(void) 
{
    // Capture a frame
    myCam.capture();

    // Compute and report frames per second

    count++;

    float elapsed = (millis() - start) / 1000.;

    if (elapsed > 0) {
        Serial.print(count/elapsed, 0);
        Serial.println(" FPS");
        Serial.flush();
    }
}
