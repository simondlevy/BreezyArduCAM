/*
JPEGStream.ino : capture images and stream them to host as JPEG files

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

static const int CS = 10;

class SerialCam : public ArduCAM_Mini_2MP {

    public:

        SerialCam(uint8_t cs) : ArduCAM_Mini_2MP(cs) { }

    protected:

        virtual bool gotStartRequest(void) override 
        {
            return (Serial.available() && Serial.read());
        }

        virtual bool gotStopRequest(void) override 
        {
            return (Serial.available() && !Serial.read());
        }

        virtual void sendByte(uint8_t b) override 
        {
            Serial.write(b);
        }


};

SerialCam myCam(CS);

void setup(void) 
{
    // ArduCAM Mini uses both I^2C and SPI buses
    Wire.begin();
    SPI.begin();

    // Talk to Arduino at fastest possible baud rate
    Serial.begin(921600);

    // Start the camera in JPEG mode with a specific image size
    myCam.initJpeg320x240();
}

void loop(void) 
{
    myCam.captureJpeg();
}
