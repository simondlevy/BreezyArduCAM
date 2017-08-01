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

ArduCAM_Mini_2MP myCam(CS);

void setup(void) 
{
    // ArduCAM Mini uses both I^2C and SPI buses
    Wire.begin();
    SPI.begin();

    // Talk to Arduino at fastest possible baud rate
    Serial.begin(921600);

    // Start the camera in JPEG mode with a specific image size
    myCam.initJpeg640x480();
}

void loop(void) 
{
    uint8_t temp = 0xff, temp_last = 0;
    bool is_header = false;

    static bool start_capture;
    static bool capturing;

    if (Serial.available() && Serial.read() == 1) {
        capturing = true;
        temp = 0xff;
        start_capture = true;
    }

    if (capturing)
    {
        while (true) {

            // Check for halt-capture command
            if (Serial.available() && Serial.read() == 0) {
                start_capture = false;
                capturing = false;
                break;
            }

            if (start_capture) {
                myCam.flush_fifo();
                myCam.clear_fifo_flag();
                //Start capture
                myCam.start_capture();
                start_capture = false;
            }

            if (myCam.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
                uint32_t length = 0;
                length = myCam.read_fifo_length();
                if ((length >= MAX_FIFO_SIZE) | (length == 0))
                {
                    myCam.clear_fifo_flag();
                    start_capture = true;
                    continue;
                }
                myCam.CS_LOW();
                myCam.set_fifo_burst();//Set fifo burst mode
                temp =  SPI.transfer(0x00);
                length --;
                while (length--) {
                    temp_last = temp;
                    temp =  SPI.transfer(0x00);
                    if (is_header) {
                        Serial.write(temp);
                    }
                    else if ((temp == 0xD8) & (temp_last == 0xFF)) {
                        is_header = true;
                        Serial.write(temp_last);
                        Serial.write(temp);
                    }
                    if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
                        break;
                    delayMicroseconds(15);
                }
                myCam.CS_HIGH();
                myCam.clear_fifo_flag();
                start_capture = true;
                is_header = false;
            }
        }
    }
}
