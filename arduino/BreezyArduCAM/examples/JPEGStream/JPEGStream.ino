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

static bool starting;
static bool is_header;
static uint8_t tmp;
static uint8_t tmp_last;

void setup(void) 
{
    // ArduCAM Mini uses both I^2C and SPI buses
    Wire.begin();
    SPI.begin();

    // Talk to Arduino at fastest possible baud rate
    Serial.begin(921600);

    // Start the camera in JPEG mode with a specific image size
    myCam.initJpeg320x240();

    starting = true;
    tmp = 0xff;
}

void loop(void) 
{
    // Wait for start bit from host
    if (Serial.available() && Serial.read() == 1) {

        while (true) {

            // Check for halt bit from host
            if (Serial.available() && Serial.read() == 0) {
                return;
            }

            if (starting) {
                myCam.flush_fifo();
                myCam.clear_fifo_flag();
                myCam.start_capture();
                starting = false;
            }

            if (myCam.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
                uint32_t length = 0;
                length = myCam.read_fifo_length();
                if ((length >= MAX_FIFO_SIZE) | (length == 0)) {
                    myCam.clear_fifo_flag();
                    starting = true;
                }
                else {
                    myCam.csLow();
                    myCam.set_fifo_burst();
                    tmp =  SPI.transfer(0x00);
                    length --;
                    while (length--) {
                        tmp_last = tmp;
                        tmp =  SPI.transfer(0x00);
                        if (is_header) {
                            Serial.write(tmp);
                        }
                        else if ((tmp == 0xD8) & (tmp_last == 0xFF)) {
                            is_header = true;
                            Serial.write(tmp_last);
                            Serial.write(tmp);
                        }
                        if ((tmp == 0xD9) && (tmp_last == 0xFF)) 
                            break;
                        delayMicroseconds(15);
                    }
                    myCam.csHigh();
                    myCam.clear_fifo_flag();
                    starting = true;
                    is_header = false;
                }
            }
        }

    }
}
