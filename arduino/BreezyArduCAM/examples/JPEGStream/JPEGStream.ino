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
static bool start_capture;

ArduCAM_Mini_2MP myCam(CS);

void setup(void) 
{
    // ArduCAM Mini uses both I^2C and SPI buses
    Wire.begin();
    SPI.begin();

    // Talk to Arduino at fastest possible baud rate
    Serial.begin(921600);

    //Check if the ArduCAM SPI bus is OK
    while (true) {
        myCam.write_reg(ARDUCHIP_TEST1, 0x55);
        uint8_t temp = myCam.read_reg(ARDUCHIP_TEST1);
        if (temp != 0x55) {
            Serial.println("ACK CMD SPI interface Error!");
            delay(1000);
            continue;
        } else{
            Serial.println("ACK CMD SPI interface OK.");
            break;
        }
    }

    // Change to JPEG capture mode and initialize the OV5642 module
    myCam.initJpeg();

    myCam.OV2640_set_JPEG_size(OV2640_320x240);

    delay(1000);
    myCam.clear_fifo_flag();
}

void loop(void) 
{
    uint8_t temp = 0xff, temp_last = 0;
    bool is_header = false;
    static bool capturing;

    if (Serial.available() && Serial.read() == 0x20) {
        capturing = true;
        temp = 0xff;
        start_capture = true;
    }

    if (capturing)
    {
        while (true) {

            // Check for halt-capture command
            if (Serial.available() && Serial.read() == 0x21) {
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
