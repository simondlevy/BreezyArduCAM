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

ArduCAM_Mini_2MP myCam(CS);

void setup(void) 
{
    // ArduCAM Mini uses both I^2C and SPI buses
    Wire.begin();
    SPI.begin();

    // Talk to Arduino at fastest possible baud rate
    Serial.begin(921600);

    // Change to QVGA capture mode and initialize the OV5642 module
    myCam.initQvga();
}

void loop(void) 
{
    static bool ready;
    static bool capturing;

    if (Serial.available()) {

        switch (Serial.read()) {
            case 1:
                ready = true;
                capturing = true;
                break;
            case 0:
                break;
            default:
                break;
        }
    }

    if (ready)
    {
        if (capturing)
        {
            //Flush the FIFO
            myCam.flush_fifo();
            myCam.clear_fifo_flag();
            //Start capture
            myCam.start_capture();
            capturing = false;
        }

        if (myCam.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
        {
            Serial.println("ACK CMD CAM Capture Done.");
            uint32_t length = 0;
            length = myCam.read_fifo_length();
            if (length >= MAX_FIFO_SIZE ) 
            {
                Serial.println("ACK CMD Over size.");
                myCam.clear_fifo_flag();
                return;
            }
            if (length == 0 ) //0 kb
            {
                Serial.println("ACK CMD Size is 0.");
                myCam.clear_fifo_flag();
                return;
            }
            myCam.CS_LOW();
            myCam.set_fifo_burst();//Set fifo burst mode

            SPI.transfer(0x00);
            char VH, VL;
            int i = 0, j = 0;
            for (i = 0; i < 240; i++)
            {
                for (j = 0; j < 320; j++)
                {
                    VH = SPI.transfer(0x00);;
                    VL = SPI.transfer(0x00);;
                    Serial.write(VL);
                    delayMicroseconds(12);
                    Serial.write(VH);
                    delayMicroseconds(12);
                }
            }
            Serial.write(0xBB);
            Serial.write(0xCC);
            myCam.CS_HIGH();

            //Clear the capture done flag
            myCam.clear_fifo_flag();
        }
    }
}
