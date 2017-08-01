// ArduCAM Mini demo (C)2017 Lee
// Web: http://www.ArduCAM.com
// This program is a demo of how to use most of the functions
// of the library with ArduCAM Mini camera, and can run on any Arduino platform.
// This demo was made for ArduCAM_Mini_5MP_Plus.
// It needs to be used in combination with PC software.
// It can take photo continuously as video streaming.
//
// The demo sketch will do the following tasks:
// 1. Set the camera to JPEG output mode.
// 2. Read data from Serial port and deal with it
// 3. If receive 0x20,camera will capture JPEG photo and write datas continuously.Stop when receive 0x21.

#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>

static const int CS = 10;
static bool start_capture;

ArduCAM myCam(CS);

void setup(void) 
{
    uint8_t temp;

    Wire.begin();
    Serial.begin(921600);

    // set the CS as an output
    pinMode(CS, OUTPUT);

    // initialize SPI
    SPI.begin();

    while (true) {
        //Check if the ArduCAM SPI bus is OK
        myCam.write_reg(ARDUCHIP_TEST1, 0x55);
        temp = myCam.read_reg(ARDUCHIP_TEST1);
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
