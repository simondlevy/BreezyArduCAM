// ArduCAM Mini demo (C)2017 Lee
// Web: http://www.ArduCAM.com
// This program is a demo of how to use most of the functions
// of the library with ArduCAM Mini 5MP camera, and can run on any Arduino platform.
// This demo was made for ArduCAM_Mini_5MP_Plus.
// It needs to be used in combination with PC software.
// It can take photo continuously as video streaming.
//
// The demo sketch will do the following tasks:
// 1. Set the camera to JPEG output mode.
// 2. Read data from Serial port and deal with it
// 3. If receive 0x00-0x08,the resolution will be changed.
// 4. If receive 0x10,camera will capture a JPEG photo and buffer the image to FIFO.Then write datas to Serial port.
// 5. If receive 0x20,camera will capture JPEG photo and write datas continuously.Stop when receive 0x21.
// 6. If receive 0x30,camera will capture a BMP  photo and buffer the image to FIFO.Then write datas to Serial port.
// 7. If receive 0x11 ,set camera to JPEG output mode.
// 8. If receive 0x31 ,set camera to BMP  output mode.
// This program requires the ArduCAM V4.0.0 (or later) library and ArduCAM_Mini_5MP_Plus
// and use Arduino IDE 1.6.8 compiler or above

#include <Wire.h>
#include <SPI.h>

#include <BreezyArduCAM.h>

#include "ov5642_regs.h"

const int CS = 10;
int mode = 0;
uint8_t start_capture = 0;

ArduCAM_Mini_5MP myCam(CS);

void setup() {

    uint8_t vid, pid;
    uint8_t temp;

    Wire.begin();
    Serial.begin(115200);

    pinMode(CS, OUTPUT);

    SPI.begin();

    while(1){
        //Check if the ArduCAM SPI bus is OK
        myCam.write_reg(ARDUCHIP_TEST1, 0x55);
        temp = myCam.read_reg(ARDUCHIP_TEST1);
        if(temp != 0x55)
        {
            Serial.println(F("ACK CMD SPI interface Error!"));
            delay(1000);continue;
        }else{
            Serial.println(F("ACK CMD SPI interface OK."));break;
        }
    }

    while(1){
        //Check if the camera module type is OV5642
        myCam.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
        myCam.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
        if ((vid != 0x56) || (pid != 0x42)){
            Serial.println(F("ACK CMD Can't find OV5642 module!"));
            delay(1000);continue;
        }else{
            Serial.println(F("ACK CMD OV5642 detected."));break;      
        }
    }

    //Change to JPEG capture mode and initialize the OV5642 module
    myCam.set_format(JPEG);
    myCam.begin();
    myCam.set_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
    myCam.clear_fifo_flag();
    myCam.write_reg(ARDUCHIP_FRAMES, 0x00);
}

void loop() {

    uint8_t temp= 0xff, temp_last =0;

    bool is_header = false;

    if (Serial.available())
    {
        temp = Serial.read();
        switch (temp)
        {
            case 0:
                myCam.setJpegSize(OV5642_320x240);delay(1000);
                Serial.println(F("ACK CMD switch to OV5642_320x240"));
                temp=0xff;
                break;
            case 1:
                myCam.setJpegSize(OV5642_640x480);delay(1000);
                Serial.println(F("ACK CMD switch to OV5642_640x480"));
                temp=0xff;
                break;
            case 2:
                myCam.setJpegSize(OV5642_1024x768);delay(1000);
                Serial.println(F("ACK CMD switch to OV5642_1024x768"));
                temp=0xff;
                break;
            case 3:
                myCam.setJpegSize(OV5642_1280x960);delay(1000);
                Serial.println(F("ACK CMD switch to OV5642_1280x960"));
                temp=0xff;
                break;
            case 4:
                myCam.setJpegSize(OV5642_1600x1200);delay(1000);
                Serial.println(F("ACK CMD switch to OV5642_1600x1200"));
                temp=0xff;
                break;
            case 5:
                myCam.setJpegSize(OV5642_2048x1536);delay(1000);
                Serial.println(F("ACK CMD switch to OV5642_2048x1536"));
                temp=0xff;
                break;
            case 6:
                myCam.setJpegSize(OV5642_2592x1944);delay(1000);
                Serial.println(F("ACK CMD switch to OV5642_2592x1944"));
                temp=0xff;
                break;
            case 0x10:
                mode = 1;
                start_capture = 1;
                Serial.println(F("ACK CMD CAM start single shoot."));
                break;
            case 0x11:
                myCam.set_format(JPEG);
                myCam.begin();
                myCam.set_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
                break;
            case 0x20:
                mode = 2;
                start_capture = 2;
                Serial.println(F("ACK CMD CAM start video streaming."));
                break;
            case 0x30:
                mode = 3;
                temp = 0xff;
                start_capture = 3;
                Serial.println(F("CAM start single shoot."));
                break;
            case 0x31:
                temp = 0xff;
                myCam.set_format(BMP);
                myCam.begin();      
                myCam.clear_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
                myCam.wrSensorReg16_8(0x3818, 0x81);
                myCam.wrSensorReg16_8(0x3621, 0xA7);
                break;
            default:
                break;
        }
    }

    if (mode == 1)
    {
        if (start_capture == 1)
        {
            myCam.flush_fifo();
            myCam.clear_fifo_flag();
            //Start capture
            myCam.start_capture();
            start_capture = 0;
        }
        if (myCam.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
        {
            Serial.println(F("CAM Capture Done."));
            myCam.read_fifo_burst(is_header);
            is_header = false;
            //Clear the capture done flag
            myCam.clear_fifo_flag();
        }
    }
    else if (mode == 2)
    {
        while (1)
        {
            temp = Serial.read();
            if (temp == 0x21)
            {
                start_capture = 0;
                mode = 0;
                Serial.println(F("ACK CMD CAM stop video streaming."));
                break;
            }
            if (start_capture == 2)
            {
                myCam.flush_fifo();
                myCam.clear_fifo_flag();
                //Start capture
                myCam.start_capture();
                start_capture = 0;
            }
            if (myCam.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
            {
                uint32_t length = 0;
                length = myCam.read_fifo_length();
                if ((length >= MAX_FIFO_SIZE) | (length == 0))
                {
                    myCam.clear_fifo_flag();
                    start_capture = 2;
                    continue;
                }
                myCam.csLow();
                myCam.set_fifo_burst();//Set fifo burst mode
                while ( length-- )
                {
                    temp_last = temp;
                    temp =  SPI.transfer(0x00);
                    if (is_header == true)
                    {
                    }
                    else if ((temp == 0xD8) & (temp_last == 0xFF))
                    {
                        is_header = true;
                        Serial.println(F("ACK IMG"));
                    }
                    if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
                        break;
                    delayMicroseconds(15);
                }
                myCam.csHigh();
                myCam.clear_fifo_flag();
                start_capture = 2;
                is_header = false;
            }
        }
    }
    else if (mode == 3)
    {
        if (start_capture == 3)
        {
            //Flush the FIFO
            myCam.flush_fifo();
            myCam.clear_fifo_flag();
            //Start capture
            myCam.start_capture();
            start_capture = 0;
        }
        if (myCam.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
        {
            Serial.println(F("ACK CMD CAM Capture Done."));
            uint8_t temp, temp_last;
            uint32_t length = 0;
            length = myCam.read_fifo_length();
            if (length >= MAX_FIFO_SIZE ) 
            {
                Serial.println(F("ACK CMD Over size."));
                myCam.clear_fifo_flag();
                return;
            }
            if (length == 0 ) //0 kb
            {
                Serial.println(F("ACK CMD Size is 0."));
                myCam.clear_fifo_flag();
                return;
            }
            myCam.csLow();
            myCam.set_fifo_burst();//Set fifo burst mode
            char VH, VL;
            int i = 0, j = 0;
            for (i = 0; i < 240; i++)
            {
                for (j = 0; j < 320; j++)
                {
                    VH = SPI.transfer(0x00);;
                    VL = SPI.transfer(0x00);;
                    //Serial.write(VL);
                    delayMicroseconds(12);
                    //Serial.write(VH);
                    delayMicroseconds(12);
                }
            }
            //Serial.write(0xBB);
            //Serial.write(0xCC);
            myCam.csHigh();
            //Clear the capture done flag
            myCam.clear_fifo_flag();
        }
    }
}

