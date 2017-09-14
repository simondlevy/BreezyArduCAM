/*
BreezyArduCAM.cpp : class implementations for BreezyArduCAM libary

Copyright (C) Simon D. Levy 2017

This code borrows heavily from https://github.com/ArduCAM/Arduino/tree/master/ArduCAM

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

#include "BreezyArduCAM.h"

#include "ov2640_regs.h"
#include "ov5642_regs.h"

#include <Wire.h>
#include <SPI.h>

#if defined(__SAM3X8E__)
#define Wire Wire1
#endif

#if defined (__AVR__)
#define cbi(reg, bitmask) *reg &= ~bitmask
#define sbi(reg, bitmask) *reg |= bitmask
#define pulse_high(reg, bitmask) sbi(reg, bitmask); cbi(reg, bitmask);
#define pulse_low(reg, bitmask) cbi(reg, bitmask); sbi(reg, bitmask);
#define cport(port, data) port &= data
#define sport(port, data) port |= data
#define swap(type, i, j) {type t = i; i = j; j = t;}
#define fontbyte(x) pgm_read_byte(&cfont.font[x])  
#define regtype volatile uint8_t
#define regsize uint8_t

#else

#define cbi(reg, bitmask) *reg &= ~bitmask
#define sbi(reg, bitmask) *reg |= bitmask

#define pulse_high(reg, bitmask) sbi(reg, bitmask); cbi(reg, bitmask);
#define pulse_low(reg, bitmask) cbi(reg, bitmask); sbi(reg, bitmask);

#define cport(port, data) port &= data
#define sport(port, data) port |= data

#define swap(type, i, j) {type t = i; i = j; j = t;}
#define fontbyte(x) cfont.font[x]  

#define regtype volatile uint32_t
#define regsize uint32_t

#define PROGMEM

#endif


/****************************************************/
/* ArduChip registers definition    	            */
/****************************************************/

#define ARDUCHIP_TEST1       	0x00    // TEST register

#define ARDUCHIP_FIFO      		0x04    // FIFO and I2C control
#define FIFO_CLEAR_MASK    		0x01
#define FIFO_START_MASK    		0x02

#define ARDUCHIP_TRIG      		0x41    // Trigger source
#define CAP_DONE_MASK      		0x08

#define BURST_FIFO_READ			0x3C    // Burst FIFO read operation
#define SINGLE_FIFO_READ		0x3D    // Single FIFO read operation

#define FIFO_SIZE1				0x42    // Camera write FIFO size[7:0] for burst to read
#define FIFO_SIZE2				0x43    // Camera write FIFO size[15:8]
#define FIFO_SIZE3				0x44    // Camera write FIFO size[18:16]

#define ARDUCHIP_FRAMES			0x01    // FRAME control register, Bit[2:0] = Number of frames to be captured
                                        // On 5MP_Plus platforms bit[2:0] = 7 means continuous capture until frame buffer is full

#define ARDUCHIP_TIM       		0x03    // Timing control

#if !(defined OV2640_MINI_2MP)
#define VSYNC_LEVEL_MASK   		0x02    // 0 = High active , 		1 = Low active
#endif

ArduCAM_Mini::ArduCAM_Mini(uint8_t addr, uint32_t mfs, uint8_t cs, class ArduCAM_FrameGrabber * fg)
{
    P_CS  = portOutputRegister(digitalPinToPort(cs));
    B_CS  = digitalPinToBitMask(cs);

    pinMode(cs, OUTPUT);
    sbi(P_CS, B_CS);

    sensor_addr = addr;
    max_fifo_size = mfs;

    grabber = fg;
}

void ArduCAM_Mini::capture(void)
{

    // Wait for start bit from host
    if (grabber->gotStartRequest()) {
        capturing = true;
        starting = true;
    }

    if (capturing) {

        // Check for halt bit from host
        if (grabber->gotStopRequest()) {
            starting = false;
            capturing = false;
            return;
        }

        if (starting) {

            flush_fifo();
            clear_fifo_flag();
            start_capture();
            starting = false;
        }

        if (get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {

            uint32_t length = read_fifo_length();

            if (length >= max_fifo_size || length == 0) {
                clear_fifo_flag();
                starting = true;
                return;
            }

            csLow();
            set_fifo_burst();


            if (usingJpeg) {
                grabJpegFrame(length);
            }
            else {
                grabQvgaFrame(length);
            }

            csHigh();
            clear_fifo_flag();
        }
    }
}

void ArduCAM_Mini_5MP::transferQvgaByte(void)
{
}


void ArduCAM_Mini_5MP::beginQvga(uint8_t _scaledown, bool _grayscale)
{
    scaledown = 1 << _scaledown;
    grayscale = _grayscale;

    // XXX should be possible to use spiCheck() instead
    while (true ) {
        //Check if the ArduCAM_5MP_QVGA SPI bus is OK
        write_reg(ARDUCHIP_TEST1, 0x55);
        if (read_reg(ARDUCHIP_TEST1) != 0x55) {
            Serial.println(F("ACK CMD SPI interface Error!"));
            delay(1000);continue;
        }else{
            Serial.println(F("ACK CMD SPI interface OK."));break;
        }
    }

    wrSensorReg16_8(0x3008, 0x80);
    wrSensorRegs16_8(OV5642_QVGA_Preview);

    delay(100);

    byte reg_val;
    wrSensorReg16_8(0x4740, 0x21);
    wrSensorReg16_8(0x501e, 0x2a);
    wrSensorReg16_8(0x5002, 0xf8);
    wrSensorReg16_8(0x501f, 0x01);
    wrSensorReg16_8(0x4300, 0x61);
    rdSensorReg16_8(0x3818, &reg_val);
    wrSensorReg16_8(0x3818, (reg_val | 0x60) & 0xff);
    rdSensorReg16_8(0x3621, &reg_val);
    wrSensorReg16_8(0x3621, reg_val & 0xdf);

    set_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
    clear_fifo_flag();
    write_reg(ARDUCHIP_FRAMES, 0x00);
    clear_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
    wrSensorReg16_8(0x3818, 0x81);
    wrSensorReg16_8(0x3621, 0xA7);

    usingJpeg = false;
}

ArduCAM_Mini_5MP::ArduCAM_Mini_5MP(uint8_t cs, class ArduCAM_FrameGrabber * fg) : ArduCAM_Mini(0x78, 0x7FFFF, cs, fg) 
{
}


void ArduCAM_Mini_5MP::beginJpeg320x240(void)
{
    beginJpeg(ov5642_320x240);
}

void ArduCAM_Mini_5MP::beginJpeg640x480(void)
{
    beginJpeg(ov5642_640x480);
}

void ArduCAM_Mini_5MP::beginJpeg1024x768(void)
{
    beginJpeg(ov5642_1024x768);
}

void ArduCAM_Mini_5MP::beginJpeg1280x960(void)
{
    beginJpeg(ov5642_1280x960);
}

void ArduCAM_Mini_5MP::beginJpeg1600x1200(void)
{
    beginJpeg(ov5642_1600x1200);
}

void ArduCAM_Mini_5MP::beginJpeg2048x1536(void)
{
    beginJpeg(ov5642_2048x1536);
}

void ArduCAM_Mini_5MP::beginJpeg2592x1944(void)
{
    beginJpeg(ov5642_2592x1944);
}


void ArduCAM_Mini_5MP::beginJpeg(const struct sensor_reg reglist[])
{    
    usingJpeg = true;

    // Check SPI connection
    spiCheck();

    wrSensorReg16_8(0x3008, 0x80);
    wrSensorRegs16_8(OV5642_QVGA_Preview);

    delay(100);

    wrSensorRegs16_8(OV5642_JPEG_Capture_QSXGA);
    wrSensorRegs16_8(reglist);
    wrSensorReg16_8(0x3818, 0xa8);
    wrSensorReg16_8(0x3621, 0x10);
    wrSensorReg16_8(0x3801, 0xb0);
    wrSensorReg16_8(0x4407, 0x04);

    capturing = false;
    starting = false;

    set_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
    clear_fifo_flag();
    write_reg(ARDUCHIP_FRAMES, 0x00);
}

void ArduCAM_Mini_2MP::beginQvga(uint8_t _scaledown, bool _grayscale)
{
    scaledown = 1 << _scaledown;
    grayscale = _grayscale;

    begin();
    wrSensorRegs8_8(OV2640_QVGA);
}


void ArduCAM_Mini_2MP::transferQvgaByte(void)
{
    SPI.transfer(0x00);
}

ArduCAM_Mini_2MP::ArduCAM_Mini_2MP(int cs, class ArduCAM_FrameGrabber * fg) : ArduCAM_Mini(0x60, 0x5FFFF, cs, fg)
{
    usingJpeg = false;
}

void ArduCAM_Mini_2MP::beginJpeg160x120(void)
{
    beginJpeg(OV2640_160x120_JPEG);
}

void ArduCAM_Mini_2MP::beginJpeg176x144(void)
{
    beginJpeg(OV2640_176x144_JPEG);
}

void ArduCAM_Mini_2MP::beginJpeg320x240(void)
{
    beginJpeg(OV2640_320x240_JPEG);
}

void ArduCAM_Mini_2MP::beginJpeg352x288(void)
{
    beginJpeg(OV2640_352x288_JPEG);
}

void ArduCAM_Mini_2MP::beginJpeg640x480(void)
{
    beginJpeg(OV2640_640x480_JPEG);
}

void ArduCAM_Mini_2MP::beginJpeg800x600(void)
{
    beginJpeg(OV2640_800x600_JPEG);
}

void ArduCAM_Mini_2MP::beginJpeg1024x768(void)
{
    beginJpeg(OV2640_1024x768_JPEG);
}

void ArduCAM_Mini_2MP::beginJpeg1280x1024(void)
{
    beginJpeg(OV2640_1280x1024_JPEG);
}

void ArduCAM_Mini_2MP::beginJpeg1600x1200(void)
{
    beginJpeg(OV2640_1600x1200_JPEG);
}

/****************************************************/
/* Private methods                                  */
/****************************************************/

void ArduCAM_Mini::grabJpegFrame(uint32_t length)
{
    uint8_t temp = 0xff, temp_last = 0;
    bool is_header = false;

    while (--length) {
        temp_last = temp;
        temp =  SPI.transfer(0x00);
        if (is_header) {
            grabber->sendByte(temp);
        }
        else if ((temp == 0xD8) & (temp_last == 0xFF)) {
            is_header = true;
            grabber->sendByte(temp_last);
            grabber->sendByte(temp);
        }
        if ((temp == 0xD9) && (temp_last == 0xFF)) 
            break;
        delayMicroseconds(15);
    }

    // supports continuous capture
    starting = true;
}

void ArduCAM_Mini::grabQvgaFrame(uint32_t length)
{
    // 2MP requires this
    transferQvgaByte();

    // ignore length and use fixed-size frame
    (void)length;

    for (int i = 0; i < 240; i++) {
        for (int j = 0; j < 320; j++) {
            uint8_t hi = SPI.transfer(0x00);;
            uint8_t lo = SPI.transfer(0x00);;
            if (!(i%scaledown) && !(j%scaledown)) {
                if (grayscale) {
                    uint16_t rgb = (((uint16_t)hi)<<8) + (uint16_t)lo;
                    uint16_t r = (rgb & 0xF800) >> 11;
                    uint16_t g = (rgb & 0x07E0) >> 5;
                    uint16_t b = rgb & 0x001F;
                    uint8_t gray = (uint8_t)(0.21*r + 0.72*g + 0.07*b);
                    grabber->sendByte(gray*2);
                }
                else {
                    grabber->sendByte(lo);
                    grabber->sendByte(hi);
                }
                delayMicroseconds(12);
            }
        }
    }
}

void ArduCAM_Mini_2MP::beginJpeg(const struct sensor_reg reglist[])
{
    usingJpeg = true;

    begin();

    wrSensorRegs8_8(OV2640_JPEG_INIT);
    wrSensorRegs8_8(OV2640_YUV422);
    wrSensorRegs8_8(OV2640_JPEG);
    wrSensorReg8_8(0xff, 0x01);
    wrSensorReg8_8(0x15, 0x00);

    wrSensorRegs8_8(reglist);
}

void ArduCAM_Mini_2MP::begin(void)
{
    spiCheck();

    wrSensorReg8_8(0xff, 0x01);
    wrSensorReg8_8(0x12, 0x80);

    capturing = false;
    starting = false;

    delay(100);
}

// ------------------------------------------------------------------------------------------

void ArduCAM_Mini::spiCheck(void)
{
    // Check if the ArduCAM SPI bus is OK
    while (true) {
        write_reg(ARDUCHIP_TEST1, 0x55);
        uint8_t temp = read_reg(ARDUCHIP_TEST1);
        if (temp != 0x55) {
            Serial.println(F("ACK CMD SPI interface Error!"));
            delay(1000);
            continue;
        } else {
            Serial.println(F("ACK CMD SPI interface OK."));
            break;
        }
    }
}

void ArduCAM_Mini::flush_fifo(void)
{
    write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void ArduCAM_Mini::start_capture(void)
{
    write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

void ArduCAM_Mini::clear_fifo_flag(void )
{
    write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

uint32_t ArduCAM_Mini::read_fifo_length(void)
{
    uint32_t len1,len2,len3,length=0;
    len1 = read_reg(FIFO_SIZE1);
    len2 = read_reg(FIFO_SIZE2);
    len3 = read_reg(FIFO_SIZE3) & 0x7f;
    length = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
    return length;	
}

void ArduCAM_Mini::set_fifo_burst()
{
    SPI.transfer(BURST_FIFO_READ);
}

void ArduCAM_Mini::csHigh(void)
{
    sbi(P_CS, B_CS);	
}
void ArduCAM_Mini::csLow(void)
{
    cbi(P_CS, B_CS);	
}

uint8_t ArduCAM_Mini::read_fifo(void)
{
    uint8_t data;
    data = bus_read(SINGLE_FIFO_READ);
    return data;
}

uint8_t ArduCAM_Mini::read_reg(uint8_t addr)
{
    uint8_t data;
    data = bus_read(addr & 0x7F);
    return data;
}

void ArduCAM_Mini::write_reg(uint8_t addr, uint8_t data)
{
    bus_write(addr | 0x80, data);
}

//Set corresponding bit  
void ArduCAM_Mini::set_bit(uint8_t addr, uint8_t bit)
{
    uint8_t temp;
    temp = read_reg(addr);
    write_reg(addr, temp | bit);
}
//Clear corresponding bit 
void ArduCAM_Mini::clear_bit(uint8_t addr, uint8_t bit)
{
    uint8_t temp;
    temp = read_reg(addr);
    write_reg(addr, temp & (~bit));
}

//Get corresponding bit status
uint8_t ArduCAM_Mini::get_bit(uint8_t addr, uint8_t bit)
{
    uint8_t temp;
    temp = read_reg(addr);
    temp = temp & bit;
    return temp;
}

uint8_t ArduCAM_Mini::bus_write(int address,int value)
{	
    cbi(P_CS, B_CS);
    SPI.transfer(address);
    SPI.transfer(value);
    sbi(P_CS, B_CS);
    return 1;
}

uint8_t ArduCAM_Mini:: bus_read(int address)
{
    uint8_t value;
    cbi(P_CS, B_CS);
    SPI.transfer(address);
    value = SPI.transfer(0x00);
    // take the SS pin high to de-select the chip:
    sbi(P_CS, B_CS);
    return value;
}

// Write 8 bit values to 8 bit register address
int ArduCAM_Mini::wrSensorRegs8_8(const struct sensor_reg reglist[])
{
    //int err = 0;
    uint16_t reg_addr = 0;
    uint16_t reg_val = 0;
    const struct sensor_reg *next = reglist;
    while ((reg_addr != 0xff) | (reg_val != 0xff)) {
        reg_addr = pgm_read_word(&next->reg);
        reg_val = pgm_read_word(&next->val);
        wrSensorReg8_8(reg_addr, reg_val);
        next++;
    }
    return 1;
}


// Write 8 bit values to 16 bit register address
// Read/write 8 bit value to/from 8 bit register address	
byte ArduCAM_Mini::wrSensorReg8_8(int regID, int regDat)
{
    Wire.beginTransmission(sensor_addr >> 1);
    Wire.write(regID & 0x00FF);
    Wire.write(regDat & 0x00FF);
    if (Wire.endTransmission()) {
        return 0;
    }
    delay(1);
    return 1;

}
byte ArduCAM_Mini::rdSensorReg8_8(uint8_t regID, uint8_t* regDat)
{	
    Wire.beginTransmission(sensor_addr >> 1);
    Wire.write(regID & 0x00FF);
    Wire.endTransmission();

    Wire.requestFrom((sensor_addr >> 1), 1);
    if (Wire.available())
        *regDat = Wire.read();
    delay(1);
    return 1;

}

// Write 8 bit values to 16 bit register address
int ArduCAM_Mini::wrSensorRegs16_8(const struct sensor_reg reglist[])
{
    unsigned int reg_addr = 0;
    unsigned char reg_val = 0;
    const struct sensor_reg *next = reglist;

    while ((reg_addr != 0xffff) | (reg_val != 0xff))
    {

        reg_addr = pgm_read_word(&next->reg);
        reg_val = pgm_read_word(&next->val);
        wrSensorReg16_8(reg_addr, reg_val);
        next++;
    }
    return 1;
}

// Read/write 8 bit value to/from 16 bit register address
byte ArduCAM_Mini::wrSensorReg16_8(int regID, int regDat)
{
    Wire.beginTransmission(sensor_addr >> 1);
    Wire.write(regID >> 8);            // sends instruction byte, MSB first
    Wire.write(regID & 0x00FF);
    Wire.write(regDat & 0x00FF);
    if (Wire.endTransmission())
    {
        return 0;
    }
    delay(1);
    return 1;
}

byte ArduCAM_Mini::rdSensorReg16_8(uint16_t regID, uint8_t* regDat)
{
    Wire.beginTransmission(sensor_addr >> 1);
    Wire.write(regID >> 8);
    Wire.write(regID & 0x00FF);
    Wire.endTransmission();
    Wire.requestFrom((sensor_addr >> 1), 1);
    if (Wire.available())
    {
        *regDat = Wire.read();
    }
    delay(1);
    return 1;
}


