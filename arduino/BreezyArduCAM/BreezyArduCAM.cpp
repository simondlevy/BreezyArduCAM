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
#endif

#if defined(__SAM3X8E__)

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

#define ARDUCHIP_TEST1       	0x00    //TEST register

#define ARDUCHIP_FIFO      		0x04    //FIFO and I2C control
#define FIFO_CLEAR_MASK    		0x01
#define FIFO_START_MASK    		0x02

#define ARDUCHIP_TRIG      		0x41    //Trigger source
#define CAP_DONE_MASK      		0x08

#define BURST_FIFO_READ			0x3C    //Burst FIFO read operation
#define SINGLE_FIFO_READ		0x3D    //Single FIFO read operation

#define FIFO_SIZE1				0x42    //Camera write FIFO size[7:0] for burst to read
#define FIFO_SIZE2				0x43    //Camera write FIFO size[15:8]
#define FIFO_SIZE3				0x44    //Camera write FIFO size[18:16]

//Define maximum frame buffer size
#define MAX_FIFO_SIZE		    0x5FFFF //384KByte


/****************************************************/
/* Public methods                                   */
/****************************************************/

ArduCAM_Mini_2MP::ArduCAM_Mini_2MP(int cs)
{
    P_CS  = portOutputRegister(digitalPinToPort(cs));
    B_CS  = digitalPinToBitMask(cs);

    pinMode(cs, OUTPUT);
    sbi(P_CS, B_CS);
    sensor_addr = 0x60;
}

void ArduCAM_Mini_2MP::beginQvga(void)
{
    begin();

    wrSensorRegs8_8(OV2640_QVGA);
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

void ArduCAM_Mini_2MP::captureJpeg(void)
{
    // Wait for start bit from host
    if (gotStartRequest()) {
        capturing = true;
        starting = true;
    }

    if (capturing) {

        // Check for halt bit from host
        if (gotStopRequest()) {
            starting = false;
            capturing = false;
            return;
        }

        capture(true);
    }
}

void ArduCAM_Mini_2MP::captureQvga(void)
{
    if (gotStartRequest()) {
        starting = true;
    }

    capture(false);
}

/****************************************************/
/* Private methods                                  */
/****************************************************/

void ArduCAM_Mini_2MP::capture(bool useJpeg)
{

    if (starting) {
        flush_fifo();
        clear_fifo_flag();
        start_capture();
        starting = false;
    }

    if (get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {

        uint32_t length = read_fifo_length();

        if (length >= MAX_FIFO_SIZE || length == 0) {
            clear_fifo_flag();
            starting = true;
            return;
        }

        csLow();
        set_fifo_burst();
        SPI.transfer(0x00);

        if (useJpeg)
            grabJpegFrame(length);
        else
            grabQvgaFrame(length);

        csHigh();
        clear_fifo_flag();
    }
}

void ArduCAM_Mini_2MP::grabJpegFrame(uint32_t length)
{
    uint8_t temp = 0xff, temp_last = 0;
    bool is_header = false;

    while (--length) {
        temp_last = temp;
        temp =  SPI.transfer(0x00);
        if (is_header) {
            sendByte(temp);
        }
        else if ((temp == 0xD8) & (temp_last == 0xFF)) {
            is_header = true;
            sendByte(temp_last);
            sendByte(temp);
        }
        if ((temp == 0xD9) && (temp_last == 0xFF)) 
            break;
        delayMicroseconds(15);
    }

    // supports continuous capture
    starting = true;
}

void ArduCAM_Mini_2MP::grabQvgaFrame(uint32_t length)
{
    // ignore length and use fixed-size frame
    (void)length;

    for (int i = 0; i < 240; i++) {
        for (int j = 0; j < 320; j++) {
            char VH = SPI.transfer(0x00);;
            char VL = SPI.transfer(0x00);;
            sendByte(VL);
            delayMicroseconds(12);
            sendByte(VH);
            delayMicroseconds(12);
        }
    }
}

void ArduCAM_Mini_2MP::beginJpeg(const struct sensor_reg reglist[])
{
    begin();

    wrSensorRegs8_8(OV2640_JPEG_INIT);
    wrSensorRegs8_8(OV2640_YUV422);
    wrSensorRegs8_8(OV2640_JPEG);
    wrSensorReg8_8(0xff, 0x01);
    wrSensorReg8_8(0x15, 0x00);

    wrSensorRegs8_8(reglist);
}
void ArduCAM_Mini_2MP::begin()
{
    // Check if the ArduCAM SPI bus is OK
    while (true) {
        write_reg(ARDUCHIP_TEST1, 0x55);
        uint8_t temp = read_reg(ARDUCHIP_TEST1);
        if (temp != 0x55) {
            Serial.println("ACK CMD SPI interface Error!");
            delay(1000);
            continue;
        } else{
            Serial.println("ACK CMD SPI interface OK.");
            break;
        }
    }

    wrSensorReg8_8(0xff, 0x01);
    wrSensorReg8_8(0x12, 0x80);

    capturing = false;
    starting = false;

    delay(100);
}

void ArduCAM_Mini_2MP::flush_fifo(void)
{
    write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void ArduCAM_Mini_2MP::start_capture(void)
{
    write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

void ArduCAM_Mini_2MP::clear_fifo_flag(void )
{
    write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

uint32_t ArduCAM_Mini_2MP::read_fifo_length(void)
{
    uint32_t len1,len2,len3,length=0;
    len1 = read_reg(FIFO_SIZE1);
    len2 = read_reg(FIFO_SIZE2);
    len3 = read_reg(FIFO_SIZE3) & 0x7f;
    length = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
    return length;	
}

void ArduCAM_Mini_2MP::set_fifo_burst()
{
    SPI.transfer(BURST_FIFO_READ);
}

void ArduCAM_Mini_2MP::csHigh(void)
{
    sbi(P_CS, B_CS);	
}
void ArduCAM_Mini_2MP::csLow(void)
{
    cbi(P_CS, B_CS);	
}

uint8_t ArduCAM_Mini_2MP::read_fifo(void)
{
    uint8_t data;
    data = bus_read(SINGLE_FIFO_READ);
    return data;
}

uint8_t ArduCAM_Mini_2MP::read_reg(uint8_t addr)
{
    uint8_t data;
    data = bus_read(addr & 0x7F);
    return data;
}

void ArduCAM_Mini_2MP::write_reg(uint8_t addr, uint8_t data)
{
    bus_write(addr | 0x80, data);
}

//Set corresponding bit  
void ArduCAM_Mini_2MP::set_bit(uint8_t addr, uint8_t bit)
{
    uint8_t temp;
    temp = read_reg(addr);
    write_reg(addr, temp | bit);
}
//Clear corresponding bit 
void ArduCAM_Mini_2MP::clear_bit(uint8_t addr, uint8_t bit)
{
    uint8_t temp;
    temp = read_reg(addr);
    write_reg(addr, temp & (~bit));
}

//Get corresponding bit status
uint8_t ArduCAM_Mini_2MP::get_bit(uint8_t addr, uint8_t bit)
{
    uint8_t temp;
    temp = read_reg(addr);
    temp = temp & bit;
    return temp;
}

uint8_t ArduCAM_Mini_2MP::bus_write(int address,int value)
{	
    cbi(P_CS, B_CS);
    SPI.transfer(address);
    SPI.transfer(value);
    sbi(P_CS, B_CS);
    return 1;
}

uint8_t ArduCAM_Mini_2MP:: bus_read(int address)
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
int ArduCAM_Mini_2MP::wrSensorRegs8_8(const struct sensor_reg reglist[])
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
byte ArduCAM_Mini_2MP::wrSensorReg8_8(int regID, int regDat)
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
byte ArduCAM_Mini_2MP::rdSensorReg8_8(uint8_t regID, uint8_t* regDat)
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
