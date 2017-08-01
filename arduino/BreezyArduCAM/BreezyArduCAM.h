/*
BreezyArduCAM.h : class declarations for BreezyArduCAM libary

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

#ifndef BreezyArduCAM_H
#define BreezyArduCAM_H

#include <Arduino.h>

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

#if defined(__CPU_ARC__)
#define cbi(reg, bitmask) *reg &= ~bitmask
#define sbi(reg, bitmask) *reg |= bitmask
#define pulse_high(reg, bitmask) sbi(reg, bitmask); cbi(reg, bitmask);
#define pulse_low(reg, bitmask) cbi(reg, bitmask); sbi(reg, bitmask);
#define cport(port, data) port &= data
#define sport(port, data) port |= data
#define swap(type, i, j) {type t = i; i = j; j = t;}
#define fontbyte(x) pgm_read_byte(&cfont.font[x])  
#define regtype volatile uint32_t
#define regsize uint32_t
#endif


/****************************************************/
/* Sensor related definition 				        */
/****************************************************/

#define OV2640_160x120 		0	//160x120
#define OV2640_176x144 		1	//176x144
#define OV2640_320x240 		2	//320x240
#define OV2640_352x288 		3	//352x288
#define OV2640_640x480		4	//640x480
#define OV2640_800x600 		5	//800x600
#define OV2640_1024x768		6	//1024x768
#define OV2640_1280x1024	7	//1280x1024
#define OV2640_1600x1200	8	//1600x1200

/****************************************************/
/* I2C Control Definition 						    */
/****************************************************/

//Define maximum frame buffer size
#define MAX_FIFO_SIZE		0x5FFFF			//384KByte

/****************************************************/
/* ArduChip registers definition 											*/
/****************************************************/
#define RWBIT				    0x80  //READ AND WRITE BIT IS BIT[7]

#define ARDUCHIP_TEST1       	0x00  //TEST register

#if !(defined OV2640_MINI_2MP)
#define ARDUCHIP_FRAMES		    0x01  //FRAME control register, Bit[2:0] = Number of frames to be captured																		                  //On 5MP_Plus platforms bit[2:0] = 7 means continuous capture until frame buffer is full
#endif

#define ARDUCHIP_MODE      		0x02  //Mode register
#define MCU2LCD_MODE       		0x00
#define CAM2LCD_MODE       		0x01
#define LCD2MCU_MODE       		0x02

#define ARDUCHIP_TIM       		0x03  //Timming control
#if !(defined OV2640_MINI_2MP)
#define HREF_LEVEL_MASK    		0x01  //0 = High active , 		1 = Low active
#define VSYNC_LEVEL_MASK   		0x02  //0 = High active , 		1 = Low active
#define LCD_BKEN_MASK      		0x04  //0 = Enable, 					1 = Disable
#if (defined ARDUCAM_SHIELD_V2)
#define PCLK_REVERSE_MASK  	    0x08  //0 = Normal PCLK, 		1 = REVERSED PCLK
#else
#define PCLK_DELAY_MASK  		0x08  //0 = data no delay,		1 = data delayed one PCLK
#endif
#endif

#define ARDUCHIP_FIFO      		0x04  //FIFO and I2C control
#define FIFO_CLEAR_MASK    		0x01
#define FIFO_START_MASK    		0x02
#define FIFO_RDPTR_RST_MASK     0x10
#define FIFO_WRPTR_RST_MASK     0x20

#define ARDUCHIP_GPIO			  0x06  //GPIO Write Register
#if !(defined (ARDUCAM_SHIELD_V2) || defined (ARDUCAM_SHIELD_REVC))
#define GPIO_RESET_MASK			0x01  //0 = Sensor reset,							1 =  Sensor normal operation
#define GPIO_PWDN_MASK			0x02  //0 = Sensor normal operation, 	1 = Sensor standby
#define GPIO_PWREN_MASK			0x04	//0 = Sensor LDO disable, 			1 = sensor LDO enable
#endif

#define BURST_FIFO_READ			0x3C  //Burst FIFO read operation
#define SINGLE_FIFO_READ		0x3D  //Single FIFO read operation

#define ARDUCHIP_REV       		0x40  //ArduCHIP revision
#define VER_LOW_MASK       		0x3F
#define VER_HIGH_MASK      		0xC0

#define ARDUCHIP_TRIG      		0x41  //Trigger source
#define VSYNC_MASK         		0x01
#define SHUTTER_MASK       		0x02
#define CAP_DONE_MASK      		0x08

#define FIFO_SIZE1				0x42  //Camera write FIFO size[7:0] for burst to read
#define FIFO_SIZE2				0x43  //Camera write FIFO size[15:8]
#define FIFO_SIZE3				0x44  //Camera write FIFO size[18:16]


/****************************************************/


/****************************************************************/
/* define a structure for sensor register initialization values */
/****************************************************************/
struct sensor_reg {
    uint16_t reg;
    uint16_t val;
};



/****************************************************************/
/* define a structure for sensor register initialization values */
/****************************************************************/

class ArduCAM_Mini_2MP
{
    public:

        ArduCAM_Mini_2MP(int CS);

        void initBmp();

        void initJpeg320x240();

        void CS_HIGH(void);
        void CS_LOW(void);

        void flush_fifo(void);
        void start_capture(void);
        void clear_fifo_flag(void);
        uint8_t read_fifo(void);

        uint8_t read_reg(uint8_t addr);
        void write_reg(uint8_t addr, uint8_t data);	

        uint32_t read_fifo_length(void);
        void set_fifo_burst(void);

        void set_bit(uint8_t addr, uint8_t bit);
        void clear_bit(uint8_t addr, uint8_t bit);
        uint8_t get_bit(uint8_t addr, uint8_t bit);

        uint8_t bus_write(int address, int value);
        uint8_t bus_read(int address);	

        // Write 8 bit values to 8 bit register address
        int wrSensorRegs8_8(const struct sensor_reg*);

        // Write 16 bit values to 8 bit register address
        int wrSensorRegs8_16(const struct sensor_reg*);

        // Write 8 bit values to 16 bit register address
        int wrSensorRegs16_8(const struct sensor_reg*);

        // Write 16 bit values to 16 bit register address
        int wrSensorRegs16_16(const struct sensor_reg*);

        // Read/write 8 bit value to/from 8 bit register address	
        byte wrSensorReg8_8(int regID, int regDat);
        byte rdSensorReg8_8(uint8_t regID, uint8_t* regDat);

        // Read/write 16 bit value to/from 8 bit register address
        byte wrSensorReg8_16(int regID, int regDat);
        byte rdSensorReg8_16(uint8_t regID, uint16_t* regDat);

        // Read/write 8 bit value to/from 16 bit register address
        byte wrSensorReg16_8(int regID, int regDat);
        byte rdSensorReg16_8(uint16_t regID, uint8_t* regDat);

        // Read/write 16 bit value to/from 16 bit register address
        byte wrSensorReg16_16(int regID, int regDat);
        byte rdSensorReg16_16(uint16_t regID, uint16_t* regDat);

        void OV2640_set_JPEG_size(uint8_t size);
        void set_format(byte fmt);
        void transferBytes_(uint8_t * out, uint8_t * in, uint8_t size);
        void transferBytes(uint8_t * out, uint8_t * in, uint32_t size);
        inline void setDataBits(uint16_t bits);

    private:

        void init();

        void initJpeg(uint8_t size);

        regtype *P_CS;
        regsize B_CS;
        byte m_fmt;
        byte sensor_addr;
};

#include "ov2640_regs.h"

#endif
