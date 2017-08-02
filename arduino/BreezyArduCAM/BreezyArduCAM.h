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
#define regtype volatile uint8_t
#define regsize uint8_t
#endif

#if defined(__SAM3X8E__)
#define regtype volatile uint32_t
#define regsize uint32_t
#endif


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

        void initQvga();

        void initJpeg160x120(void);
        void initJpeg176x144(void);
        void initJpeg320x240(void);
        void initJpeg352x288(void);
        void initJpeg640x480(void);
        void initJpeg800x600(void);
        void initJpeg1024x768(void);
        void initJpeg1280x1024(void);
        void initJpeg1600x1200(void);

        void captureJpeg(void);
        void captureRaw(void);

    protected:

        virtual bool gotStartRequest(void) = 0;
        virtual bool gotStopRequest(void) = 0;
        virtual void sendByte(uint8_t b) = 0;

    private:

        void csHigh(void);
        void csLow(void);

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

        void init();

        void initJpeg(uint8_t size);

        regtype *P_CS;
        regsize B_CS;
        byte sensor_addr;

        bool capturing;
        bool starting;
        bool got_header;
        uint8_t tmp;
        uint8_t tmp_last;
};

class Serial_ArduCAM_Mini_2MP : public ArduCAM_Mini_2MP {

    public:

        Serial_ArduCAM_Mini_2MP(uint8_t cs) : ArduCAM_Mini_2MP(cs) { }

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


#include "ov2640_regs.h"

#endif
