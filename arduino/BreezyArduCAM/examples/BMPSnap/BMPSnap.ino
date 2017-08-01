  
#include <Wire.h>
#include <BreezyArduCAM.h>
#include <SPI.h>

#define BMPIMAGEOFFSET 66
const unsigned char bmp_header[BMPIMAGEOFFSET] PROGMEM =
{
    0x42, 0x4D, 0x36, 0x58, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x00, 0x28, 0x00,
    0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x58, 0x02, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x1F, 0x00,
    0x00, 0x00
};

// set pin 10 as the slave select for the digital pot:
static const int CS = 10;

ArduCAM myCam(CS);

void setup(void) 
{
    uint8_t vid, pid;
    uint8_t temp;

    Wire.begin();
    Serial.begin(921600);

    Serial.println(F("ACK CMD ArduCAM Start!"));

    // set the CS as an output:
    pinMode(CS, OUTPUT);

    // initialize SPI:
    SPI.begin();

    while(true) {
        //Check if the ArduCAM SPI bus is OK
        myCam.write_reg(ARDUCHIP_TEST1, 0x55);
        temp = myCam.read_reg(ARDUCHIP_TEST1);
        if (temp != 0x55){
            Serial.println(F("ACK CMD SPI interface Error!"));
            delay(1000);continue;
        }else{
            Serial.println(F("ACK CMD SPI interface OK."));break;
        }
    }

    while(true) {
        //Check if the camera module type is OV2640
        myCam.wrSensorReg8_8(0xff, 0x01);
        myCam.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
        myCam.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
        if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 ))){
            Serial.println(F("ACK CMD Can't find OV2640 module!"));
            delay(1000);continue;
        }
        else{
            Serial.println(F("ACK CMD OV2640 detected."));break;
        } 
    }

    //Change to BMP capture mode and initialize the OV5642 module
    myCam.initBmp();
    Serial.println(F("ACK CMD CAM set format BMP"));
}

void loop(void) 
{
    static bool ready;
    static bool capturing;

    if (Serial.available()) {

        switch (Serial.read()) {
            case 0x30:
                ready = true;
                capturing = true;
                Serial.println(F("ACK CMD CAM start single shoot"));
                break;
            case 0x31:
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
            Serial.println(F("ACK CMD CAM Capture Done."));
            uint8_t temp;
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
            myCam.CS_LOW();
            myCam.set_fifo_burst();//Set fifo burst mode

            for (temp = 0; temp < BMPIMAGEOFFSET; temp++)
            {
                Serial.write(pgm_read_byte(&bmp_header[temp]));
            }
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
