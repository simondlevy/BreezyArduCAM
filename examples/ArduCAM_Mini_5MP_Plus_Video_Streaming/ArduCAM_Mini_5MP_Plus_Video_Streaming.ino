#include <Wire.h>
#include <SPI.h>

#include <BreezyArduCAM.h>

const int CS = 10;

Serial_ArduCAM_FrameGrabber fg;
ArduCAM_Mini_5MP myCam(CS, &fg);

void setup(void) 
{

    // ArduCAM Mini uses both I^2C and SPI buses
    Wire.begin();
    SPI.begin();

    // Talk to Arduino at fastest possible baud rate
    Serial.begin(921600);

    // Start the camera in JPEG mode with a specific image size
    myCam.beginJpeg();
}

void loop(void) 
{
    myCam.capture();
}

