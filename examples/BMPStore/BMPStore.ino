/*
   BMPStore.ino : grab a single frame and store it on the flash RAM

   Image is grabbed and saved as soon as the sketch finishes flashing.

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
#include <FS.h>

// set pin 10 as the slave select for the digital pot:
static const int CS = 10;

// arbitrary name for image file
static const char * FILENAME = "/test.bmp";

/*
   BMP header for 320x240 image ------------------------------------------------------

See: http://www.fastgraph.com/help/bmp_header_format.html
See: https://upload.wikimedia.org/wikipedia/commons/c/c4/BMPfileFormat.png
 */
static const uint8_t HDRSIZE = 66;
static const uint8_t bmp_header[HDRSIZE] = {
    0x42, 0x4D,             // signature, must be 4D42 hex
    0x36, 0x58, 0x02, 0x00, // size of BMP file in bytes (unreliable)
    0x00, 0x00,             // reserved, must be zero
    0x00, 0x00,             // reserved, must be zero
    0x42, 0x00, 0x00, 0x00, // offset to start of image data in bytes
    0x28, 0x00, 0x00, 0x00, // size of BITMAPINFOHEADER structure, must be 40
    0x40, 0x01, 0x00, 0x00, // image width in pixels
    0xF0, 0x00, 0x00, 0x00, // image height in pixels
    0x01, 0x00,             // number of planes in the image, must be 1
    0x10, 0x00,             // number of bits per pixel
    0x03, 0x00, 0x00, 0x00, // compression type
    0x00, 0x58, 0x02, 0x00, // size of image data in bytes (including padding)
    0xC4, 0x0E, 0x00, 0x00, // horizontal resolution in pixels per meter (unreliable)
    0xC4, 0x0E, 0x00, 0x00, // vertical resolution in pixels per meter (unreliable)
    0x00, 0x00, 0x00, 0x00, // number of colors in image, or zero
    0x00, 0x00, 0x00, 0x00, // number of important colors, or zero
    0x00, 0xF8, 0x00, 0x00, // red channel bitmask
    0xE0, 0x07, 0x00, 0x00, // green channel bitmask
    0x1F, 0x00, 0x00, 0x00  // blue channel bitmask
};

File file;

class FlashRAM_FrameGrabber : public ArduCAM_FrameGrabber {

    private:

        bool done;

    public:

        FlashRAM_FrameGrabber(void)
        {
            done = false;
        }

    protected:

        /**
         * Implements the gotStartRequest() method by checking whether we've written the file yet
         */
        virtual bool gotStartRequest(void) override 
        {
            return !done;
        }

        /**
         * Implements the gotStopRequest() method by checking whether we've written the file yet
         */
        virtual bool gotStopRequest(void) override 
        {

            if (done) {
                file.close();
                DOSFS.end();
            }

            return done;
        }

        /**
         * Implements the sencByte() method by writing the byte to a file.
         * @param b the byte to send
         */
        virtual void sendByte(uint8_t b) override 
        {
            // Write the byte to the file
            file.write(b);

            // As soon as we've handled the first byte, we know we'll capture the whole frame
            done = true; 
        }
};

FlashRAM_FrameGrabber fg;

// Choose your camera 
//ArduCAM_Mini_2MP myCam(CS, &fg);
ArduCAM_Mini_5MP myCam(CS, &fg);

void setup(void) 
{
    // Enable file interaction
    DOSFS.begin();

    // Delete the old file if it exists
    if (DOSFS.exists(FILENAME)) { 
        DOSFS.remove(FILENAME);
    }

    // Open the file for writing, creating it
    file = DOSFS.open(FILENAME, "w");

    // Write the BMP header to the file
    for (int k=0; k<HDRSIZE; ++k) {
        file.write(bmp_header[k]);
    }

    // ArduCAM Mini uses both I^2C and SPI buses
    Wire.begin();
    SPI.begin();

    // Begin capturing in  QVGA mode
    myCam.beginQvga();

}

void loop(void) 
{
    myCam.capture();
}
