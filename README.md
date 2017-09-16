# A simple Arduino API for ArduCAM Mini

<img src="image.png" width=200>

This repository contains an Arduino library and examples for the popular ArduCAM Mini camera. 
I've taken the code in the original ArduCAM Arduino [repository](https://github.com/ArduCAM/Arduino) 
and simplified the API, so you can write new applications with a minimal amount of effort.  

As you can see in this 
<a href="https://github.com/simondlevy/BreezyArduCAM/blob/master/examples/JPEGStream/JPEGStream.ino#L29-L49">
example</a>,
you just call a constructor method, a begin method, and a capture method.
Each example
(streaming JPEG images, BMP snapshot) has a corresponding Python script that you can run on your host 
computer.

I have tested this library with ArduCAM Mini 2MP (OV2640) and ArducamMini 5MP Plus (OV5642).

To run the example programs you will need:

<ul>
<li> <a href="http://www.arducam.com/tag/arducam-mini">ArduCAM Mini</a>
<li> Arduino board
<li> <a href="https://pypi.python.org/pypi/pyserial">pyserial</a>
<li> OpenCV (for the streaming JPEG example; see note below)
</ul>

If (like me) you use an ordinary Arduino board without the ArduCAM Shield you can use the ArduCAM 
[tutorial](http://www.arducam.com/knowledge-base/category/tutorial/arduino/) to set up your camera.
Pay special attention to the <b>CS</b> pin, which you'll have to 
[change](http://www.arducam.com/wp-content/uploads/2017/06/cs.jpg) in the Arduino
sketch to match the wiring in the tutorial.

As usual with Arduino, you should clone this repository into your Arduino libraries folder.
Then launch the Arduino IDE and go to <b>File/Examples/BreezyArduCAM</b>) to upload an example sketch.

Once you've wired up the camera and loaded the sketch onto your Arduino, edit the <b>PORT</b> variable
at the top of the Python script to reflect the serial port on which your Arduino is connected.

<h2>Note on the streaming JPEG example</h2>

This example requires you to be running OpenCV for Python on your computer.  The Python program saves the
successive JPEG images to a single temporary file for display.  If you're a
Python (or C++) programmer interested in computer vision, you're going to want
to get familiar with OpenCV, and if you're a Python programmer, you're
eventually going to have to switch from Python2 to Python3 if you haven't
already.  Since I use Windows and Ubuntu, I can't help you with doing this on
Mac OS X.  On Ubuntu 16.04 I was able to get OpenCV installed for Python3 in
about ten minutes, using this
[tutorial](http://cyaninfinite.com/tutorials/installing-opencv-in-ubuntu-for-python-3/).
On Windows 10 I followed these
[directions](https://www.solarianprogrammer.com/2016/09/17/install-opencv-3-with-python-3-on-windows/)
(which may also help you if you need to install pyserial).

<h2>Note on the BMP snapshot example</h2>

This example saves a single 320x240 image to a file <b>test.bmp</b>.  For the reason explained 
[here](https://stackoverflow.com/questions/8346115/why-are-bmps-stored-upside-down), 
the BMP file may appear upside-down when displayed. This happens with  the 2MP camera but not the 5MP, for reasons I don't currently know.

<h2>Known bugs</h2>

Sometimes the <b>bmpsnap.py</b> script will report a Unicode decoding error when you run it for the first time.
