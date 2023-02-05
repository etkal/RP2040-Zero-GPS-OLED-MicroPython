"""
GPS acquisition and display using MicroPython.

------------------------------------------------------------------

MIT License

Copyright (c) 2023 Erik Tkal

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

------------------------------------------------------------------

Customize your GPIO and UART/I2C IDs in the code below as needed.

------------------------------------------------------------------

"""

from machine import Pin, UART, I2C
from ssd1306 import SSD1306_I2C

import utime, time
import _thread
import math
import sys

import neopixel
neo = neopixel.NeoPixel(machine.Pin(16), 1)
def neo24(r, g, b):
    return (r, g, b)
neoOff    = neo24(0,0,0)
neoWhite  = neo24(100,100,100)
neoGray   = neo24(50,50,50)
neoBlue   = neo24(0,0,100)
neoRed    = neo24(100,0,0)
neoGreen  = neo24(0,100,0)
neoPurple = neo24(50,0,50)
def blinkLED(color=neoWhite, durationMs=10):
    neo[0] = color
    neo.write()
    time.sleep_ms(durationMs)
    neo[0] = neoOff
    neo.write()

# Input GPS module using GP0/1 UART0
uartGPS  = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1), timeout=5000)
# Output UART using GP4/5 UART1, also input from computer
uartSerial = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5), timeout=50)

# SD1306 OLED 128x64 using GP20/21 I2C0
i2c=I2C(1,sda=Pin(2), scl=Pin(3), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)
oled.rotate(2)  # Rotate 180 degrees

pinExit = Pin(8, Pin.IN, Pin.PULL_UP)

ECHO_GPS = True  # Set to echo received data to the second UART
PRINT_INPUT = False  # Set to print received data as is
DEBUG_APP = False

TIMEOUT = False
FIX_TIME = False
FIX_STATUS = False
GPGSV_IN_PROGRESS = False

latitude = ""
longitude = ""
altitude = ""
satellites = ""
GPStime = ""
mode3D = "No Fix"
speedKts = ""
numGSV = "0"

satList = []     # Complete list sent by GPS
tempSatList = [] # Temporary list used during $GPGSV processing
usedList = []    # Satellites used for position calculation
satListTime = 0.0
antenna = "Internal"

SAT_CENTER_X = 32
SAT_CENTER_Y = 32
SAT_RADIUS = 28

# For text placement
def line(pos):
    return (pos - 1) * 9

# Conver NMEA (d)ddmm.mmmm to (d)dd.dddd format for display
def convertToDecimalDegree(DegreeMinutes):
    FloatValue = float(DegreeMinutes)
    firstDigits = int(FloatValue/100)
    nextTwo = FloatValue - float(firstDigits*100)
    
    Result = float(firstDigits + nextTwo/60.0)
    Result = '{0:.4f}'.format(Result)
    return str(Result)

# Check formatting and checksum of received sentence
def verifySentence(buff):
    slen = len(buff)
    if (slen < 1 or buff[0] != '$'):
        print("Sentence does not start with $")
        return False
    if (slen < 6 or buff[slen-2:slen] != '\r\n' or buff[slen-5] != '*'):
        print("Sentence does not end with *XX\\r\\n")
        return False
    specifiedCheck = buff[slen-4:slen-2]
    expectedCheck = checkSum(buff[1:slen-5])
    if (expectedCheck != specifiedCheck):
        print("Error verifying: " + buff)
        print("         Passed: " + buff[1:slen-5])
        print("Sentence calculated checksum " + expectedCheck + " does not match " + specifiedCheck)
        return False
    return True

# Calculate XOR of all bytes between the $ and * in a sentence
def checkSum(sentence):
    check = 0
    for element in sentence:
        check = check ^ ord(element)
    retval = "{:02X}".format(check)
    return retval

# Takes an NMEA sentence (without $ or checksum) and properly formats for output
def writeSentence(sentence, gpsModule):
    # Write given un-checksummed sentence
    hexChecksum = checkSum(sentence)
    outstr = "$" + sentence + "*" + hexChecksum + "\r\n"
    #print(outstr)
    outbuff = outstr.encode('ascii')
    gpsModule.write(outbuff)

# Draws a small open circe for graphical satellite representation
def openCircle(el, az, radius):
    # Draw open circle (clear, then draw)
    dx = (SAT_RADIUS) * math.cos(math.radians(el)) * math.sin(math.radians(az))
    dy = (SAT_RADIUS) * math.cos(math.radians(el)) * -math.cos(math.radians(az))
    x = SAT_CENTER_X + int(dx)
    y = SAT_CENTER_Y + int(dy)
    oled.ellipse(x,y,radius,radius,0,1)  # Clear area with fill
    oled.ellipse(x,y,radius,radius,1,0)  # Draw circle without fill

# Draws a small closed circe for graphical satellite representation
def closedCircle(el, az, radius):
    # Draw closed circle
    dx = (SAT_RADIUS) * math.cos(math.radians(el)) * math.sin(math.radians(az))
    dy = (SAT_RADIUS) * math.cos(math.radians(el)) * -math.cos(math.radians(az))
    x = SAT_CENTER_X + int(dx)
    y = SAT_CENTER_Y + int(dy)
    oled.ellipse(x,y,radius,radius,1,1)  # Draw circle with fill

# Update the UI
def updateUI():
    global FIX_STATUS, FIX_TIME, TIMEOUT, latitude, longitude, altitude, satellites, \
        GPStime, mode3D, speedKts, numGSV, satListTime, oled, satList, antenna

    if (DEBUG_APP):
        print("updateUI called.  " + GPStime + " " + latitude + " " + longitude)
        print(str(usedList) + " " + str(satList))
    oled.fill(0)
    
    # Draw a grid for satellite display
    oled.ellipse(SAT_CENTER_X,SAT_CENTER_Y,SAT_RADIUS,SAT_RADIUS,1,0)
    oled.ellipse(SAT_CENTER_X,SAT_CENTER_Y,int(SAT_RADIUS/2),int(SAT_RADIUS/2),1,0)
    oled.vline(SAT_CENTER_X, SAT_CENTER_Y-SAT_RADIUS-2, 2*SAT_RADIUS+5, 1)
    oled.hline(SAT_CENTER_X-SAT_RADIUS-2, SAT_CENTER_Y, 2*SAT_RADIUS+5, 1)
    oled.text("'",SAT_CENTER_X-6,0,1)
    oled.text("`",SAT_CENTER_X-2,0,1)

    # Draw the text elements as appropriate
    oled.text(satellites,128-8*len(satellites),line(4),1)
    oled.text(mode3D,128-8*len(mode3D),line(5),1)
    radius = 1
    if (FIX_STATUS):
        oled.text(latitude,128-8*len(latitude),line(1),1)
        oled.text(longitude,128-8*len(longitude),line(2),1)
        #oled.text(speedKts,128-8*len(speedKts),line(3),1)
        oled.text(altitude,128-8*len(altitude),line(3),1)
        radius = 2
    if (FIX_TIME):
        oled.text(GPStime,128-8*len(GPStime),line(7),1)
 
    # Populate the satellite display
    for sat in satList:
        openCircle(sat[1],sat[2], radius)
    for sat in satList:
        for used in usedList:
            if (sat[0] == used):
                closedCircle(sat[1],sat[2], radius)
                break
    oled.show()

#
# Main code
#

time.sleep_ms(100)
writeSentence("PMTK605", uartGPS)  # Query GPS
writeSentence("PGCMD,33,1", uartGPS)  # Enable antenna output for PA6H
writeSentence("CDCMD,33,1", uartGPS)  # Enable antenna output for PA1616S
time.sleep_ms(100)
oled.contrast(0x10)  # Dim the display to not be full brightness

while True:
    if (pinExit.value() == 0):  # Check if pinExit was grounded as an exit signal
        oled.fill(0)
        oled.show()
        print("Exiting program.")
        break

    # Check for data from computer, allows possible control of GPS output
    inputData = uartSerial.readline()
    if (str(inputData) != "None"):
        if (DEBUG_APP):
            print("Computer: " + str(inputData))
        uartGPS.write(inputData)  # Send the input data to the GPS module

    # Check for data from the GPS module
    inputData = uartGPS.readline()
    if (ECHO_GPS and str(inputData) != "None"):
        if (DEBUG_APP):
            print("GPS: " + str(inputData))
        uartSerial.write(inputData)  # Echo the input data to UART1

    # Convert to ascii
    try:
        buff = str(inputData, 'ascii')  # NMEA is ASCII by definition
    except:
        print("Unable to convert input data to ascii")
        print(str(inputData))
        GPGSV_IN_PROGRESS = False; # In case $GPGSV is in progress, reset
        blinkLED(neoRed, 100)
        continue

    # Validate received data
    if (verifySentence(buff) != True):
        print("Ignoring invalid data")
        GPGSV_IN_PROGRESS = False; # In case $GPGSV is in progress, reset
        blinkLED(neoRed, 100)
        continue

    if (DEBUG_APP):
        print(buff[:len(buff)-2])

    # Split the NMEA sentence into comma separated elements
    elems = buff[:len(buff)-5].split(',')
    if (len(elems) < 2):
        TIMEOUT = True
        continue
    TIMEOUT = False
    if (satListTime <= time.time() - 30): # No $GPGSV for 30 seconds
        satList.clear()
        tempSatList.clear()

    try:
        # Handle the $GPRMC sentence - time, position and speed
        if (elems[0] == "$GPRMC"):
            if (elems[1]):
                GPStime = elems[1][0:2] + ':' + elems[1][2:4] + ':' + elems[1][4:6] + 'Z'
                FIX_TIME = True
            else:
                FIX_TIME = False;
            if (elems[2] == "A"):  # A = valid, V = not valid
                if (elems[3] and elems[4] and elems[5] and elems[6]):
                    FIX_STATUS = True
                    latitude = convertToDecimalDegree(elems[3])
                    latitude = latitude + elems[4]
                    longitude = convertToDecimalDegree(elems[5])
                    longitude = longitude + elems[6]
                if (elems[7]):
                    kts = float(elems[7])
                    if (kts < 10.0):
                        speedKts = "{:5.1f}".format(kts) + "kn"
                    else:
                        speedKts = "{:5.0f}".format(kts) + "kn"
            else:
                FIX_STATUS = False

            if (FIX_STATUS):
                blinkLED(neoGreen if (antenna == "Internal") else neoBlue)
            elif (FIX_TIME):
                blinkLED(neoPurple)
            else:
                blinkLED(neoRed)
            updateUI()

        # Handle the $GPGGA sentence - number of satellites used (also time and position)
        if (elems[0] == "$GPGGA"):
            if (elems[7]):
                satellites = "Sat: " + elems[7]
            if (elems[9]):
                alt = float(elems[9])
                if (alt < 1000.0):
                    altitude = "{:6.1f}".format(alt) + "m"
                else:
                    altitude = "{:6.0f}".format(alt) + "m"

        # Handle the $GPGSV sentence - Multipart list of satellites - El/Az/RSSI
        if (elems[0] == "$GPGSV"):
            # Multipart, use temporary list
            if (elems[2] == "1"):
                tempSatList.clear()
                numGSV = elems[1]
                GPGSV_IN_PROGRESS = True
            numSatsInGSV = min(4, int(elems[3]) - 4*(int(elems[2]) - 1))
            if (GPGSV_IN_PROGRESS):
                for i in range(4, 4*numSatsInGSV + 1, 4):
                    if (elems[i] and elems[i+1] and elems[i+2]):
                        tempSatList.append((int(elems[i]), int(elems[i+1]), int(elems[i+2]))) # Sat, El, Az
                    else:
                        break
                if (elems[2] == numGSV):  # Last one received
                    GPGSV_IN_PROGRESS = False
                    satListTime = time.time()
                    satList.clear()
                    for sat in tempSatList:
                        satList.append(sat)
                    tempSatList.clear()

        # Handle the $GPGSA sentence - Fix mode and list of satellites used for acquisition
        if (elems[0] == "$GPGSA"):
            usedList.clear()
            mode3D = elems[2] + "D Fix"
            if (elems[2] == "1"):
                mode3D = "No Fix"
            for i in range(3, 15):
                if (elems[i]):
                    usedList.append(int(elems[i]))
                else:
                    break

        if (elems[0] == "$PMTK705"): # Version info
            blinkLED(neoWhite, 200)
        if (elems[0] == "$PGTOP"): # PA6H antenna output
            if (elems[2] and elems[2] == "2"):
                antenna = "Internal"
            if (elems[2] and elems[2] == "3"):
                antenna = "External"
        if (elems[0] == "$PCD"): # PA1616S antenna output
            if (elems[2] and elems[2] == "1"):
                antenna = "Internal"
            if (elems[2] and elems[2] == "2"):
                antenna = "External"
 
    except:
        print("Exception caught")

