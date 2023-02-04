# RP2040-Zero-GPS-OLED
- MicroPython RP2040-Zero GPS receiver with OLED display

- Operation:

  Main loops forever reading GPS data.  When a $GPRMC sentence is received the UI is updated, which will be about once per second.  In addition, the GPS sentences are echoed to a second UART, in my hardware implementation a UART to RS232 converter is used so that the data can also be fed to a computer.

  GPIO8 is monitored and program will exit if it detects that is grounded.

- Requirements:

  This version is written for integrated device w/ RP2040-zero, Adafruit GPS breakout, Max2332 for serial port and OLED ssd1306 display.  Requires ssd1306.py located at https://peppe8o.com/download/micropython/ssd1306/ssd1306.py (implements 180 degree rotation).

