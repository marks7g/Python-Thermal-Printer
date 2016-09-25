#!/usr/bin/python

# Main script for AMELABS Pi Printer. 
# RaspberryPI + Thermal Receipt Printer + pimoroni blinkt + 2 arcade buttons and LEGO
# for taps and holds, performs periodic actions (Twitter polling by default)
# and daily actions (Sudoku and weather by default).
#
# Inspired by some Adafruit projects and code
#
# MUST BE RUN AS ROOT (due to GPIO access)
#
# Required software includes Adafruit_Thermal, Python Imaging and PySerial
# libraries. Other libraries used are part of stock Python install.
#
# Resources:
# http://www.adafruit.com/products/597 Mini Thermal Receipt Printer
# http://www.adafruit.com/products/600 Printer starter pack

# need to add blinkt stuff


from __future__ import print_function
import RPi.GPIO as GPIO
import subprocess, time, Image, socket
from Adafruit_Thermal import *


# for blinkt
import colorsys
import time
from blinkt import set_clear_on_exit, set_brightness, set_pixel, show


#ledPin       = 18     # remove LED and add blinkt
button1Pin    = 23     # left green button. willa
button2Pin    = 24    # right green button. myles
holdTime     = 2     # Duration for button hold (shutdown)
tapTime      = 0.01  # Debounce time for button taps
nextInterval = 0.0   # Time of next recurring operation
dailyFlag    = False # Set after daily trigger occurs
lastId       = '1'   # State information passed to/from interval script
printer      = Adafruit_Thermal("/dev/ttyAMA0", 19200, timeout=5)


# Called when button1 is briefly tapped.  Invokes time/temperature script.
def tap1():
#  GPIO.output(ledPin, GPIO.HIGH)  # LED on while working
#  subprocess.call(["python", "timetemp.py"])
  subprocess.call(["python", "acmeblinkt.py"])
#  GPIO.output(ledPin, GPIO.LOW)

# Called when button2 is briefly tapped.  Invokes time/tempeture script.
def tap2():
#  GPIO.output(ledPin, GPIO.HIGH)  # LED on while working
#  subprocess.call(["python", "timetemp.py"])
  subprocess.call(["python", "acmeblinkt.py"])
#  GPIO.output(ledPin, GPIO.LOW)


# Called when button1 is held down. 
def hold1():
  GPIO.output(ledPin, GPIO.HIGH)
  printer.printImage(Image.open('gfx/goodbye.png'), True)
  printer.feed(3)
#  subprocess.call("sync")
#  subprocess.call(["shutdown", "-h", "now"])
#  GPIO.output(ledPin, GPIO.LOW)

# Called when button is held down.  Prints image, invokes shutdown process.

def hold()):
  GPIO.output(ledPin, GPIO.HIGH)
  printer.printImage(Image.open('gfx/goodbye.png'), True)
  printer.feed(3)
  subprocess.call("sync")
  subprocess.call(["shutdown", "-h", "now"])
  GPIO.output(ledPin, GPIO.LOW)  


# Called at periodic intervals (30 seconds by default).
# Invokes twitter script.
def interval():
  GPIO.output(ledPin, GPIO.HIGH)
  p = subprocess.Popen(["python", "twitter.py", str(lastId)],
    stdout=subprocess.PIPE)
  GPIO.output(ledPin, GPIO.LOW)
  return p.communicate()[0] # Script pipes back lastId, returned to main


# Called once per day (6:30am by default).
# Invokes weather forecast and sudoku-gfx scripts.
def daily():
  GPIO.output(ledPin, GPIO.HIGH)
  subprocess.call(["python", "forecast.py"])
  subprocess.call(["python", "sudoku-gfx.py"])
  GPIO.output(ledPin, GPIO.LOW)


# Initialization

# Use Broadcom pin numbers (not Raspberry Pi pin numbers) for GPIO
GPIO.setmode(GPIO.BCM)

# Enable LED and button (w/pull-up on latter)
#GPIO.setup(ledPin, GPIO.OUT)
GPIO.setup(button1Pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(button2Pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)


# LED on while working
#GPIO.output(ledPin, GPIO.HIGH)

# Processor load is heavy at startup; wait a moment to avoid
# stalling during greeting.
time.sleep(30)

# Show IP address (if network is available)
try:
	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	s.connect(('8.8.8.8', 0))
	printer.print('My IP address is ' + s.getsockname()[0])
	printer.feed(3)
except:
	printer.boldOn()
	printer.println('Network is unreachable.')
	printer.boldOff()
	printer.print('Connect display and keyboard\n'
	  'for network troubleshooting.')
	printer.feed(3)
	exit(0)

# Print greeting image
printer.printImage(Image.open('gfx/hello.png'), True)
printer.feed(3)
# GPIO.output(ledPin, GPIO.LOW)

# Poll initial button state and time
prevButton1State = GPIO.input(button1Pin)
prevTime1        = time.time()
tapEnable       = False
holdEnable      = False

# Main loop
while(True):

  # Button1

  # get working with 2 buttons. 
  # Poll current button state and time
  button1State = GPIO.input(button1Pin)  
  t            = time.time()
  t1           = time.time()

  # Has button1 state changed?
  if button1State != prevButton1State:
    prevButton1State = button1State   # Yes, save new state/time
    prevTime1        = t1
  else:                             # Button state unchanged
    if (t1 - prevTime1) >= holdTime:  # Button held more than 'holdTime'?
      # Yes it has.  Is the hold action as-yet untriggered?
      if holdEnable == True:        # Yep!
        hold1()                      # Perform hold action (usu. shutdown)
        holdEnable = False          # 1 shot...don't repeat hold action
        tapEnable  = False          # Don't do tap action on release
    elif (t1 - prevTime) >= tapTime: # Not holdTime.  tapTime elapsed?
      # Yes.  Debounced press or release...
      if button1State == True:       # Button released?
        if tapEnable == True:       # Ignore if prior hold()
          tap1()                     # Tap triggered (button released)
          tapEnable  = False        # Disable tap and hold
          holdEnable = False
      else:                         # Button pressed
        tapEnable  = True           # Enable tap and hold actions
        holdEnable = True

 # Button2

 # Poll current button state and time
  button2State = GPIO.input(button2Pin)
  t2           = time.time()

  # Has button1 state changed?
  if button2State != prevButton2State:
    prevButton2State = button2State   # Yes, save new state/time
    prevTime2        = t2
  else:                             # Button state unchanged
    if (t2 - prevTime2) >= holdTime:  # Button held more than 'holdTime'?
      # Yes it has.  Is the hold action as-yet untriggered?
      if holdEnable == True:        # Yep!
        hold2()                      # Perform hold action (usu. shutdown)
        holdEnable = False          # 1 shot...don't repeat hold action
        tapEnable  = False          # Don't do tap action on release
    elif (t2 - prevTime2) >= tapTime: # Not holdTime.  tapTime elapsed?
      # Yes.  Debounced press or release...
      if button2State == True:       # Button2 released?
        if tapEnable == True:       # Ignore if prior hold()
          tap2()                     # Tap triggered (button released)
          tapEnable  = False        # Disable tap and hold
          holdEnable = False
      else:                         # Button pressed
        tapEnable  = True           # Enable tap and hold actions
        holdEnable = True



  # LED blinks while idle, for a brief interval every 2 seconds.
  # Pin 18 is PWM-capable and a "sleep throb" would be nice, but
  # the PWM-related library is a hassle for average users to install
  # right now.  Might return to this later when it's more accessible.
 
 # if ((int(t) & 1) == 0) and ((t - int(t)) < 0.15):
 #   GPIO.output(ledPin, GPIO.HIGH)
 # else:
 #  GPIO.output(ledPin, GPIO.LOW)

  # Once per day (currently set for 6:30am local time, or when script
  # is first run, if after 6:30am), run forecast and sudoku scripts.
  l = time.localtime()
  if (60 * l.tm_hour + l.tm_min) > (60 * 6 + 30):
    if dailyFlag == False:
      daily()
      dailyFlag = True
  else:
    dailyFlag = False  # Reset daily trigger

  # Every 30 seconds, run Twitter scripts.  'lastId' is passed around
  # to preserve state between invocations.  Probably simpler to do an
  # import thing.
  if t > nextInterval:
    nextInterval = t + 30.0
    result = interval()
    if result is not None:
      lastId = result.rstrip('\r\n')

