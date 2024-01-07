#
# This file is part of the BioLab project. (https://paulzanna.com)
# Copyright (c) 2023 Paul Zanna.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# Author: Paul Zanna <mail@paulzanna.com>
#
# To run from REPL type "exec(open('main.py').read())"
#
#

# Imports
import machine
import time
import uselect
import sh1106
import math
from math import sqrt, trunc
from time import sleep_ms, ticks_ms
from machine import SPI, Pin, PWM
from usys import stdin
from uselect import poll

# Global variables
rpm_counter = 0
rpmtemp = 1000
rcftemp = 100
timetemp = 30
timetemp_hrs = 0
timetemp_mins = 0
timetemp_secs = 0
last_rpm = 0
last_time = 0
last_button = 0
run_state = 0
start_time = 0

# Constants
SAMPLING_TIME = 500 # RPM sample time in microseconds
STOP_SPEED = 1300000 # Stop speed
START_SPEED = 1360000 # Start speed
MAX_SPEED = 1470000 # Max speed
RPM_MIN = 0 # Minimum RPM setting
RPM_MAX = 4000 # Maximum RPM setting
RCF_MIN = 0 # Minimum RCF setting
RCF_MAX = 1960 # Maximum RCF setting
TIME_MIN = 0 # Minimum time setting in seconds
TIME_MAX = 3600 # Maximum time setting in seconds (1 hour)

# Variables
last_revolutions_per_minute = 0 # Last RPM measurement
avg_speed_RPM = 0 # average speed units per RPM
rnd_revolutions_per_minute = 0 # RPMs rounded to the nearest 50 RPMs
relative_centrifugal_force = 0 # Relative centrifugal force
Run_Speed = STOP_SPEED # Start with the stop speed
RPM_Set = 1000 # Current RPM setting
RCF_SET = 100 # Current RCF
Time_Set = 30 # Current Time setting (Total seconds)
Time_Set_HRS = 0 # Current Time setting (Hours)
Time_Set_MINS = 0 # Current Time setting (Minutes)
Time_Set_SECS = 0 # Current Time setting (Seconds)
Time_Remaining = 0 # Time remaining (Total seconds)
Time_Remaining_HRS = 0 # Time remaining (Hours)
Time_Remaining_MINS = 0 # Time remaining (Minutes)
Time_Remaining_SECS = 0 # Time remaining (Seconds)
RPM_Error = 0 # RPM error value

# Pins for motor
pwm = PWM(Pin(15)) # Motor control - White
TachoPin = Pin(14, Pin.IN) # Speed sensor - Purple

# Pins for Button
ButtonPin = Pin(9, Pin.IN, Pin.PULL_UP) # Start button switch- Grey
ButtonLED = Pin(8, Pin.OUT) # Start button LED - White

# RPM/RCF switch
RcfPin = Pin(4, Pin.IN, Pin.PULL_UP) # RPM/RCF switch - Red

# Pin for Buzzer
buzzer = PWM(Pin(6)) # Piezo buzzer - Orange

# using default address 0x3C
spi = SPI(0, 100000, mosi=Pin(3), sck=Pin(2)) # MOSI - Purple, Clock - Grey
display = sh1106.SH1106_SPI(128, 64, spi, Pin(0), Pin(7), Pin(1)) # DC - Green, RES - Blue, CS - Yellow
display.sleep(False) # Turn off sleep mode
display.fill(0) # Fill the display buffer with black

# Pins for encoder and button
encoder_pin_clk1 = Pin(26, Pin.IN, Pin.PULL_UP) # Encoder 1 clock pin (RPMs) - Orange
encoder_pin_dt1 = Pin(27, Pin.IN, Pin.PULL_UP) # Encoder 1 data pin (RPMs) - Red
encoder_pin_sw1 = Pin(13, Pin.IN, Pin.PULL_UP) # Encoder 1 switch pin (RPMs) - Brown
encoder_pin_clk2 = Pin(10, Pin.IN, Pin.PULL_UP) # Encoder 2 clock pin (Time) - Yellow
encoder_pin_dt2 = Pin(11, Pin.IN, Pin.PULL_UP) # Encoder 2 data pin (Time) - Green
encoder_pin_sw2 = Pin(12, Pin.IN, Pin.PULL_UP) # Encoder 2 switch pin (Time) - Blue

# Calibration
print("Calibrating")
print("Setting Min Speed")
display.init_display() # Initialise Display
display.text('Calibrating.', 0, 0, 1)
display.text('Set Min Speed.', 0, 10, 1)
display.show()
pwm.freq(50) # Set the PWN frequency for the ESC to 50Hz
pwm.duty_ns(STOP_SPEED) # Set the duty cycle to the STOP speed, you should hear a beep from the ESC.
time.sleep(5) # Wait for 5 seconds
Run_Speed = START_SPEED
pwm.duty_ns(Run_Speed) # Set the duty cycle to the START speed

# Clear display
display.fill(0)
display.show()

# Play buzzer
buzzer.freq(500)
buzzer.duty_u16(1000)
time.sleep(1)
buzzer.duty_u16(0)

# Interrupt functions
def button_press(pin):
    global run_state, last_button, start_time
    new_button = time.ticks_ms() # if it has been less than 1/5 of a second since the last press it is ignored
    if (new_button - last_button) > 200:
        if run_state == 0:
            run_state = 1
            start_time = (time.ticks_ms()//1000)
        else:
            run_state = 0
            start_time = 0
        # Play buzzer
        buzzer.freq(1000)
        buzzer.duty_u16(1000)
        time.sleep_ms(200)
        buzzer.duty_u16(0)        
    new_button = last_button

# RPM counter interupt
def rpm_isr(pin):
  global rpm_counter
  rpm_counter += 1

# RPM/RCF speed set function
def rpm_select(pin):
    global RPM_Set, rpmtemp, rcftemp, RCF_Set
    if run_state == 0:
        RCF_Set = rcftemp
        RPM_Set = rpmtemp
            
    # Play buzzer
    buzzer.freq(1000)
    buzzer.duty_u16(1000)
    time.sleep_ms(200)
    buzzer.duty_u16(0)

# RPM rotate encoder function
def rpm_change(pin):
    global RPM_MIN, RPM_MAX, RCF_MIN, RCF_MAX, rpmtemp, rcftemp, last_rpm
    new_rpm = time.ticks_ms() # if it has been less than a 1/10th of a second since the last press it is ignored
    if (new_rpm - last_rpm) > 100:
        if encoder_pin_clk1.value() == encoder_pin_dt1.value():
            if RcfPin.value() == 0:
                if rpmtemp < RPM_MAX:
                    rpmtemp += 100
            else:
                if rcftemp < RCF_MAX:
                    rcftemp += 10
        else:
            if RcfPin.value() == 0:
                if rpmtemp > RPM_MIN:
                    rpmtemp -= 100
            else:
                if rcftemp > RCF_MIN:
                    rpmtemp -= 10
    last_rpm = new_rpm

# Time set function
def time_select(pin):
    global Time_Set, timetemp
    Time_Set = timetemp
    # Play buzzer
    buzzer.freq(1000)
    buzzer.duty_u16(1000)
    time.sleep_ms(200)
    buzzer.duty_u16(0)

# RPM rotate encoder function
def time_change(pin):
    global Time_Set, timetemp, last_time
    new_time = time.ticks_ms() # if it has been less than a 1/10th of a second since the last press it is ignored
    if (new_time - last_time) > 100:
        if encoder_pin_clk2.value() == encoder_pin_dt2.value():
            if timetemp < TIME_MAX:
                timetemp += 10
        else:         
            if timetemp > TIME_MIN:
                timetemp -= 10
    last_time = new_time
    
# Interrupts
TachoPin.irq(trigger=Pin.IRQ_FALLING,handler=rpm_isr) # Interrupt registration for RPM IR sensor
encoder_pin_sw1.irq(trigger=Pin.IRQ_RISING,handler=rpm_select) # Interrupt registration for RPM set button
encoder_pin_dt1.irq(trigger=Pin.IRQ_FALLING,handler=rpm_change) # Interrupt registration for RPM rotate dial
encoder_pin_sw2.irq(trigger=Pin.IRQ_RISING,handler=time_select) # Interrupt registration for time set button
encoder_pin_dt2.irq(trigger=Pin.IRQ_FALLING,handler=time_change) # Interrupt registration for time rotate dial
ButtonPin.irq(trigger=Pin.IRQ_FALLING,handler=button_press) # Interrupt registration for START/STOP button

# Main Code
print("Starting")

while True:
    print(str(rpmtemp) + " - " + str(rcftemp) +" :{:4d}".format(timetemp), Time_Set, Time_Remaining, start_time)
    time.sleep_ms(SAMPLING_TIME)
    revolutions_per_sampling_time = rpm_counter
    if rpm_counter > 220: # If count is over 4400 RPMs the tube has stopped on the sensor
        rpm_counter = 0
    revolutions_per_minute = (revolutions_per_sampling_time * (60000 // SAMPLING_TIME)) // 6
    revolutions_per_minute = (last_revolutions_per_minute + revolutions_per_minute) // 2
    last_revolutions_per_minute = revolutions_per_minute
    rnd_revolutions_per_minute = (revolutions_per_minute // 10) * 10
    
    if revolutions_per_minute > 0:
        avg_speed_RPM = (Run_Speed - START_SPEED)//revolutions_per_minute # ESC duty cycle setting per RPM
        
    if RPM_Set > 0:
        RPM_Error = ((RPM_Set - revolutions_per_minute) / RPM_Set) * 100 # Differnce between set speed and actual speed
    
    print("RPM Set : " + str(RPM_Set) + " ("+ str(Run_Speed - START_SPEED) + ") - RPM : ", rnd_revolutions_per_minute, " (", revolutions_per_sampling_time, ") - AVG : " + str(avg_speed_RPM))
    
    # Calculate time
    if run_state == 0:
        timetemp_hrs = timetemp // 3600
        timetemp_mins = (timetemp - (timetemp_hrs * 3600)) // 60
        timetemp_secs = timetemp - ((timetemp_hrs * 3600) + (timetemp_mins * 60))
        Time_Set_HRS = Time_Set // 3600
        Time_Set_MINS = (Time_Set - (Time_Set_HRS * 3600)) // 60
        Time_Set_SECS = Time_Set - ((Time_Set_HRS * 3600) + (Time_Set_MINS * 60))
    else:
        Time_Remaining = Time_Set - ((time.ticks_ms()//1000) - start_time)
        Time_Remaining_HRS = Time_Remaining // 3600
        Time_Remaining_MINS = (Time_Remaining - (Time_Remaining_HRS * 3600)) // 60
        Time_Remaining_SECS = Time_Remaining - ((Time_Remaining_HRS * 3600) + (Time_Remaining_MINS * 60))
    
    # Calculate RPM/RCF (RCF = (RPM)2 × 0.00001118 × r) : (RPM = SQRT(RCF / (0.00001118 x r)
    relative_centrifugal_force = (rnd_revolutions_per_minute * rnd_revolutions_per_minute) * 11 * 0.00001118
    if RcfPin.value() == 0:
        rcftemp = (rpmtemp * rpmtemp) * 11 * 0.00001118
    else:
        rpmtemp = trunc(sqrt(rcftemp // (11 * 0.00001118)))
    
    # Update display
    display.fill(0)
    if run_state == 0:
        if RcfPin.value() == 0:
            display.text('RPM  : ' + str(rpmtemp), 0, 0, 1)
            display.text('RCF  : {:.0f}'.format(rcftemp), 0, 10, 1) # Radius is 11cm
        else:                    
            display.text('RCF  : {:.0f}'.format(rcftemp), 0, 0, 1) # Radius is 11cm
            display.text('RPM  : ' + str(rpmtemp), 0, 10, 1)
        display.text('Time : {:02d}'.format(timetemp_hrs) + ':{:02d}'.format(timetemp_mins) + ':{:02d}'.format(timetemp_secs), 0, 20, 1)
    else:
        if RcfPin.value() == 0:
            display.text('RPM  : ' + str(rnd_revolutions_per_minute), 0, 0, 1)
            display.text('RCF  : {:.0f}'.format(relative_centrifugal_force), 0, 10, 1)
        else:
            display.text('RCF  : {:.0f}'.format(relative_centrifugal_force), 0, 0, 1)
            display.text('RPM  : ' + str(rnd_revolutions_per_minute), 0, 10, 1)
        display.text('Time : {:02d}'.format(Time_Remaining_HRS) + ':{:02d}'.format(Time_Remaining_MINS) + ':{:02d}'.format(Time_Remaining_SECS), 0, 20, 1)
    
    display.show()
    
    # Stop motor when timer reaches 0
    if Time_Remaining <= 0 and run_state == 1:
        run_state = 0
        buzzer.freq(500)
        buzzer.duty_u16(1000) # Play buzzer
        time.sleep(2)
        buzzer.duty_u16(0)
    
    if revolutions_per_minute < RPM_Set and run_state == 1:
        Run_Speed += (RPM_Set - revolutions_per_minute)
        if Run_Speed < MAX_SPEED:
            pwm.duty_ns(Run_Speed)
    
    if revolutions_per_minute > RPM_Set and run_state == 1:
        Run_Speed -= (revolutions_per_minute - RPM_Set)
        if Run_Speed < START_SPEED:
            Run_Speed = START_SPEED
        pwm.duty_ns(Run_Speed)

    if revolutions_per_minute == 0 and run_state == 1:
        if RPM_Set <= 1000:
            Run_Speed = START_SPEED + (RPM_Set * 12) # Set initial run speed at 12 times the required RPMs
        elif RPM_Set > 1000 and RPM_Set <= 2000:
            Run_Speed = START_SPEED + (RPM_Set * 10) # Set initial run speed at 10 times the required RPMs
        elif RPM_Set > 2000 and RPM_Set <= 3000:
            Run_Speed = START_SPEED + (RPM_Set * 12) # Set initial run speed at 12 times the required RPMs
        elif RPM_Set > 3000:
            Run_Speed = START_SPEED + (RPM_Set * 10) # Set initial run speed at 10 times the required RPMs
            
        if Run_Speed > MAX_SPEED:
            Run_Speed = MAX_SPEED
        pwm.duty_ns(Run_Speed)
    
    # Flash the START/STOP when running 
    if run_state == 0:
        pwm.duty_ns(0)
        Run_Speed = START_SPEED
        ButtonLED.value(1)
    else:
        ButtonLED.toggle()
    
    # Make sure that if we have switched from RCF to RPM mode that RPMs are round back to the nearest 100
    rpmtemp = (rpmtemp // 100) * 100
    rcftemp = (rcftemp // 10) * 10
    # reset the RPM counter to zero 
    rpm_counter = 0
    
