import os     #importing os library so as to communicate with the system
import time   #importing time library to make Rpi wait because its too impatient 
os.system ("sudo pigpiod") #Launching GPIO library
time.sleep(1) # As i said it is too impatient and so if this delay is removed you will get an error
import pigpio #importing GPIO library

ESC=4  #Connect the ESC in this GPIO pin 
max_value = 1900 #change this if your ESC's max value is different or leave it be
min_value = 1100  #change this if your ESC's min value is different or leave it be

pi = pigpio.pi();

pi.set_servo_pulsewidth(ESC, 0)
time.sleep(1)
# pi.set_servo_pulsewidth(ESC, max_value)
# time.sleep(1)
# pi.set_servo_pulsewidth(ESC, min_value)
# time.sleep(1)
pi.set_servo_pulsewidth(ESC, 1500)
time.sleep(0.5)
pi.set_servo_pulsewidth(ESC, 2000)
time.sleep(2)
pi.set_servo_pulsewidth(ESC, 0)
pi.stop()