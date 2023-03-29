from __future__ import division
import pygame
import RPi.GPIO as GPIO
from time import sleep, time, strftime, localtime



# Import the PCA9685 module.
import Adafruit_PCA9685
pwm = Adafruit_PCA9685.PCA9685()
pygame.init()

#PS4CONTROLLER INNIT
joystick = pygame.joystick.Joystick(0)
joystick.init()
GPIO.setmode(GPIO.BOARD)

servo_pin1 = 33
servo_pin2 = 32
# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096
# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    #print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    #print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)



try:
    while True:
        

        pygame.event.get()
        servo_horiz_axis = int((joystick.get_axis(3)*225)+375)
        servo_verti_axis = int(((-joystick.get_axis(4))*225)+375)
        servo_horiz_axis1 = int((joystick.get_axis(1)*225)+360)
        servo_verti_axis1 = int(((-joystick.get_axis(0))*225)+360)
        #print(servo_horiz_axis1)
        
       
        pwm.set_pwm(0, 0, servo_horiz_axis)
        pwm.set_pwm(1, 0, servo_verti_axis)
        pwm.set_pwm(2, 0, servo_horiz_axis1)
        pwm.set_pwm(3, 0, servo_verti_axis1)
        
except KeyboardInterrupt:
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
# 
