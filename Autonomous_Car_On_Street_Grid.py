'''
    --Autonomous Driving Program--
    
    Platform:
    
    -Sunfounder Picar-X
    
    Implemented Functionalities:
    
    -Google Firebase Descision Making
    
    -Adafruit 9-AXIS Inertail Measurement Unit Forward Steering Alignment
    
    -Line Following program for Picar-X  

    -Dual IR Wing Sensors For Lane Detection
    
'''

import board
import adafruit_bno055
import numpy as np
import pyrebase
# from .pin import Pin
from picarx import Picarx
from time import sleep
import RPi.GPIO as GPIO

config = {
    "apiKey" : "*******************",
    "authDomain" : "******************",
    "databaseURL" : "********************",
    "storageBucket" : "*********************"
    }


firebase = pyrebase.initialize_app(config)
db = firebase.database()
px = Picarx()
i2c = board.I2C()  # uses board.SCL and board.SDA
last_val = 0xFFFF
sampling_rate = 100
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor = adafruit_bno055.BNO055_I2C(i2c)
px = Picarx(grayscale_pins=['A0', 'A1', 'A2']) 
px.set_grayscale_reference(500)
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN)
GPIO.setup(4, GPIO.IN)
#LeftWing = GPIO.PIN(17) #Pin("D0")
#RightWing = GPIO.PIN(18) #Pin("D1")
# Wings = [17,18]
# px.grayscale.reference = 1400
innit_angle = 0
last_state = None
current_state = None
px_power = 10
offset = 20
paused = 0
left_wing = 4
right_wing = 17
steer = 0

#function occurs when car goes into an intersection (triggered by going off the black line that it is following)
#detected by the middle three grayscale sensors
def outHandle():
    global last_state, current_state
    #read the firebase descision
    x = db.child("pi").child("1-set").child('x').get()
    #if firebase gives 0 it turns left
    px.forward(10)
    if x.val() == 0:
        left()
    #firebase is 1 go straight
    elif x.val() == 1:
        straight()   
    #firebase is anything else go right
    else:
        right()
           
    #steer the direction until finding the new line
        
            
    
def straight():
    innit_angle = sensor.euler[0]
    while GPIO.input(left_wing) != 1 and GPIO.input(right_wing) != 1:
        gm_val_list = px.get_grayscale_data()
        gm_state = px.get_line_status(gm_val_list)
        if gm_state != 'stop':
            break
            #euler angle consistent
        if (sensor.euler[0] is not None):
            if int(innit_angle - sensor.euler[0]) >= 2:
                px.set_dir_servo_angle(10)
            elif int(innit_angle - sensor.euler[0]) <= -2:
                px.set_dir_servo_angle(-10)
            else:
                px.set_dir_servo_angle(0)

def left():
    steer = -35
    px.set_dir_servo_angle(steer)
    while True:
        if GPIO.input(left_wing) == 1 or GPIO.input(right_wing) == 1:
            escapelines()
        gm_val_list = px.get_grayscale_data()
        gm_state = px.get_line_status(gm_val_list)
        if gm_state == 'forward':
            break
def right():
    steer = 35
    px.set_dir_servo_angle(steer)
    while True:
        if GPIO.input(left_wing) == 1 or GPIO.input(right_wing) == 1:
            escapelines()
        gm_val_list = px.get_grayscale_data()
        gm_state = px.get_line_status(gm_val_list)
        if gm_state == 'forward':
            break
        
def escapelines():
    while GPIO.input(left_wing) != 0:
        gm_val_list = px.get_grayscale_data()
        gm_state = px.get_line_status(gm_val_list)
        if gm_state != 'stop':
            px.set_dir_servo_angle(-35)
            px.forward(-10)
            sleep(.1)
        px.set_dir_servo_angle(35)
        px.forward(10)
        #time.sleep(0.1)
        
    while GPIO.input(right_wing) != 0:
        gm_val_list = px.get_grayscale_data()
        gm_state = px.get_line_status(gm_val_list)
        if gm_state != 'stop':
            px.set_dir_servo_angle(35)
            px.forward(-10)
            sleep(.1)
        px.set_dir_servo_angle(-35)
        px.forward(10)
        #time.sleep(0.1)
        

if __name__=='__main__':
    try:
        while True:
            paused = 0
            gm_val_list = px.get_grayscale_data()
            distance = px.ultrasonic.read()
            gm_state = px.get_line_status(gm_val_list)
#             print("gm_val_list: %s, %s"%(gm_val_list, gm_state))
            if gm_state != "stop":
                last_state = gm_state

            if gm_state == 'forward':
                px.set_dir_servo_angle(0)
                px.forward(px_power) 
            elif gm_state == 'left':
                px.set_dir_servo_angle(offset)
                px.forward(px_power) 
            elif gm_state == 'right':
                px.set_dir_servo_angle(-offset)
                px.forward(px_power) 
            else:
                outHandle()
                
    except KeyboardInterrupt:
        px.stop()