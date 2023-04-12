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
from picarx import Picarx
import RPi.GPIO as GPIO
from robot_hat.utils import reset_mcu
from vilib import Vilib
from time import sleep, time, strftime, localtime
import readchar
import pygame
import pyrebase

config = {
    "apiKey" : "76o1qRJhT5mxuUsDVJGGB1yQvA1llL7HTEKU0Z5m",
    "authDomain" : "seniordesignproj-cef3d.firebaseapp.com",
    "databaseURL" : "https://seniordesignproj-cef3d-default-rtdb.firebaseio.com",
    "storageBucket" : "seniordesignproj-cef3d.appspot.com"
    }


firebase = pyrebase.initialize_app(config)
db = firebase.database()

reset_mcu()
sleep(0.2)
pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

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
#Vilib.camera_start(vflip=False,hflip=False)
#Vilib.display(local=True,web=True)
innit_angle = 0
last_state = None
current_state = None
px_power = 10
offset = 20
paused = 0
left_wing = 4
right_wing = 17
steer = 0
direction_index = 0
#direction_array = ['south', 'left', 'straight' ,'right','left','straight' ,'end']
direction_array = db.child("traxxas").child('path').get().val()

if direction_array[0] == 'north':
    orientation = 1
elif direction_array[0] == 'east':
    orientation = 2
elif direction_array[0] == 'south':
    orientation = 3
elif direction_array[0] == 'west':
    orientation = 0


    
def Calib_ang():
    global direction_array, orientation
    if orientation == 0:
        innit_angle = 28
    elif orientation == 1:
        innit_angle = 128
    elif orientation == 2:
        innit_angle = 208
    elif orientation == 3:
        innit_angle = 298
    rotate = 'none'
    rotate_cnt = 0
    #tweak for debug
    rotate_sharpness = 100
    rotate_tol = 2
    while True:
    
        if (sensor.euler[0] is not None):
            euler_angle = sensor.euler[0]
            if (euler_angle is not None):
                if euler_angle >= innit_angle:
                    rotate = "left"
                elif euler_angle <= innit_angle:
                    rotate = "right"
                if euler_angle <= innit_angle + rotate_tol and euler_angle >= innit_angle - rotate_tol:
                    rotate = "done"
                    straight_call()
                    
        if rotate == "right":
            if rotate_cnt <= rotate_sharpness:
                px.set_dir_servo_angle(-35)
                px.forward(-5)
            else:
                px.set_dir_servo_angle(35)
                px.forward(5)

               
        elif rotate == "left":
            if rotate_cnt <= rotate_sharpness:
                px.set_dir_servo_angle(35)
                px.forward(-5)
            else:
                px.set_dir_servo_angle(-35)
                px.forward(5)
        else:
            continue
        rotate_cnt +=1
        if rotate_cnt == rotate_sharpness*2 +1:
           rotate_cnt = 0
           
    

def outHandle():
    global last_state, current_state, direction_index, orientation
    #read the firebase descision
    #db.child("Traxxas").child("ID5").set(orientation)
    print('deciding')
    
    immediate_direction = direction_array[direction_index]
    direction_index += 1
    print('decided', immediate_direction)
    #if firebase gives 0 it turns left

    if immediate_direction == 'left':
        left()
    #firebase is 1 go straight
    elif immediate_direction == 'straight':
        straight_call()   
    elif immediate_direction == 'right':
        right()
    elif immediate_direction == 'end':
        
        px.stop()
        manual_control()
    elif immediate_direction == 'south' or 'north' or 'east' or 'west':
        Calib_ang()
        
    else:
        left()
           
    #steer the direction until finding the new line

def straight_call():
    #straight method implemented with orientation global var.
    global orientation
    print('straight')
    px.forward(7)
    str8_dur = 0
    end = 0
    innit_angle = None
    if orientation == 0:
        innit_angle = 24
    elif orientation == 1:
        innit_angle = 110
    elif orientation == 2:
        innit_angle = 202
    elif orientation == 3:
        innit_angle = 285
    
    while str8_dur < 260:
        #end = db.child("traxxas").child("end").get()
        #print(end.val())
        #if end.val() == 1:
        #    outHandle()
        
        
        str8_dur +=1      
            
        #euler angle consistency
        if (sensor.euler is not None):
            sensoreuler = sensor.euler[0]
            if sensoreuler is not None:    
                #db.child("traxxas").child("ID5").update({"Orientation":sensoreuler})
                if int(innit_angle - sensoreuler) >= 2:
                    px.set_dir_servo_angle(10)
                elif int(innit_angle - sensoreuler) >= 1:
                    px.set_dir_servo_angle(5)
                elif int(innit_angle - sensoreuler) <= -2:
                    px.set_dir_servo_angle(-10)
                elif int(innit_angle - sensoreuler) <= -1:
                    px.set_dir_servo_angle(-5)
                else:
                    px.set_dir_servo_angle(0)
    outHandle()           

def straight():
    #straight method implemented with orientation global var.
    global orientation
    print('straight')
    px.forward(7)
    str8_dur = 0
    end = 0
    innit_angle = None
    
    
 
    if orientation == 0:
        innit_angle = 24
    elif orientation == 1:
        innit_angle = 110
    elif orientation == 2:
        innit_angle = 202
    elif orientation == 3:
        innit_angle = 285
    
    
    while str8_dur < 40:
        #end = db.child("traxxas").child("end").get()
        #print(end.val())
        #if end.val() == 1:
        #    outHandle()
        
        
        str8_dur +=1      
            
        #euler angle consistency
        if (sensor.euler is not None):
            sensoreuler = sensor.euler[0]
            if sensoreuler is not None:
                #db.child("traxxas").child("ID5").update({"Orientation":sensoreuler})   
                if int(innit_angle - sensoreuler) >= 2:
                    px.set_dir_servo_angle(10)
                elif int(innit_angle - sensoreuler) >= 1:
                    px.set_dir_servo_angle(5)
                elif int(innit_angle - sensoreuler) <= -2:
                    px.set_dir_servo_angle(-10)
                elif int(innit_angle - sensoreuler) <= -1:
                    px.set_dir_servo_angle(-5)
                else:
                    px.set_dir_servo_angle(0)
    outHandle()    
def left():
    global orientation
    if orientation == 0:
        orientation = 3
    else:
        orientation -=1
    if orientation == 0:
        boundmin = 22
        boundmax = 28
        
    elif orientation == 1:
        boundmin = 112
        boundmax = 118
        
    elif orientation == 2:
        boundmin = 202
        boundmax = 208
        
    elif orientation == 3:
        boundmin = 292
        boundmax = 298
        
    print('left')
    steer = -35
    px.set_dir_servo_angle(steer)
    px.forward(7)
    while True:
        if (sensor.euler[0] is not None):
            euler_angle = sensor.euler[0]
            if (euler_angle is not None):
                db.child("traxxas").child("ID5").update({"Orientation":euler_angle})
                if euler_angle >= boundmin and euler_angle <= boundmax:
                    straight()
        #print(euler_angle)
            
        #db.child("pi").update({"euler":euler_angle})
        if GPIO.input(left_wing) == 1 or GPIO.input(right_wing) == 1:
            escapelines()
            px.set_dir_servo_angle(steer)
        
            #db.child("pi").update({"orientation":orientation})
        
        #gm_val_list = px.get_grayscale_data()
        #gm_state = px.get_line_status(gm_val_list)
        #if gm_state != 'stop':
            
def right():
    global orientation
    
    print('right')
    steer = 35
    px.set_dir_servo_angle(steer)
    px.forward(7)
    
    if orientation == 3:
        orientation = 0
    else:
        orientation +=1
    if orientation == 0:
        boundmin = 22
        boundmax = 28

    elif orientation == 1:
        boundmin = 112
        boundmax = 118
        
    elif orientation == 2:
        boundmin = 202
        boundmax = 208
        
    elif orientation == 3:
        boundmin = 292
        boundmax = 298
    while True:
        
        if (sensor.euler[0] is not None):
            euler_angle = sensor.euler[0]
            if (euler_angle is not None):
                db.child("traxxas").child("ID5").update({"Orientation":euler_angle})
                if euler_angle >= boundmin and euler_angle <= boundmax:
            
            #db.child("pi").update({"orientation":orientation})
                    straight()
        #print(euler_angle)
        #db.child("pi").update({"euler":euler_angle})
        if GPIO.input(left_wing) == 1 or GPIO.input(right_wing) == 1:
            escapelines()
            px.set_dir_servo_angle(steer)

        
        
        #gm_val_list = px.get_grayscale_data()
        #gm_state = px.get_line_status(gm_val_list)
        #print(gm_val_list)
        
        ##if gm_val_list[0] or gm_val_list[1] or gm_val_list[2] <= 400:
        #    linefollow()
        #if gm_state != 'stop':
         #   linefollow()
        
def escapelines():
    while GPIO.input(left_wing) != 0:
        px.set_dir_servo_angle(5)
        px.forward(5)
        
        
            
        
    while GPIO.input(right_wing) != 0:
        px.set_dir_servo_angle(-5)
        px.forward(5)
    px.forward(5)   
        
        #time.sleep(0.1)

def take_photo():
    _time = strftime('%Y-%m-%d-%H-%M-%S',localtime(time()))
    name = 'photo_%s'%_time
    path = "/home/pi/Pictures/picar-x/"
    Vilib.take_photo(name, path)
    print('\nphoto save as %s%s.jpg'%(path,name))


def manual_control():
    global direction_index, orientation, direction_array
    orientation = 0
    movement = 0
    speed = 0
    picture = 0
    status = 'stop'
    gear = 0
    oldgear = 0
    direction_index = 0
    sleep(2)  # wait for startup
    #print(manual)

    while True:
#         print("\rstatus: %s , speed: %s    "%(status, speed), end='', flush=True)
        pygame.event.get()
        # Get the value of the left joystick's x-axis
        x_axis = joystick.get_axis(0) #steering axis
        #y_axis = -joystick.get_axis(0)
        lt_throttle = joystick.get_axis(2) #left throttle  axis
        rt_throttle = joystick.get_axis(5) #right throttle acis
        servo_horiz_axis = joystick.get_axis(4)
        servo_verti_axis = joystick.get_axis(3)
        
        # Set the servo's angle based on the x-axis value
        #servo.ChangeDutyCycle(7.5 + x_axis)
        px.set_dir_servo_angle(x_axis*35)
        px.set_camera_servo1_angle(servo_verti_axis*35)
        px.set_camera_servo2_angle(-servo_horiz_axis*35)
        # Get the value of the A button
        b_button = joystick.get_button(0)
        rt_button = joystick.get_button(8)
        lt_button = joystick.get_button(7)
        c_button = joystick.get_button(9)
        # If the A button is pressed, increase the motor's speed
        if lt_throttle > -.99:
            movement -= ((lt_throttle+1))
            #print('lt')
            gear = 2
        elif rt_throttle > -.99:
            movement += ((rt_throttle+1))
            #print('rt')
            gear = 1
     
        else:
            gear = 0
            #if movement < -12:
            #    movement += .9
            #elif movement > 12:
            #    movement -= .9 #coast value
            #elif movement <= -.4:
            #    movement += .4
            #elif movement >= .4:
            #    movement -= .4
            #if movement <= .4 and movement >= -.4:
            movement = 0
        if movement >= 65:
            movement = 65
        if movement <= -65:
            movement = -65
        if sensor.euler[0] is not None:
            angle_euler = sensor.euler[0]
            if angle_euler is not None:
                db.child("traxxas").child("ID5").update({"Orientation":angle_euler})
                # print(angle_euler)
        if b_button:
            movement = 0
            direction_array = db.child("traxxas").child('path').get().val()
            if direction_array[0] == 'north':
                orientation = 1
            elif direction_array[0] == 'east':
                orientation = 2
            elif direction_array[0] == 'south':
                orientation = 3
            elif direction_array[0] == 'west':
                orientation = 0
            outHandle()
# #         if c_button:
        
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                
                if event.button == 3:
                    outHandle()
                    
                if event.button == 10: 
                    
                    _time = strftime('%Y-%m-%d-%H-%M-%S',localtime(time()))
                    name = 'photo_%s'%_time
                    path = "/home/pi/Pictures/picar-x/"
                    Vilib.take_photo(name, path)
                    print('\nphoto save as %s%s.jpg'%(path,name))
                    gear = 3
           
           
            
    
        px.forward(movement*.5)
        if gear != oldgear:
            
            db.child("pi").update({"movement_state":gear})
        oldgear = gear

def linefollow():
    outHandle()
if __name__=='__main__':
    try:
        while sensor.calibration_status[3] != 3:
            print(sensor.calibrated)
            print(sensor.calibration_status)
        sleep(5)
        outHandle()
        

    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        px.stop()
    except TypeError:
        print("TypeError")
        px.stop()
    except Exception as e:
        print(e)
        print('manual control')
        px.stop()
    finally:
        print('manual control')
        manual_control()



