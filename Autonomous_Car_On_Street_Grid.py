
'''
    --Autonomous Driving Program--
    
    Platform:
    
    -Sunfounder Picar-X
    
    Implemented Functionalities:
    
    -Google Firebase Descision Making
    
    -Adafruit 9-AXIS Inertail Measurement Unit; Left, Right, Forward Steering Alignment 
    
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

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN)
GPIO.setup(4, GPIO.IN)
GPIO.setup(22, GPIO.IN)
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
wheel_enc = 22
#direction_array = ['south', 'left', 'straight' ,'right','left','straight' ,'end']
direction_array = db.child("overhead").child('path').get().val()

if direction_array[0] == 'north':
    orientation = 1
elif direction_array[0] == 'east':
    orientation = 2
elif direction_array[0] == 'south':
    orientation = 3
elif direction_array[0] == 'west':
    orientation = 0

pulse_count = 0
total_distance = 0

# Interrupt handler for pulse events
def pulse_handler(channel):
    global pulse_count
    pulse_count += 1
    distance_travel = int(pulse_count * .72)
    #db.child("ID5").child("data").update({"distance":distance_travel})

# Set up interrupt on rising edge of pulse signal
GPIO.add_event_detect(wheel_enc, GPIO.RISING, callback=pulse_handler)


    
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
                    initial_straight()
                    
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
    prev_subsquare = 0
    global last_state, current_state, direction_index, orientation
    #read the firebase descision
    #db.child("Traxxas").child("ID5").set(orientation)
    
    subsquare = int(db.child("ID6").child("location").child("subsquare").get().val())
    print(subsquare)
    if  subsquare >= 2 and subsquare <= 15 and subsquare != 4 and subsquare != 13:
        traffic_read = 1
        
    else:
        traffic_read = 0
        
    immediate_direction = direction_array[direction_index]
    direction_index += 1
    
    #if firebase gives 0 it turns left

    if immediate_direction == 'left':
        left()
    #firebase is 1 go straight
    elif immediate_direction == 'straight':
        if traffic_read == 0:
            straight_call()
        else:
            straight_read(prev_subsquare)
    elif immediate_direction == 'right':
        right()
    elif immediate_direction == 'end':
        end()
    elif immediate_direction == 'south' or 'north' or 'east' or 'west':
        #Calib_ang()
        initial_straight()
    else:
        left()
    
    prev_subsquare = subsquare
           
    #steer the direction until finding the new line
def orientate():
    global orientation
    if orientation == 0:
        innit_angle = 30
    elif orientation == 1:
        innit_angle = 115
    elif orientation == 2:
        innit_angle = 207
    elif orientation == 3:
        innit_angle = 301
    return innit_angle

def straight_read(subsquare):
    #straight method implemented with orientation global var.
    global orientation, pulse_count
    print('straight read')
    px.forward(5)
    initial_pulse = pulse_count
    str8_dur = 0
    end = 0
    time_stopped = 0
    innit_angle = None
    innit_angle = orientate()
    if orientation == 0 or orientation == 2:  
        travel_ticks = 46
    else:
        travel_ticks = 42
    
    while (pulse_count - initial_pulse) < travel_ticks:
        #LIGHT NUMBER 11#
        if subsquare == 12 or subsquare == 7:
            trafficlight_str = "11"
            LED_STATE = db.child("Lights").child(trafficlight_str).get().val()
            if orientation == 0:
                if LED_STATE == 'south':
                    print('stopped south #11')
                    px.stop()
                    while True:
                        LED_STATE = db.child("Lights").child(trafficlight_str).get().val()
                        if LED_STATE == 'south':
                            time_stopped+=1
                        else:
                            print(time_stopped)
                            time_stopped = 0
                            px.forward(5)
                            break
            elif orientation == 3: 
                if LED_STATE == 'west':
                    print('stopped west #11')
                    px.stop()
                    while True:
                        LED_STATE = db.child("Lights").child(trafficlight_str).get().val()
                        if LED_STATE == 'west':
                            time_stopped+=1
                        else:
                            print(time_stopped)
                            time_stopped = 0
                            px.forward(5)
                            break
            
        #LIGHT NUMBER 10#
        elif subsquare == 14 or subsquare == 11:
            trafficlight_str = "10"
            LED_STATE = db.child("Lights").child(trafficlight_str).get().val()
            if orientation == 2:
                if LED_STATE == 'north':
                    print('stopped north #10')
                    px.stop()
                    while True:
                        LED_STATE = db.child("Lights").child(trafficlight_str).get().val()
                        if LED_STATE == 'north':
                            time_stopped+=1
                        else:
                            print(time_stopped)
                            time_stopped = 0
                            px.forward(5)
                            break
            elif orientation == 3: 
                if LED_STATE == 'west':
                    print('stopped west #10')
                    px.stop()
                    while True:
                        LED_STATE = db.child("Lights").child(trafficlight_str).get().val()
                        if LED_STATE == 'west':
                            time_stopped+=1
                        else:
                            print(time_stopped)
                            time_stopped = 0
                            px.forward(5)
                            break
                    
        #LIGHT NUMBER 7#
        elif subsquare == 6 or subsquare == 3:
            trafficlight_str = "7"
            LED_STATE = db.child("Lights").child(trafficlight_str).get().val()
            if orientation == 1:
                if LED_STATE == 'east':
                    px.stop()
                    print('stopped east #7')
                    while True:
                        LED_STATE = db.child("Lights").child(trafficlight_str).get().val()
                        if LED_STATE == 'east':
                            time_stopped+=1
                        else:
                            print(time_stopped)
                            time_stopped = 0
                            px.forward(5)
                            break
            elif orientation == 0: 
                if LED_STATE == 'south':
                    px.stop()
                    print('stopped south #7')
                    while True:
                        LED_STATE = db.child("Lights").child(trafficlight_str).get().val()
                        if LED_STATE == 'south':
                            time_stopped+=1
                        else:
                            print(time_stopped)
                            time_stopped = 0
                            px.forward(5)
                            break
            
        #LIGHT NUMBER 6#
        elif subsquare == 5 or subsquare == 10:
            trafficlight_str = "6"
        LED_STATE = db.child("Lights").child(trafficlight_str).get().val()
        if orientation == 2:
            if LED_STATE == 'north':
                print('stopped north #6')
                px.stop()
                while True:
                    LED_STATE = db.child("Lights").child(trafficlight_str).get().val()
                    if LED_STATE == 'north':
                        time_stopped+=1
                    else:
                        print(time_stopped)
                        time_stopped = 0
                        px.forward(5)
                        break
        elif orientation == 1: 
            if LED_STATE == 'east':
                print('stopped east #6')
                px.stop()
                while True:
                    LED_STATE = db.child("Lights").child(trafficlight_str).get().val()
                    if LED_STATE == 'east':
                        time_stopped+=1
                    else:
                        print(time_stopped)
                        time_stopped = 0
                        px.forward(5)
                        break
        

        subsquare = db.child("ID6").child("location").child("subsquare").get().val()
   
        if GPIO.input(left_wing) == 1 or GPIO.input(right_wing) == 1:
            escapelines_fwd()    
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
def straight_call():
    #straight method implemented with orientation global var.
    global orientation, pulse_count
    print('straight call')
    px.forward(5)
    initial_pulse = pulse_count
    str8_dur = 0
    end = 0
    innit_angle = None
    innit_angle = orientate()
    if orientation == 0 or orientation == 2:    
        travel_ticks = 47
    else:
        travel_ticks = 42
    while (pulse_count - initial_pulse) < travel_ticks:
   
        if GPIO.input(left_wing) == 1 or GPIO.input(right_wing) == 1:
            escapelines_fwd()    
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
    global orientation, pulse_count
    print('straight')
    px.forward(5)
    initial_pulse = pulse_count
    str8_dur = 0
    end = 0
    innit_angle = None
    innit_angle = orientate()
    
    if orientation == 0 or orientation == 2:  
        travel_ticks = 8
    else:
        travel_ticks = 6
    while (pulse_count - initial_pulse) <= travel_ticks :
        #end = db.child("traxxas").child("end").get()
        #print(end.val())
        #if end.val() == 1:
        #    outHandle()
        
        
             
        if GPIO.input(left_wing) == 1 or GPIO.input(right_wing) == 1:
            escapelines_fwd()
                
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
    
def initial_straight():
    #straight method implemented with orientation global var.
    global orientation, pulse_count
    print('initial straight')
    px.forward(5)
    initial_pulse = pulse_count
    str8_dur = 0
    end = 0
    innit_angle = None
    innit_angle = orientate()
    if orientation == 0 or orientation == 2:  
        travel_ticks = 35
    else:
        travel_ticks = 30
    while (pulse_count - initial_pulse) <= travel_ticks :
        #end = db.child("traxxas").child("end").get()
        #print(end.val())
        #if end.val() == 1:
        #    outHandle()
             
        if GPIO.input(left_wing) == 1 or GPIO.input(right_wing) == 1:
            escapelines_fwd()
                
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

def end():
    #straight method implemented with orientation global var.
    global orientation, pulse_count
    print('end')
    px.forward(5)
    initial_pulse = pulse_count
    str8_dur = 0
    end = 0
    innit_angle = None
    innit_angle = orientate()
    if orientation == 0 or orientation == 2:  
        travel_ticks = 25
    else:
        travel_ticks = 20
    while (pulse_count - initial_pulse) <= 20 :
  
        if GPIO.input(left_wing) == 1 or GPIO.input(right_wing) == 1:
            escapelines()
                
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
    px.stop()
    manual_control()
    
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
    px.forward(5)
    while True:
        if (sensor.euler[0] is not None):
            euler_angle = sensor.euler[0]
            if (euler_angle is not None):
                #db.child("traxxas").child("ID5").update({"Orientation":euler_angle})
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
    px.forward(5)
    
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
#                 db.child("traxxas").child("ID5").update({"Orientation":euler_angle})
                if euler_angle >= boundmin and euler_angle <= boundmax:
            
            #db.child("pi").update({"orientation":orientation})
                    straight()
        #print(euler_angle)
        #db.child("pi").update({"euler":euler_angle})
        if GPIO.input(left_wing) == 1 or GPIO.input(right_wing) == 1:
            escapelines()
            px.set_dir_servo_angle(steer)

def escapelines():
    while GPIO.input(left_wing) != 0:
        px.set_dir_servo_angle(5)
        px.forward(5)

    while GPIO.input(right_wing) != 0:
        px.set_dir_servo_angle(-5)
        px.forward(5)
    px.forward(5)
def escapelines_fwd():
    while GPIO.input(left_wing) != 0:
        px.set_dir_servo_angle(20)
        

    while GPIO.input(right_wing) != 0:
        px.set_dir_servo_angle(-20)
        
    px.forward(5)   

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

    while True:
        if (sensor.euler[0] is not None):
            euler_angle = sensor.euler[0]
            if (euler_angle is not None):
                 db.child("ID6").child("data").update({"Orientation":euler_angle})
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
        #if sensor.euler[0] is not None:
            #angle_euler = sensor.euler[0]
            #if angle_euler is not None:
                #db.child("traxxas").child("ID5").update({"Orientation":angle_euler})
                # print(angle_euler)
        if b_button:
            movement = 0
            direction_array = db.child("overhead").child('path').get().val()
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
        #if gear != oldgear:
            #for taillights
            #db.child("pi").update({"movement_state":gear})
            
        oldgear = gear

if __name__=='__main__':
    try:
        while sensor.calibration_status[3] != 3:
            print(sensor.calibrated)
            print(sensor.calibration_status)
        sleep(5)
        manual_control()
        

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




