import tkinter as tk
from tkinter import *
import networkx as nx
import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
from libcamera import controls
from picamera2 import MappedArray, Picamera2, Preview
import time
import pyrebase
config = {
    "apiKey" : "76o1qRJhT5mxuUsDVJGGB1yQvA1llL7HTEKU0Z5m",
    "authDomain" : "seniordesignproj-cef3d.firebaseapp.com",
    "databaseURL" : "https://seniordesignproj-cef3d-default-rtdb.firebaseio.com",
    "storageBucket" : "seniordesignproj-cef3d.appspot.com"
    }



image_width = 1520
image_height = 1296



firebase = pyrebase.initialize_app(config)
db = firebase.database()



intersections = [
    (1,1), (1,4), (1,7), (1,10),
    (4,1), (4,4), (4,7), (4,10),
    (7,1), (7,4), (7,7), (7,10),
    (10,1), (10,4), (10,7), (10,10)
]
orientation = {}
north = (3,0)
south = (-3,0)
west = (0,3)
east = (0,-3)
orientation[east] = "east"
orientation[west] = "west"
orientation[south] = "south"
orientation[north] = "north"
beginOrientation = ""
pathToServer = []

direction = {}
northnorth = ("north", "north")
northeast = ("north" , "east")
northwest = ("north" , "west")
southsouth = ("south", "south")
southeast = ("south" , "east")
southwest = ("south" , "west")
easteast = ("east" , "east")
eastnorth = ("east" , "north")
eastsouth = ("east" , "south")
westwest = ("west" , "west")
westnorth = ("west" , "north")
westsouth = ("west", "south")

direction[northnorth] = "straight"
direction[northeast] = "right"
direction[northwest] = "left"
direction[southsouth] = "straight"
direction[southeast] = "left"
direction[southwest] = "right"
direction[easteast] = "straight"
direction[eastnorth] = "left"
direction[eastsouth] = "right"
direction[westwest] = "straight"
direction[westnorth] = "right"
direction[westsouth] = "left"



G = nx.DiGraph()
G.add_edges_from([
    (1,1), (1,5), (2,2), (2,1), (3,3), (3,2), (3,7), (4,4), (4,3), (5,5), (5,6), (5,9), (6,6), (6,7), (6,2), (7,7),
    (7,8), (7,11), (8,8), (8,4), (9,9), (9,13), (10,10), (10,6), (10,9), (11,11), (11,10), (11,15),
    (12,12), (12,8), (12,11), (13,13), (13,14), (14,14), (14,10), (14,15), (15,15), (15,16), (16,16), (16,12)
])


def get_midpoint(corner1, corner2, corner3, corner4):
    """Return the midpoint of a polygon with four corners."""
    midpoint1_x = (corner1[0] + corner2[0]) / 2
    midpoint1_y = (corner1[1] + corner2[1]) / 2
    midpoint2_x = (corner2[0] + corner3[0]) / 2
    midpoint2_y = (corner2[1] + corner3[1]) / 2
    midpoint3_x = (corner3[0] + corner4[0]) / 2
    midpoint3_y = (corner3[1] + corner4[1]) / 2
    midpoint4_x = (corner4[0] + corner1[0]) / 2
    midpoint4_y = (corner4[1] + corner1[1]) / 2
    midpoint_x = (midpoint1_x + midpoint2_x + midpoint3_x + midpoint4_x) / 4
    midpoint_y = (midpoint1_y + midpoint2_y + midpoint3_y + midpoint4_y) / 4
    return midpoint_x, midpoint_y


def get_square(x, y):
    """Return the square number that the point (x, y) belongs to."""
    row = y // 324  # each row is 1296 / 4 = 324 pixels high
    col = x // 380  # each column is 1520 / 4 = 380 pixels wide
    square_number = row * 4 + col + 1
    if square_number > 16:
        return None
    else:
        return square_number
    

def astar(start, end):

        global beginOrientation
        global pathToServer
        H = nx.shortest_path(G, source=(start), target=(end))

        path = []
        pp = []
        for x in H:
            path.append(intersections[x-1])

        beginOrientation = orientation[path[0][0]-path[1][0],path[0][1]-path[1][1]]
        rng = range(1,(len(path) -1))
        currOrientation = beginOrientation
        pp.append(beginOrientation)
        for x in rng:
            temp = orientation[path[x][0]-path[x+1][0],path[x][1]-path[x+1][1]]
            pp.append(direction[(currOrientation, temp)])
            currOrientation = temp
        pp += ['end']
        pathToServer = pp
        print(pathToServer)
        
        db.child("traxxas").set({"path":pathToServer})
        return path


def clear_path():
    global pathToServer
    canvas.delete("path")
    canvas.delete("end")
    pathToServer = ""
    
def draw_path():
    #must grab intersection position from xy
    
    clear_path()
    end = int(input_entry.get())
    path = astar(start, end) 
    end = intersections[end-1]
    new_path = [(y*130+40, x*110+10) for (x, y) in path]
    # Draw the path
    for i in range(len(new_path)-1): 
        x1, y1 = new_path[i]
        x2, y2 = new_path[i+1]
        canvas.create_line(x1, y1, x2, y2, fill="blue", width=2,tags="path")
    # Draw the start and end points 
    canvas.create_oval(start2[0]-5, start2[1]-5, start2[0]+5, start2[1]+5, fill="red")
    end = (end[1] * 130+40, end[0] * 110+10)
    canvas.create_oval(end[0]-5, end[1]-5, end[0]+5, end[1]+5, fill="green", tags="end")
    path.insert(0,beginOrientation)
    
    #picam2.start(show_preview=True)
    
    
def update():
    i = 0
    j = 0
    while i<1:
        i+=1
        frame = picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        detectorParams = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(gray)

        canvas.delete("circs")
        for j in range(len(markerCorners)):
            car_ID = int(markerIds[j])
            
            corner1 = markerCorners[j][0][0]
            corner2 = markerCorners[j][0][1]
            corner3 = markerCorners[j][0][2]
            corner4 = markerCorners[j][0][3]
            midpoint = get_midpoint(corner1, corner2, corner3, corner4)

            #get intersection number postion
            square_number = get_square(x, y)


            #generate string with car ID
            car_str = "ID" + str(car_ID)

            #send firebase closest intersection relative to the car(s)
            db.child("traxxas").update({"end":square_number})
            
            #send firebase X,Y positioning.
            db.child("traxxas").child(car_str).update({"X":midpoint[0]})
            db.child("traxxas").child(car_str).update({"Y":midpoint[1]})
            

            #canvas.create_oval(x_pos-5, y_pos-5, x_pos+5, y_pos + 5, fill="red", tags="circs")
            
            #label cars unique id next to its polygon
            canvas.create_text(x_pos + 10 ,y_pos + 10, text = str(car_ID), fill="black", tags ="circs", width = 30)

            #Create a polygon representing the car
            canvas.create_polygon(x_pos, y_pos, mid_x_pos,mid_y_pos,x_pos1,y_pos1,front_x_pos, front_y_pos, fill="blue",tags = "circs")
            j+=1

        
   
    root.after(10, update)        



#start position 
start = 1
thingy = intersections[start-1]
start2 = (thingy[1] * 160+10 , thingy[0] * 100+10 )

# Create the GUI window

picam2 = Picamera2()
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
#picam2.start_preview(Preview.QTGL)
#picam2.title_fields = ["Hey"]
config = picam2.create_preview_configuration(main={"size": (image_width, image_height)})
picam2.configure(config)
picam2.start()
root = tk.Tk()
root.title("GUI Control") 
root.geometry("1920x1300")

# Create the canvas to draw on 
canvas = tk.Canvas(root, width=1920, height=1300)
canvas.pack()

# Draw the grid

# canvas.create_rectangle(0, 0, 1800, 1280, fill="light grey")

canvas.create_rectangle(110, 30, 1410, 160, fill="grey")
canvas.create_rectangle(110, 370, 1410, 510, fill="grey")
canvas.create_rectangle(110, 680, 1410, 830, fill="grey")
canvas.create_rectangle(110, 1040, 1410, 1190, fill="grey")

canvas.create_rectangle(110, 30, 270, 1190, fill="grey")
canvas.create_rectangle(485, 30, 640, 1190, fill="grey")
canvas.create_rectangle(850, 30, 1010, 1190, fill="grey")
canvas.create_rectangle(1220, 30, 1410, 1190, fill="grey")
#canvas.create_rectangle(0, 200, 570, 220, fill="grey")
#canvas.create_rectangle(0, 350, 570, 370, fill="grey")
#canvas.create_rectangle(0, 500, 570, 520, fill="grey")

#canvas.create_rectangle(50, 0, 70, 570, fill="grey")
#canvas.create_rectangle(200, 0, 220, 570, fill="grey")
#canvas.create_rectangle(350, 0, 370, 570, fill="grey")
#canvas.create_rectangle(500, 0, 520, 570, fill="grey")

#canvas.create_line(60, 0, 60, 40, width =2, arrow=LAST)
#canvas.create_line(360, 0, 360, 40, width =2, arrow=LAST)
#canvas.create_line(210, 570, 210, 530, width =2, arrow=LAST)
#canvas.create_line(510, 570, 510, 530, width =2, arrow=LAST)

#canvas.create_line(0, 210, 40, 210, width =2, arrow=LAST)
#canvas.create_line(0, 510, 40, 510, width =2, arrow=LAST)
#canvas.create_line(570, 60, 530, 60, width =2, arrow=LAST)
#canvas.create_line(570, 360, 530, 360, width =2, arrow=LAST)


destination_label = tk.Label(root, text="Destination:", font=("Arial", 15))
destination_label.place(x=1720, y=60)

input_entry = tk.Entry(root, width=15)
input_entry.place(x=1720, y=100)

destination_button = tk.Button(root, text="Submit", command=draw_path) 
destination_button.place(x=1720, y=150)

clear_button = tk.Button(root, text="Clear", command=clear_path)
clear_button.place(x=1800, y=150)

# Draw the start point
canvas.create_oval(start2[0]-5, start2[1]-5, start2[0]+5, start2[1]+5, fill="red")



update()      
root.mainloop()





# Define the x and y ranges for each square

# Function to determine which square a point (x, y) belongs to

