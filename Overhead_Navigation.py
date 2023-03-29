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
        return path


def clear_path():
    global pathToServer
    canvas.delete("path")
    canvas.delete("end")
    pathToServer = ""
    
def draw_path(): 
    clear_path()
    end = int(input_entry.get())
    path = astar(start, end) 
    end = intersections[end-1]
    new_path = [(y*50+10, x*50+10) for (x, y) in path]
    # Draw the path
    for i in range(len(new_path)-1): 
        x1, y1 = new_path[i]
        x2, y2 = new_path[i+1]
        canvas.create_line(x1, y1, x2, y2, fill="blue", width=2,tags="path")
    # Draw the start and end points 
    canvas.create_oval(start2[0]-5, start2[1]-5, start2[0]+5, start2[1]+5, fill="red")
    end = (end[1] * 50+10, end[0] * 50+10)
    canvas.create_oval(end[0]-5, end[1]-5, end[0]+5, end[1]+5, fill="green", tags="end")
    path.insert(0,beginOrientation)
    
    #picam2.start(show_preview=True)
    
#capturing image, decoding aruco, and updating aruco position on the GUI    
def update():
    i = 0
    j = 0
    while i<1:
        i+=1
        frame = picam2.capture_array()
        image_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        detectorParams = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(gray)
        print(markerCorners, markerIds)
        print(len(markerCorners))
        canvas.delete("circs")
        for j in range(len(markerCorners)):
            print(markerCorners[j][0][0][0],markerCorners[j][0][0][1])
            #put oval on GUI where aruco corner is detected.
            canvas.create_oval(int(markerCorners[j][0][0][0])-5, int(markerCorners[j][0][0][1])-5, int(markerCorners[j][0][0][0])+5, int(markerCorners[j][0][0][1])+5, fill="red", tags="circs")
            #NEXT PUT ID IN TEXT NEXT TO CIRCLE (WOULD BE HELPFUL) CAN INDICATE FRONTIERS BASED ON POSITION / ID
            j+=1
        time.sleep(.05)
   
    root.after(1000, update)        



#start position 
start = 1
thingy = intersections[start-1]
start2 = (thingy[1] * 50+10 , thingy[0] * 50+10 )

# Create the GUI window
picam2 = Picamera2()
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous, "LensPosition": 0.0})
picam2.start_preview(Preview.QTGL)
#picam2.title_fields = ["Overhead Preview"]
config = picam2.create_preview_configuration(main={"size": (1920, 1080)})
picam2.configure(config)
picam2.start()
root = tk.Tk()
root.title("GUI Control") 
root.geometry("1920x1080")

# Create the canvas to draw on 
canvas = tk.Canvas(root, width=1920, height=1080)
canvas.pack()

# Draw the grid 
canvas.create_rectangle(0, 0, 570, 570, fill="light grey")

canvas.create_rectangle(0, 50, 570, 70, fill="grey")
canvas.create_rectangle(0, 200, 570, 220, fill="grey")
canvas.create_rectangle(0, 350, 570, 370, fill="grey")
canvas.create_rectangle(0, 500, 570, 520, fill="grey")

canvas.create_rectangle(50, 0, 70, 570, fill="grey")
canvas.create_rectangle(200, 0, 220, 570, fill="grey")
canvas.create_rectangle(350, 0, 370, 570, fill="grey")
canvas.create_rectangle(500, 0, 520, 570, fill="grey")

canvas.create_line(60, 0, 60, 40, width =2, arrow=LAST)
canvas.create_line(360, 0, 360, 40, width =2, arrow=LAST)
canvas.create_line(210, 570, 210, 530, width =2, arrow=LAST)
canvas.create_line(510, 570, 510, 530, width =2, arrow=LAST)

canvas.create_line(0, 210, 40, 210, width =2, arrow=LAST)
canvas.create_line(0, 510, 40, 510, width =2, arrow=LAST)
canvas.create_line(570, 60, 530, 60, width =2, arrow=LAST)
canvas.create_line(570, 360, 530, 360, width =2, arrow=LAST)


destination_label = tk.Label(root, text="Destination:", font=("Arial", 15))
destination_label.place(x=500, y=600)

input_entry = tk.Entry(root, width=30)
input_entry.place(x=620, y=600)

destination_button = tk.Button(root, text="Submit", command=draw_path) 
destination_button.place(x=620, y=650)

clear_button = tk.Button(root, text="Clear", command=clear_path)
clear_button.place(x=700, y=650)

# Draw the start point
canvas.create_oval(start2[0]-5, start2[1]-5, start2[0]+5, start2[1]+5, fill="red")



update()      
root.mainloop()

