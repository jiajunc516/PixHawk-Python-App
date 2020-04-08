'''
FILE:        main.py

AUTHOR:      Jiajun Chang

PROJECT:     EECS 159 Senior Design
             Project Testla: Water Quality Detection Boat

DESCRIPTION: This file builds the command interface, using pymavlink module,
             to receive and send message to autopilot PIXHAWK.
'''

from pymavlink import mavutil
from pymavlink import mavwp
from math import sqrt
import os
import sys

# gcc draft.c -o DRAFT -lwiringPi -lwiringPiDev -lm

def mav_connect():
''' For connection string
    Windows: 'com4'
    Rspberry Pi: '/dev/ttyACM0' (set baud=115200)
'''
    the_connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_system))
    return the_connection

def get_location(the_connection):
'''
    Return the location (lat, lng, alt) message
'''
    location = the_connection.location()
    #print_gps(location)
    return location

def print_gps(location):
'''
    Print (lat, lng, alt) in location
'''
    print("Lat: {:10.7f} Lon: {:10.7f} Alt: {:10.7f}".format(location.lat, location.lng, location.alt))

def read_input(filename):
'''
    Read the file that contains the input of waypoints,
    and return the waypoints as a list
'''
    wp_list = []
    f = open(filename, "r")
    for line in f.readlines():
        latlng = line.split()
        #print("Lat: {} Lon: {}".format(latlng[0], latlng[1]))
        wp_list.append(latlng)
    f.close()
    return wp_list

def add_waypoint(wp, lat, lng, alt):
'''
    Add the waypoint to the mavwp class
'''
    wp.add_latlonalt(lat,lng,alt)

def save_waypoint(wp):
'''
    Save the waypoints information as a log file, which can be loaded
    on any Ground Station Control such as Mission Planner.
'''
    wp.save("demo_log.txt")

def set_home_location(wp, location):
'''
    Add the home waypoint to the mavwp class
'''
    wp.clear()
    add_waypoint(wp, location.lat, location.lng, location.alt)

def return_home(master, wp, location):
'''
    Move to the home base
    Note: Current have a bug that the boat does not response to this action
'''
    print("Return To Base")
    add_waypoint(wp, location.lat, location.lng, location.alt)
    target_lat = float(location.lat)*10000000
    target_lng = float(location.lng)*10000000
    on_the_way = True
    while on_the_way:
        loc = the_connection.location()
        print_gps(location)
        my_lat = float(loc.lat)*10000000
        my_lng = float(loc.lng)*10000000
        if reach_target(my_lat, my_lng, target_lat, target_lng):
            on_the_way = False

def add_mission(wp, latlng, alt):
'''
    Intermediate call for add_waypoint
'''
    add_waypoint(wp, latlng[0], latlng[1], alt)

def reach_target(mlat, mlng, tlat, tlng):
'''
    Return true if the boat's current location is near the target
    Note: Due to the inaccuracy of the GPS, this function needs an
          error (500 in this case)
'''
    if (abs(tlat-mlat)<500) and (abs(tlng-mlng)<500):
        return True
    return False

'''
APPROACH:
    Once reach the target loaction, this program will write a command in "wfile".
    Another C program keeps reading "wfile" and trigger the PH sensor once it
    reads a command. After PH data is saved in log file, the C program writes
    a command in "rfile". This program will move on when it reads a command in
    "rfile".
'''
def clean_file(rfile, wfile):
'''
    Clean "rfile" and "wfile" before exiting the program
'''
    rf = open(rfile, "w")
    rf.write("")
    rf.close()
    wf = open(wfile, "w")
    wf.write("100")
    wf.close()

def disarm(master):
'''
    Disarm the autopilot
'''
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)

def arm(master):
'''
    Arm the autopilot
'''
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)
    master.set_mode_auto()

def execute():
'''
    Designed to execute C program using linux command
    Note: Failed due to import error, which has to do with Raspberry Pi
'''
    os.system('gcc draft.c -o DRAFT -lwiringPi -lwiringPiDev -lm')
    os.system('./DRAFT')

def run_mission(the_connection, wp, wp_list, alt, cnt, rfile, wfile):
'''
    Execute the entire mission
'''
    i = 0
    while i < cnt:
        latlng = wp_list[i]
        add_mission(wp, latlng, alt)
        target_lat = float(latlng[0])*10000000
        target_lng = float(latlng[1])*10000000
        on_the_way = True
        arm(the_connection)
        while on_the_way:
            location = the_connection.location()
            print_gps(location)
            my_lat = float(location.lat)*10000000
            my_lng = float(location.lng)*10000000
            if reach_target(my_lat, my_lng, target_lat, target_lng):
                print("Reached location. Reading Data...")
                f = open(wfile, "w")
                f.write(str(i))
                f.close()
                reading = True
                while reading:
                    rf = open(rfile, "r")
                    if rf.read() != "":
                        reading = False
                        rf.close()
                        print("Data Sent")
                        clean_file(rfile, wfile)
                on_the_way = False
        i += 1
    print("Mission Complete")

'''
Route:
    To find shortest path to visite all waypoints.
    Give two options:
        1. Brute Force, O(n!)
        2. Greedy, O(n^2), 20% longer than optimal solution
'''
class Route:
    def __init__(self, home, points):
        self.origin = self.getOrigin(home)
        self.points = self.getPoints(points)
    
    def getOrigin(self, origin):
        return (float(origin.lat), float(origin.lng))

    def getPoints(self, points):
        res = []
        for i, latlng in enumerate(points):
            res.append((i, float(latlng[0], float(latlng[1]))))
        return res
    
    def distance(self, origin, point):
        return sqrt((origin[0]-point[0])**2 + (origin[1]-point[1])**2)
    
    def bruteForce(self):
        mind, minpath = self.bruteForceHelp(self.origin, self.points, [], 0)
        return minpath
    
    def bruteForceHelp(self, start, points, path, sumd):
        if len(points) == 1:
            sumd += self.distance(start, (points[0][1:]))
            path.append((points[0][1:]))
            return sumd, path
        mind, minpath = sys.maxsize, []
        for i in range(len(points)):
            path.append(points[i][1:])
            d = self.distance(start, points[i][1:])
            sumd += self.distance(start, points[i][1:])
            resd, respath = self.bruteForceHelp(points[i][1:], points[:i] + points[i+1:], path, sumd)
            if mind > resd:
                mind = resd
                minpath = respath
            path.pop()
            sumd -= d
        return mind, minpath

    def greedy(self):
        res = []
        i, n = 0, len(self.points)
        start = self.origin
        while i < n - 1:
            mind = sys.maxsize
            k = i
            for j in range(i, n):
                d = self.distance(start, self.points[j])
                if mind > d:
                    mind = d
                    k = self.points[j][0]
            if k != i:
                self.points[i], self.points[k] = self.points[k], self.points[i]
            res.append((self.points[k][1:]))
            i += 1
            start = self.points[i]
        res.append((self.points[-1][1:]))
        return res

def run_os(filename):
'''
    Execute the program
'''
    print("Connecting To Pixhawk...")
    the_connection = mav_connect()
    disarm(the_connection)
    print("Success. Now Set Up Mission")
    home_location = get_location(the_connection)
    print_gps(home_location)
    altitude = home_location.alt
    wp = mavwp.MAVWPLoader()
    set_home_location(wp, home_location)
    print("Home Base Set Up")
    print("Read Target GPS Point...")
    wp_list = read_input(filename)
    wp_cnt = len(wp_list)
    print("Received")
    #save_waypoint(wp)

    ##Test Shortest Path To Travel To All Points
    # route = Route(home_location, wp_list)
    # wp_list = route.bruteForce()
    # wp_list = route.greedy()
    
    # readfile:  read from pi if data is sent
    # writefile: write to the file that pi reads to know whether read data or not
    print("Start Mission")
    readfile = "datasent.txt"
    writefile = "senddata.txt"
    run_mission(the_connection, wp, wp_list, altitude, wp_cnt, readfile, writefile)
    return_home(the_connection, wp, home_location)
    disarm(the_connection)
    wp.clear()
    return the_connection

if __name__ == '__main__':
    try:
        gpsfile = "test.txt"
        run_os(gpsfile)
        #execute()
    except:
        master = mav_connect()
        print("Mission abort")
        wp = mavwp.MAVWPLoader()
        wp.clear()
        clean_file("datasent.txt", "senddata.txt")
        disarm(master)
