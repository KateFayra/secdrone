'''
    Modified from the following files:
    mjpeg.py
    Author: Igor Maculan - n3wtron@gmail.com
    A Simple mjpg stream http server

    vehicle_state.py:
    Demonstrates how to get and set vehicle state and parameter information,
    and how to observe vehicle attribute (state) changes.

    Full documentation is provided at http://python.dronekit.io/examples/vehicle_state.html (Link seems to now be dead, repo is below)
    https://github.com/dronekit/dronekit-python
    
    Both files mentioned above are licensed under the Apache 2 license, which is compatable with this project's license which is GNU GPLv3.
    For more information see: https://www.apache.org/licenses/GPL-compatibility.html
'''

from dronekit import connect, VehicleMode, LocationGlobalRelative
import signal
import time
import cv2
#import Image
from PIL import Image
from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
import time
import sys
from SocketServer import ThreadingMixIn
import threading
from threading import Thread
capture = None
captureRate = 6
viewRate = 6
sftpSendCount = 3
import Queue
import json
import argparse
import os.path
import os
import time
from subprocess import Popen, PIPE
from datetime import datetime
import ivPID.PID
import argparse
import cv2
import pysftp

# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

global capture
capture = cv2.VideoCapture(0)
capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 360)
capture.set(cv2.cv.CV_CAP_PROP_SATURATION,0.2)
global img
global server
armingInProgress = False
modeChangeInProgress = False
destQueue = Queue.Queue()
configAlt = 5 #TODO: Configuration file/options
configGroundspeed = 0.1 #TODO: Configuration file/options
configHeading = 180 #TODO: Configuration file/options

EMERGENCY_LANDING_ENABLED = True
MIN_BATTERY_VOLTAGE = 10
NON_BATT_WARN_AUDIO = True
CONNECT_VEHICLE = True
USE_CAMERA = True
USE_AUDIO = False
LOGGING_ENABLED = False
USE_MJPEG_STREAM = True
CONFIG_TIME_AT_NODE = 3
CONFIG_SEND_IMAGES = False
LAND_WHEN_BATTERY_LOW = True

configCircuit = True
configAltitudeOverride = False
json_routes_prefix = "routes/route"
json_routes_suffix = ".json"
json_routes_index = 0
json_routes_names_list = []

STABLE_YAW = 1525
LEFT_YAW = 1300
RIGHT_YAW = 1750

global flightLogFile
now = time.strftime("%c")
flightLogFile = open("logs/flight-" + now.replace(" ", "-").replace("--", "-") + ".log", "a")
global currentDestinationNode
currentDestinationNode = None
global currentDestinationNodeReached
currentDestinationNodeReached = False
global sftpSendCounter
sftpSendCounter = 0
global timeAtNode
timeAtNode = 0

currentNodeHeading = 0
currentNodeStable = False
currentNodeStableCount = 0
routeThreadObj = None
systemMonitorThreadObj = None
captureThreadObj = None
imageProcessingThreadObj = None
emergencyLandingThreadObj = None

running_procs = []
currentJpg = None
currentJpgLength = 0

# def preexec(): # Don't forward signals.
#     os.setpgrp()


def speak(s, override=False):
    if USE_AUDIO:
        if NON_BATT_WARN_AUDIO or override:
            running_procs.append(Popen(["/usr/bin/espeak", "\"" + s + "\"", "-ven-us+f4", "-s200", "-w", "tmp.wav"], close_fds=True))
            running_procs.append(Popen(["/usr/bin/omxplayer", "tmp.wav", "--vol", "1000"], close_fds=True))

def writeToLog(s):
    if LOGGING_ENABLED:
        flightLogFile.write(s)

class DestNode:
    def __init__(self, lat, lng, alt):
        self.lat = lat
        self.lng = lng
        self.alt = alt
        self.hdg = configHeading

    def setHeading(self, hdg):
        self.hdg = hdg


def isclose(a, b, rel_tol=1e-09, abs_tol=0.0): #Backported from python 3: http://stackoverflow.com/a/33024979
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

# def condition_yaw(heading, relative=False):
#     if relative:
#         is_relative=1 #yaw relative to direction of travel
#     else:
#         is_relative=0 #yaw is an absolute angle
#     # create the CONDITION_YAW command using command_long_encode()
#     msg = vehicle.message_factory.command_long_encode(
#             0, 0,    # target system, target component
#             mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
#             0, #confirmation
#             heading,    # param 1, yaw in degrees
#             0,          # param 2, yaw speed deg/s
#             1,          # param 3, direction -1 ccw, 1 cw
#             is_relative, # param 4, relative offset 1, absolute angle 0
#             0, 0, 0)    # param 5 ~ 7 not used
#     # send command to vehicle
#     vehicle.send_mavlink(msg)

def positiveDiff(diff):
    if diff >= 0:
        return diff
    else:
        return diff + 360


class RouteThread(Thread):
    def __init__(self):
        ''' Constructor. '''

        Thread.__init__(self)

    def run(self):
        global currentDestinationNode
        global currentDestinationNodeReached
        global timeAtNode

        while True:
            writeToLog("RouteThread(): Running!\n")
            if currentDestinationNodeReached:
                writeToLog("RouteThread(): Node reached!\n")

                heading = vehicle.heading
                rightDiff = positiveDiff(configHeading - heading)
                leftDiff = positiveDiff(heading - configHeading)
                writeToLog("heading: " + str(heading) + "\n")
                writeToLog("rightDiff: " + str(rightDiff) + "\n")
                writeToLog("leftDiff: " + str(-leftDiff) + "\n")
                global currentNodeStable
                global currentNodeStableCount
                global currentNodeHeading
                if isclose(heading, configHeading, abs_tol=5):
                    if currentNodeStableCount < 30:
                        currentNodeStableCount = currentNodeStableCount+1
                    elif currentNodeStable is False:
                        currentNodeStable = True
                        timeAtNode = int(time.time()) #start unix timestamp
                        speak("Holding stable at node; at heading " + str(configHeading) + " degrees.")
                        currentNodeHeading = configHeading
                else:
                    currentNodeStableCount = 0
                    currentNodeStable = False


                if rightDiff < leftDiff:
                    pid.update(-rightDiff)
                else:
                    pid.update(leftDiff)

                output = pid.output
                writeToLog("output:" + str(output) + "\n")

                overridePWM = STABLE_YAW + (output*0.01)*STABLE_YAW
                if overridePWM < LEFT_YAW:
                    overridePWM = LEFT_YAW
                elif overridePWM > RIGHT_YAW:
                    overridePWM = RIGHT_YAW

                writeToLog("overridePWM: " + str(overridePWM) + "\n")
                vehicle.channels.overrides['4'] = overridePWM

                if CONFIG_TIME_AT_NODE != 0 and timeAtNode != 0 and int(time.time()) > timeAtNode + CONFIG_TIME_AT_NODE:
                    guidedNextNode()


            elif currentDestinationNode is not None:
                writeToLog("RouteThread(): Global Location (relative altitude): %s\n" % vehicle.location.global_relative_frame)
                if configAltitudeOverride:
                    currentAlt = configAlt
                else:
                    currentAlt = currentDestinationNode.alt
                if isclose(currentDestinationNode.lat, vehicle.location.global_relative_frame.lat, abs_tol=0.00001) and isclose(currentDestinationNode.lng, vehicle.location.global_relative_frame.lon, abs_tol=0.00001) and isclose(currentAlt, vehicle.location.global_relative_frame.alt, abs_tol=0.5):
                #if True:
                    writeToLog("RouteThread(): Node reached, setting to True!\n")
                    speak("Node reached, orienting towards heading " + str(configHeading) + " degrees.")
                    currentDestinationNodeReached = True
                    global currentNodeStable
                    currentNodeStable = False
                    global currentNodeStableCount
                    currentNodeStableCount = 0
                    pid = ivPID.PID.PID(0.2, 0.2, 0)
                    pid.SetPoint=0.0
                    pid.setSampleTime(0.1)

            time.sleep(0.1)

class EmergencyLandingThread(Thread):
    def __init__(self):
        ''' Constructor. '''

        Thread.__init__(self)


    def run(self):
        while True:
            if vehicle.channels['3'] <= 900 and vehicle.mode.name != "LAND" and vehicle.mode.name != "LOITER" and vehicle.mode.name != "STABILIZE":
                vehicle.channels.overrides['4'] = None
                global currentDestinationNode
                currentDestinationNode = None
                global currentDestinationNodeReached
                currentDestinationNodeReached = False
                print "\nSet Vehicle.mode=LAND (currently: %s)" % vehicle.mode.name
                vehicle.mode = VehicleMode("LAND")
                speak("Emergency landing activated! Signal from transmitter was lost!")
                time.sleep(3)

            time.sleep(0.1)

class SystemMonitorThread(Thread):
    def __init__(self):
        ''' Constructor. '''
        Thread.__init__(self)

    def run(self):
        while True:
            battery_voltage = vehicle.battery.voltage
            if battery_voltage < MIN_BATTERY_VOLTAGE:
                if LAND_WHEN_BATTERY_LOW and not vehicle.mode.name=='RTL':
                    global modeChangeInProgress
                    modeChangeInProgress = True
                    vehicle.mode = VehicleMode("RTL")
                    while not vehicle.mode.name=='RTL':  #Wait until mode has changed
                        time.sleep(1)

                speak("Battery low! " + str(battery_voltage), override=True)
            time.sleep(5)

class CaptureThread(Thread):
    def __init__(self):
        ''' Constructor. '''

        Thread.__init__(self)

    def run(self):
        while True:
            try:
                rc,img = capture.read()
                if not rc:
                    continue
                imgRGB=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
                jpg = Image.fromarray(imgRGB)
                now = datetime.now().strftime("%H:%M:%S.%f")
                global filename
                filename = ""
                if currentDestinationNodeReached and currentDestinationNode is not None and currentNodeStable:
                    filename = str(currentDestinationNode.lat) + "_" + str(currentDestinationNode.lng) + "_" + now.replace(" ", "-").replace("--", "-") + ".jpg"
                    tmpFile = open("images/" + filename, "w")
                else:
                    loc = vehicle.location.global_frame
                    filename = str(loc.lat) + "_" + str(loc.lon) + "_" + now.replace(" ", "-").replace("--", "-") + ".jpg"
                    tmpFile = open("images/" + filename, "w")
                jpg.save(tmpFile,'JPEG')
                global currentJpg
                global currentJpgLength
                currentJpg = jpg
                currentJpgLength = tmpFile.tell()
                tmpFile.close()
                global sftpSendCounter
                sftpSendCounter = sftpSendCounter + 1
                print sftpSendCounter
                if CONFIG_SEND_IMAGES and sftpSendCounter == sftpSendCount:
                    sftpSendCounter = 0
                    print "SENDING IMAGE."
                    try:
                        with pysftp.Connection('192.168.43.133', username='pi', password='bestPhoneMuchVR#') as sftp:
                            with sftp.cd('pedestrian-detection/images'): # temporarily chdir to public
                                sftp.put("images/" + filename)  # upload file to public/ on remote
                    except:
                        print "Failed to connect to SFTP host."


                time.sleep(1.0/captureRate)
            except KeyboardInterrupt:
                break


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""
    bind_and_activate=False
    allow_reuse_address = True


def handler(signum, frame):
    #Close vehicle object before exiting script
    print "\nSIGINT CLOSING CONNECTION!"
    vehicle.close()
    capture.release()
    # while server.isAlive():
    #     try:
    #         server._Thread__stop()
    #     except:
    #         print "Thread could not be terminated"

    server.socket.close()
    for thr in threading.enumerate():
        while thr.isAlive():
            try:
                thr._Thread__stop()
            except:
                print "Additional thread could not be terminated"

    flightLogFile.close()
    # for proc in running_procs:
    #     retcode = proc.poll()
    #     if retcode is not None: # Process finished.
    #         running_procs.remove(proc)
    #         break
    #     else: # No process is done, wait a bit and check again.
    #         time.sleep(.1)
    #         continue
    print "Completed, exiting"
    sys.exit()

def guidedNextNode():
    global timeAtNode
    timeAtNode = 0
    if destQueue.empty():
        return

    print "\nSet Vehicle.mode=GUIDED (currently: %s)" % vehicle.mode.name
    global modeChangeInProgress
    modeChangeInProgress = True
    vehicle.mode = VehicleMode("GUIDED")
    speak("Engaging autopilot!")
    while not vehicle.mode.name=='GUIDED':  #Wait until mode has changed
        print " Waiting for mode change ..."
        time.sleep(1)


    modeChangeInProgress = False
    node = destQueue.get()
    if configCircuit:
        destQueue.put(node)
    vehicle.channels.overrides['4'] = None
    global currentDestinationNode
    currentDestinationNode = node
    global configHeading
    configHeading = node.hdg
    global currentDestinationNodeReached
    currentDestinationNodeReached = False
    writeToLog("\nHeading to \nlat: %s\n" % str(node.lat))
    writeToLog("lng: %s\n" % str(node.lng))
    print "\nHeading to \nlat: %s\n" % str(node.lat)
    print "lng: %s\n" % str(node.lng)
    if configAltitudeOverride:
        global configAltitudeOverride
        location = LocationGlobalRelative(node.lat, node.lng, configAlt)
        writeToLog("alt: %s\n" % str(configAlt))
        print "alt: %s\n" % str(configAlt)
    else:
        location = LocationGlobalRelative(node.lat, node.lng, node.alt)
        writeToLog("alt: %s\n" % str(node.alt))
        print "alt: %s\n" % str(node.alt)

    global configGroundspeed
    writeToLog("with speed: %s\n" % str(configGroundspeed))
    print "with speed: %s\n" % str(configGroundspeed)
    vehicle.simple_goto(location, groundspeed=configGroundspeed)
    speak("Autopilot engaged!")


class CamHandler(BaseHTTPRequestHandler): # TODO: Refactor to helper methods, refactor HTTP response. Add json exceptions file exceptions, function to create file name
    def do_GET(self):
        if self.path.startswith('/action/speak'):
            ttsString = self.path.split('/')[-1]
            ttsString = ttsString.replace('%20', ' ')
            speak(ttsString)
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write('True')
            return

        elif self.path.startswith('/load/route'):
            routeIndexString = self.path.split('/')[-1]
            if not os.path.isfile(json_routes_prefix + routeIndexString + json_routes_suffix):
                self.send_response(200)
                self.send_header('Content-type','text/html')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write('False')
                speak("Route failed to load!")
                return

            else:
                with destQueue.mutex:
                    destQueue.queue.clear()

                json_data = open(json_routes_prefix + routeIndexString + json_routes_suffix)
                data = json.load(json_data)
                for node in data['nodes']:
                    newNode = DestNode(float(node['lat']), float(node['lng']), float(node['alt']))

                    try:
                        hdg = float(node['hdg'])
                        newNode.setHeading(hdg)
                    except:
                        print "Could not set heading from JSON"

                    destQueue.put(newNode)

                self.send_response(200)
                self.send_header('Content-type','text/html')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write('True')
                speak("Route loaded!")
                return

        elif self.path.startswith('/set/route'):
            name = self.path.split('/')[-1]
            name = name.replace("%20", " ")
            data = {'name': name, 'nodes': []}  # begin building the JSON data object

            for node in list(destQueue.queue):
                new_json_node = {'lat': node.lat, 'lng': node.lng, 'alt': node.alt, 'hdg': node.hdg}
                data['nodes'].append(new_json_node)

            global json_routes_prefix
            global json_routes_suffix
            global json_routes_index
            file = open(json_routes_prefix + str(json_routes_index) + json_routes_suffix, "w")
            file.write(json.dumps(data, sort_keys=True, indent=4, separators=(',', ': ')))
            file.close()
            json_routes_names_list.append(name)
            json_routes_index=json_routes_index+1

            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write('True')
            speak("Route saved!")
            return

        elif self.path.startswith('/toggle/altitudeoverride'):
            global configAltitudeOverride
            configAltitudeOverride = (not configAltitudeOverride)

            print "\n\nconfigAltitudeOverride:"
            print configAltitudeOverride
            print "\n\n"

            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write('True')
            speak("Toggled altitude override to " + str(configAltitudeOverride) + "!")
            return

        elif self.path.startswith('/toggle/circuit'):
            global configCircuit
            configCircuit = (not configCircuit)

            print "\n\nconfigCircuit:"
            print configCircuit
            print "\n\n"

            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write('True')
            speak("Toggled circuit mode to " + str(configCircuit) + "!")
            return

        elif self.path.startswith('/set/groundspeed'):
            groundspeedString = self.path.split('/')[-1]
            print "\n\ngroundspeedString:"
            print groundspeedString
            print "\n\n"

            global configGroundspeed
            configGroundspeed = float(groundspeedString)

            print "\n\nconfigGroundspeed:"
            print configGroundspeed
            print "\n\n"

            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write('True')
            speak("Set groundspeed to " + str(configGroundspeed) + " meters per second!")
            return

        elif self.path.startswith('/set/heading'):
            headingString = self.path.split('/')[-1]
            print "\n\nheadingString:"
            print headingString
            print "\n\n"

            global configHeading
            configHeading = int(headingString)
            global currentNodeStable
            currentNodeStable = False
            global currentNodeStableCount
            currentNodeStableCount = 0

            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write('True')
            speak("Set heading to " + str(configHeading) + " degrees!")
            return


        elif self.path.startswith('/set/alt'):
            altString = self.path.split('/')[-1]
            print "\n\naltString:"
            print altString
            print "\n\n"

            global configAlt
            configAlt = float(altString)

            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write('True')
            speak("Set altitude to " + str(altString) + " meters!")
            return

        elif self.path.startswith('/set/destqueue'):
            latLngString = self.path.split('/')[-1]
            print "\n\nlatLngString:"
            print latLngString
            print "\n\n"

            latLngString = latLngString.replace("%20", " ")
            print latLngString
            latLngTuple = tuple(float(x) for x in latLngString.strip('()').split(','))
            lat = latLngTuple[0]
            lng = latLngTuple[1]
            global configAlt
            node = DestNode(lat, lng, configAlt)
            node.setHeading(configHeading)
            global destQueue
            destQueue.put(node)


            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write('True')
            speak("Added route node!")
            return

        elif self.path == '/get/destqueue':
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()

            self.wfile.write('(')
            first = True
            for node in list(destQueue.queue):
                if not first:
                    self.wfile.write(', ')

                self.wfile.write(node.lat)
                self.wfile.write(', ')
                self.wfile.write(node.lng)
                first = False

            self.wfile.write(')')
            return

        elif self.path == '/clear/destqueue':
            with destQueue.mutex:
                destQueue.queue.clear()

            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write('True')
            speak("Cleared route!")

        elif self.path == '/get/latlon':
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write('(')
            if CONNECT_VEHICLE:
                loc = vehicle.location.global_frame
                self.wfile.write(loc.lat)
                self.wfile.write(', ')
                self.wfile.write(loc.lon)
                self.wfile.write(')')
            else:
                self.wfile.write(40.0967841453)
                self.wfile.write(', ')
                self.wfile.write(-88.1970512867)
                self.wfile.write(')')
            return

        elif self.path == '/action/arm':
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            # #Check that vehicle is armable
            # while not vehicle.is_armable:
            #     print " Waiting for vehicle to initialise..."
            #     print " GPS eph (HDOP): %s" % vehicle.gps_0.eph
            #     time.sleep(1)
            #     # If required, you can provide additional information about initialisation
            #     # using `vehicle.gps_0.fix_type` and `vehicle.mode.name`.


            if CONNECT_VEHICLE:
                global armingInProgress
                armingInProgress = True
                print "\nSet Vehicle.armed=True (currently: %s)" % vehicle.armed
                vehicle.armed = True
                speak("Arming!")
                while not vehicle.armed:
                    print " Waiting for arming..."
                    time.sleep(1)
                print " Vehicle is armed: %s" % vehicle.armed
                armingInProgress = False
                speak("Successfully armed!")

            #vehicle.simple_takeoff(800) # Take off to target altitude - needed for simulator
            return

        elif self.path == '/action/disarm':
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            if CONNECT_VEHICLE:
                vehicle.channels.overrides['4'] = None
                global currentDestinationNode
                currentDestinationNode = None
                global currentDestinationNodeReached
                currentDestinationNodeReached = False
                print "\nSet Vehicle.armed=False (currently: %s)" % vehicle.armed
                vehicle.armed = False
                global armingInProgress
                armingInProgress = True
                speak("Disarming!")
                while vehicle.armed:
                    print " Waiting for disarming..."
                    time.sleep(1)
                print " Vehicle is armed: %s" % vehicle.armed
                armingInProgress = False
                speak("Successfully disarmed!")
            return

        elif self.path == '/action/loiter':
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            if CONNECT_VEHICLE:
                vehicle.channels.overrides['4'] = None
                global currentDestinationNode
                currentDestinationNode = None
                global currentDestinationNodeReached
                currentDestinationNodeReached = False
                print "\nSet Vehicle.mode=LOITER (currently: %s)" % vehicle.mode.name
                global modeChangeInProgress
                modeChangeInProgress = True
                vehicle.mode = VehicleMode("LOITER")
                speak("Engaging manual control!")
                while not vehicle.mode.name=='LOITER':  #Wait until mode has changed
                    print " Waiting for mode change ..."
                    time.sleep(1)

                modeChangeInProgress = False
                speak("Manual control engaged!")
            return

        elif self.path == '/action/guided':
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            if CONNECT_VEHICLE:
                guidedNextNode()

            return

        elif self.path == '/action/takeoff':
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            global modeChangeInProgress
            modeChangeInProgress = True
            vehicle.mode = VehicleMode("GUIDED")
            speak("Engaging autopilot!")
            while not vehicle.mode.name=='GUIDED':  #Wait until mode has changed
                print " Waiting for mode change ..."
                time.sleep(1)

            modeChangeInProgress = False
            vehicle.simple_takeoff(configAlt) # Take off to target altitude

        elif self.path == '/action/rtl':
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            global modeChangeInProgress
            modeChangeInProgress = True
            vehicle.mode = VehicleMode("RTL")
            speak("Engaging autopilot!")
            while not vehicle.mode.name=='RTL':  #Wait until mode has changed
                print " Waiting for mode change ..."
                time.sleep(1)

            modeChangeInProgress = False

        elif self.path == '/routes.html':
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write('''<!DOCTYPE html>
                             <html>
                             <head></head>
                             <title>SecDrone Saved Routes</title>
                             <body>''')

            for i in range(json_routes_index):
                if os.path.isfile(json_routes_prefix + str(i) + json_routes_suffix):
                    json_data = open(json_routes_prefix + str(i) + json_routes_suffix).read()
                    json_data = json_data.replace('\n', '<br />')
                    self.wfile.write('<br />' + str(i) + ':<br />')
                    self.wfile.write(json_data)

            self.wfile.write('''</body></html>''')
            return

        elif self.path == '/status.html':
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()

            if CONNECT_VEHICLE:
                self.wfile.write('<h1>')

                self.wfile.write("%s" % vehicle.battery)
                self.wfile.write('<br />')

                if vehicle.channels['3'] > 900:
                    self.wfile.write("TX IS ON!")
                else:
                    self.wfile.write("TX IS OFF!")

                self.wfile.write('<br />')
                self.wfile.write(" Armed: %s" % vehicle.armed   )
                self.wfile.write('<br />')
                self.wfile.write(" Mode: %s" % vehicle.mode.name    )
                global armingInProgress
                if armingInProgress:
                    self.wfile.write("<br />ARMED STATE CHANGE IN PROGRESS")

                global modeChangeInProgress
                if modeChangeInProgress:
                    self.wfile.write("<br />MODE CHANGE IN PROGRESS")

                self.wfile.write('<br /></h1><h2>')

                self.wfile.write('Configured altitude: ')
                global configAlt
                self.wfile.write(configAlt)
                self.wfile.write('<br />')

                self.wfile.write('Configured groundspeed: ')
                global configGroundspeed
                self.wfile.write(configGroundspeed)
                self.wfile.write('<br />')

                self.wfile.write('Configured heading: ')
                global configHeading
                self.wfile.write(configHeading)
                self.wfile.write('<br />')

                self.wfile.write('Circuit route: ')
                global configCircuit
                self.wfile.write(configCircuit)
                self.wfile.write('<br />')

                self.wfile.write('Altitude override: ')
                global configAltitudeOverride
                self.wfile.write(configAltitudeOverride)
                self.wfile.write('<br />')

                self.wfile.write('currentDestinationNodeReached: ')
                global currentDestinationNodeReached
                self.wfile.write(currentDestinationNodeReached)
                self.wfile.write('<br />')

                self.wfile.write('currentNodeStable: ')
                global currentNodeStable
                self.wfile.write(currentNodeStable)
                self.wfile.write('<br />')

                self.wfile.write('currentNodeStableCount: ')
                global currentNodeStableCount
                self.wfile.write(currentNodeStableCount)
                self.wfile.write('<br />')






                self.wfile.write("</h2> Ch1: %s" % vehicle.channels['1'])
                self.wfile.write('<br />')
                self.wfile.write(" Ch2: %s" % vehicle.channels['2'])
                self.wfile.write('<br />')
                self.wfile.write(" Ch3: %s" % vehicle.channels['3'])
                self.wfile.write('<br />')
                self.wfile.write(" Ch4: %s" % vehicle.channels['4'])
                self.wfile.write('<br />')

                self.wfile.write("Get all vehicle attribute values:")
                self.wfile.write('<br />')
                self.wfile.write(" Global Location: %s" % vehicle.location.global_frame)
                self.wfile.write('<br />')
                self.wfile.write(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
                self.wfile.write('<br />')
                self.wfile.write(" Local Location: %s" % vehicle.location.local_frame)
                writeToLog("\nGlobal Location: %s\n" % vehicle.location.global_frame)
                writeToLog("Global Location (relative altitude): %s\n" % vehicle.location.global_relative_frame)
                writeToLog("Local Location: %s\n" % vehicle.location.local_frame)
                self.wfile.write('<br />')
                self.wfile.write(" Attitude: %s" % vehicle.attitude)
                self.wfile.write('<br />')
                self.wfile.write(" Velocity: %s" % vehicle.velocity)
                writeToLog(" Velocity: %s\n" % vehicle.velocity)
                self.wfile.write('<br />')
                gpsInf = vehicle.gps_0
                self.wfile.write(" GPS:")
                self.wfile.write('<br />')
                self.wfile.write(" eph: %s" % gpsInf.eph)
                self.wfile.write('<br />')
                self.wfile.write(" epv: %s" % gpsInf.epv)
                self.wfile.write('<br />')
                self.wfile.write(" fix_type: %s" % gpsInf.fix_type)
                self.wfile.write('<br />')
                self.wfile.write(" satellites_visible : %s" % gpsInf.satellites_visible)
                self.wfile.write('<br />')
                self.wfile.write(" Gimbal status: %s" % vehicle.gimbal)
                self.wfile.write('<br />')
                self.wfile.write(" EKF OK?: %s" % vehicle.ekf_ok)
                self.wfile.write('<br />')
                self.wfile.write(" Last Heartbeat: %s" % vehicle.last_heartbeat)
                self.wfile.write('<br />')
                self.wfile.write(" Rangefinder: %s" % vehicle.rangefinder)
                self.wfile.write('<br />')
                self.wfile.write(" Rangefinder distance: %s" % vehicle.rangefinder.distance)
                self.wfile.write('<br />')
                self.wfile.write(" Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
                self.wfile.write('<br />')
                self.wfile.write(" Heading: %s" % vehicle.heading)
                self.wfile.write('<br />')
                self.wfile.write(" Is Armable?: %s" % vehicle.is_armable)
                self.wfile.write('<br />')
                self.wfile.write(" System status: %s" % vehicle.system_status.state)
                self.wfile.write('<br />')
                self.wfile.write(" Groundspeed: %s" % vehicle.groundspeed    )
                self.wfile.write('<br />')
                self.wfile.write(" Airspeed: %s" % vehicle.airspeed    )

                self.wfile.write('<br /><h2>Saved routes:</h2>')
                self.wfile.write('<br />')
                global json_routes_index
                for i in range(json_routes_index):
                    self.wfile.write('<button onclick="httpGetAsync(\'/load/route/' + str(i) + '\')"><h2>' + json_routes_names_list[i] + '</h2></button><br />')

                self.wfile.write('<br /><br /><h2>Route:</h2>')

                i = 0
                for node in list(destQueue.queue):
                    self.wfile.write('<br />')
                    self.wfile.write(i)
                    i = i+1
                    self.wfile.write(': lat: ')
                    self.wfile.write(node.lat)
                    self.wfile.write(', lng: ')
                    self.wfile.write(node.lng)
                    self.wfile.write(', alt: ')
                    self.wfile.write(node.alt)
                    self.wfile.write(', hdg: ')
                    self.wfile.write(node.hdg)
            else:
                self.wfile.write('<h1>Not Connected!</h1>')

            return

        elif self.path == '/cam.mjpg' and USE_CAMERA and USE_MJPEG_STREAM:
            print "MJPEG Requested"
            self.send_response(200)
            self.send_header('Content-type','multipart/x-mixed-replace; boundary=--jpgboundary')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            while True:
                try:
                    if currentJpg is not None:
                        self.wfile.write("--jpgboundary")
                        self.send_header('Content-type','image/jpeg')
                        self.send_header('Content-length', str(currentJpgLength))
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        currentJpg.save(self.wfile,'JPEG')
                    time.sleep(1.0/viewRate)
                except KeyboardInterrupt:
                    break

            return

        elif self.path == '/index.html' or self.path == '/':
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            with open('index.html', 'r') as myfile:
                data=myfile.read()

            global mapsAPIKey
            data = data.replace("$GOOGLEMAPSAPIKEYGOESHERE", mapsAPIKey)
            self.wfile.write(data)
            speak("Control interface loaded!")
            return

        elif self.path == '/map.html':
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            with open('map.html', 'r') as myfile:
                data=myfile.read()

            self.wfile.write(data)
            return

        elif self.path == '/green_marker.png':
            self.send_response(200)
            self.send_header('Content-type','image/png')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            with open('green_marker.png', 'r') as myfile:
                data=myfile.read()

            self.wfile.write(data)
            return

        else:
            self.send_response(404)
            self.send_header('Content-type','image/png')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write('<html><head></head><body>404 Not Found</body></html>')
            return
    def log_message(self, format, *args):
        return

signal.signal(signal.SIGINT, handler)


#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='SecDrone')
parser.add_argument('--connect', default='/dev/ttyACM0',
                    help="vehicle connection target. Default '/dev/ttyACM0'")
args = parser.parse_args()

with open('mapsAPIKey.txt', 'r') as myfile:
    mapsAPIKey=myfile.read()

while os.path.isfile(json_routes_prefix + str(json_routes_index) + json_routes_suffix):
    json_data = open(json_routes_prefix + str(json_routes_index) + json_routes_suffix)
    data = json.load(json_data)
    json_routes_names_list.append(data['name'])
    json_routes_index = json_routes_index+1

print "\njson_routes_index: %s" % str(json_routes_index)

print "\nGoogle maps API key: %s" % mapsAPIKey

# Connect to the Vehicle.
#   Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
print "\nConnecting to vehicle on: %s" % args.connect
if CONNECT_VEHICLE:
    speak("Connecting to vehicle!")
    vehicle = connect(args.connect, wait_ready=True)

    # Get Vehicle Home location - will be `None` until first set by autopilot
    while not vehicle.home_location:
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()
        if not vehicle.home_location:
            print " Waiting for home location ..."
    # We have a home location, so print it!
    print "\n Home location: %s" % vehicle.home_location


    # Set vehicle home_location, mode, and armed attributes (the only settable attributes)

    print "\nSet new home location"
    # Home location must be within 50km of EKF home location (or setting will fail silently)
    # In this case, just set value to current location with an easily recognisable altitude (222)
    my_location_alt=vehicle.location.global_frame
    my_location_alt.alt=222.0
    vehicle.home_location=my_location_alt
    print " New Home Location (from attribute - altitude should be 222): %s" % vehicle.home_location

    #Confirm current value on vehicle by re-downloading commands
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    print " New Home Location (from vehicle - altitude should be 222): %s" % vehicle.home_location


    print "\nSet Vehicle.mode=STABILIZE (currently: %s)" % vehicle.mode.name
    vehicle.mode = VehicleMode("STABILIZE")
    while not vehicle.mode.name=='STABILIZE':  #Wait until mode has changed
        print " Waiting for mode change ..."
        time.sleep(1)

    # Get/Set Vehicle Parameters
    print "\nRead and write parameters"
    print " Read vehicle param 'GPS_HDOP_GOOD': %s" % vehicle.parameters['GPS_HDOP_GOOD'] #NOTE: ORIGINAL IS 230.0

    print " Write vehicle param 'GPS_HDOP_GOOD' : 230.0"
    vehicle.parameters['GPS_HDOP_GOOD']=230.0
    print " Read new value of param 'GPS_HDOP_GOOD': %s" % vehicle.parameters['GPS_HDOP_GOOD']

    print " Read vehicle param 'ARMING_CHECK': %s" % vehicle.parameters['ARMING_CHECK'] #NOTE: ORIGINAL IS

    print " Write vehicle param 'ARMING_CHECK' : 1"
    vehicle.parameters['ARMING_CHECK']=1
    print " Read new value of param 'ARMING_CHECK': %s" % vehicle.parameters['ARMING_CHECK']

    routeThreadObj = RouteThread()
    routeThreadObj.setName('Route Thread')
    routeThreadObj.start()

    systemMonitorThreadObj = SystemMonitorThread()
    systemMonitorThreadObj.setName('System Monitor Thread')
    systemMonitorThreadObj.start()

    if EMERGENCY_LANDING_ENABLED:
        emergencyLandingThreadObj = EmergencyLandingThread()
        emergencyLandingThreadObj.setName('Emergency Landing Thread')
        emergencyLandingThreadObj.start()

if USE_CAMERA:
    captureThreadObj = CaptureThread()
    captureThreadObj.setName('Capture Thread')
    captureThreadObj.start()

speak("Starting control interface!")

server = ThreadedHTTPServer(('',8080),CamHandler)
# server.allow_reuse_address=True
# server.server_bind()
# server.server_activate()
print "server started"
server.serve_forever()
