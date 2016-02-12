from dronekit import connect, VehicleMode, LocationGlobalRelative
import signal
import time
import cv2
#import Image
from PIL import Image
from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
import StringIO
import time
import sys
from SocketServer import ThreadingMixIn
import threading
from threading import Thread
capture = None
framerate = 24
import Queue

global capture
capture = cv2.VideoCapture(0)
capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 640);
capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 360);
capture.set(cv2.cv.CV_CAP_PROP_SATURATION,0.2);
global img
global server
armingInProgress = False
modeChangeInProgress = False
destQueue = Queue.Queue()

class destNode:
    def __init__(self, lat, lng, alt):
        self.lat = lat
        self.lng = lng
        self.alt = alt


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""


def handler(signum, frame):
    #Close vehicle object before exiting script
    print "\nSIGINT CLOSING CONNECTION!"
    vehicle.close()
    capture.release()
    while thread.isAlive():
        try:
            thread._Thread__stop()
        except:
            print "Thread could not be terminated"

    for thr in threading.enumerate():
        while thr.isAlive():
            try:
                thr._Thread__stop()
            except:
                print "Additional thread could not be terminated"

    server.socket.close()
    print("Completed, exiting")
    sys.exit()


class CamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith('/set/destqueue'):
            latLngString = self.path.split('/')[-1]
            print "\n\nlatLngString:"
            print latLngString
            print "\n\n"

            latLngString = latLngString.replace("%20", " ")
            print latLngString
            latLngTuple = tuple(float(x) for x in latLngString.strip('()').split(','))
            lat = latLngTuple[0]
            lng = latLngTuple[1]
            alt = 4
            node = destNode(lat, lng, alt)
            global destQueue
            destQueue.put(node)


            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write('True')
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

        elif self.path == '/get/latlon':
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write('(')
            loc = vehicle.location.global_frame
            self.wfile.write(loc.lat)
            self.wfile.write(', ')
            self.wfile.write(loc.lon)
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

            global armingInProgress
            armingInProgress = True
            print "\nSet Vehicle.armed=True (currently: %s)" % vehicle.armed
            vehicle.armed = True
            while not vehicle.armed:
                print " Waiting for arming..."
                time.sleep(1)
            print " Vehicle is armed: %s" % vehicle.armed
            armingInProgress = False
            return

        elif self.path == '/action/disarm':
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            print "\nSet Vehicle.armed=False (currently: %s)" % vehicle.armed
            vehicle.armed = False
            global armingInProgress
            armingInProgress = True
            while vehicle.armed:
                print " Waiting for disarming..."
                time.sleep(1)
            print " Vehicle is armed: %s" % vehicle.armed
            armingInProgress = False
            return

        elif self.path == '/action/loiter':
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            print "\nSet Vehicle.mode=LOITER (currently: %s)" % vehicle.mode.name
            global modeChangeInProgress
            modeChangeInProgress = True
            vehicle.mode = VehicleMode("LOITER")
            while not vehicle.mode.name=='LOITER':  #Wait until mode has changed
                print " Waiting for mode change ..."
                time.sleep(1)

            modeChangeInProgress = False
            return

        elif self.path == '/action/guided':
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            if destQueue.empty():
                return

            print "\nSet Vehicle.mode=GUIDED (currently: %s)" % vehicle.mode.name
            global modeChangeInProgress
            modeChangeInProgress = True
            vehicle.mode = VehicleMode("GUIDED")
            while not vehicle.mode.name=='GUIDED':  #Wait until mode has changed
                print " Waiting for mode change ..."
                time.sleep(1)

            modeChangeInProgress = False
            node = destQueue.get()
            location = LocationGlobalRelative(node.lat, node.lng, 4)
            vehicle.simple_goto(location, groundspeed=0.5)
            return

        elif self.path == '/status.html':
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write('<h1>')

            self.wfile.write("%s" % vehicle.battery)
            self.wfile.write('</br>')

            if vehicle.channels['3'] > 900:
                self.wfile.write("TX IS ON!")
            else:
                self.wfile.write("TX IS OFF!")
                if vehicle.mode.name != "LAND" and vehicle.mode.name != "LOITER":
                    print "\nSet Vehicle.mode=LAND (currently: %s)" % vehicle.mode.name
                    vehicle.mode = VehicleMode("LAND")


            self.wfile.write('</br>')
            self.wfile.write(" Armed: %s" % vehicle.armed   )
            self.wfile.write('</br>')
            self.wfile.write(" Mode: %s" % vehicle.mode.name    )
            global armingInProgress
            if armingInProgress:
                self.wfile.write("</br>ARMED STATE CHANGE IN PROGRESS")

            global modeChangeInProgress
            if modeChangeInProgress:
                self.wfile.write("</br>MODE CHANGE IN PROGRESS")

            self.wfile.write('</br></h1>')

            self.wfile.write(" Ch1: %s" % vehicle.channels['1'])
            self.wfile.write('</br>')
            self.wfile.write(" Ch2: %s" % vehicle.channels['2'])
            self.wfile.write('</br>')
            self.wfile.write(" Ch3: %s" % vehicle.channels['3'])
            self.wfile.write('</br>')
            self.wfile.write(" Ch4: %s" % vehicle.channels['4'])
            self.wfile.write('</br>')

            self.wfile.write("Get all vehicle attribute values:")
            self.wfile.write('</br>')
            self.wfile.write(" Global Location: %s" % vehicle.location.global_frame)
            self.wfile.write('</br>')
            self.wfile.write(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
            self.wfile.write('</br>')
            self.wfile.write(" Local Location: %s" % vehicle.location.local_frame)
            self.wfile.write('</br>')
            self.wfile.write(" Attitude: %s" % vehicle.attitude)
            self.wfile.write('</br>')
            self.wfile.write(" Velocity: %s" % vehicle.velocity)
            self.wfile.write('</br>')
            gpsInf = vehicle.gps_0
            self.wfile.write(" GPS:")
            self.wfile.write('</br>')
            self.wfile.write(" eph: %s" % gpsInf.eph)
            self.wfile.write('</br>')
            self.wfile.write(" epv: %s" % gpsInf.epv)
            self.wfile.write('</br>')
            self.wfile.write(" fix_type: %s" % gpsInf.fix_type)
            self.wfile.write('</br>')
            self.wfile.write(" satellites_visible : %s" % gpsInf.satellites_visible)
            self.wfile.write('</br>')
            self.wfile.write(" Gimbal status: %s" % vehicle.gimbal)
            self.wfile.write('</br>')
            self.wfile.write(" EKF OK?: %s" % vehicle.ekf_ok)
            self.wfile.write('</br>')
            self.wfile.write(" Last Heartbeat: %s" % vehicle.last_heartbeat)
            self.wfile.write('</br>')
            self.wfile.write(" Rangefinder: %s" % vehicle.rangefinder)
            self.wfile.write('</br>')
            self.wfile.write(" Rangefinder distance: %s" % vehicle.rangefinder.distance)
            self.wfile.write('</br>')
            self.wfile.write(" Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
            self.wfile.write('</br>')
            self.wfile.write(" Heading: %s" % vehicle.heading)
            self.wfile.write('</br>')
            self.wfile.write(" Is Armable?: %s" % vehicle.is_armable)
            self.wfile.write('</br>')
            self.wfile.write(" System status: %s" % vehicle.system_status.state)
            self.wfile.write('</br>')
            self.wfile.write(" Groundspeed: %s" % vehicle.groundspeed    )
            self.wfile.write('</br>')
            self.wfile.write(" Airspeed: %s" % vehicle.airspeed    )
            return

        elif self.path == '/cam.mjpg':
            print "MJPEG Requested"
            self.send_response(200)
            self.send_header('Content-type','multipart/x-mixed-replace; boundary=--jpgboundary')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            while True:
                try:
                    rc,img = capture.read()
                    if not rc:
                        continue
                    imgRGB=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
                    jpg = Image.fromarray(imgRGB)
                    tmpFile = StringIO.StringIO()
                    jpg.save(tmpFile,'JPEG')
                    self.wfile.write("--jpgboundary")
                    self.send_header('Content-type','image/jpeg')
                    self.send_header('Content-length',str(tmpFile.len))
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    jpg.save(self.wfile,'JPEG')
                    time.sleep(1/framerate)
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

            self.wfile.write(data)
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


signal.signal(signal.SIGINT, handler)




#Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect', default='/dev/ttyACM0',
                    help="vehicle connection target. Default '127.0.0.1:14550'")
args = parser.parse_args()


# Connect to the Vehicle.
#   Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
print "\nConnecting to vehicle on: %s" % args.connect
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


print "\nSet Vehicle.mode=LOITER (currently: %s)" % vehicle.mode.name
vehicle.mode = VehicleMode("LOITER")
while not vehicle.mode.name=='LOITER':  #Wait until mode has changed
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
vehicle.parameters['ARMING_CHECK']=0
print " Read new value of param 'ARMING_CHECK': %s" % vehicle.parameters['ARMING_CHECK']


server = ThreadedHTTPServer(('',8080),CamHandler)
print "server started"
server.serve_forever()
