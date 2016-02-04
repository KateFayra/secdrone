from dronekit import connect, VehicleMode
import signal
import time
import cv2
#import Image
from PIL import Image
from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
import StringIO
import time
import sys
from threading import Thread
capture = None
framerate = 24

global capture
capture = cv2.VideoCapture(0)
capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 320);
capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 240);
capture.set(cv2.cv.CV_CAP_PROP_SATURATION,0.2);
global img
global server

def webserver_thread(arg):
    server = HTTPServer(('',8080),CamHandler)
    print "server started"
    server.serve_forever()


def handler(signum, frame):
    #Close vehicle object before exiting script
    print "\nSIGINT CLOSING CONNECTION!"
    vehicle.close()
    capture.release()
    while thread.isAlive():
        try:
            thread._Thread__stop()
        except:
            print " could not be terminated"

    server.socket.close()
    print("Completed, exiting")
    sys.exit()


class CamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        print "REQUEST GOT:"
        print self.path

        if self.path == '/status.html' or  self.path == '/status.html/':
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.end_headers()
            self.wfile.write('<html><head></head><body><h1>')

            self.wfile.write("%s" % vehicle.battery)
            self.wfile.write('</br>')

            if vehicle.channels['3'] > 900:
                self.wfile.write("TX IS ON!")
            else:
                self.wfile.write("TX IS OFF!")

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
            self.wfile.write('</br>')
            self.wfile.write(" Mode: %s" % vehicle.mode.name    )
            self.wfile.write('</br>')
            self.wfile.write(" Armed: %s" % vehicle.armed   )
            self.wfile.write('</br>')
            self.wfile.write('</br>')
            self.wfile.write('</body></html>')

        elif self.path.endswith('.mjpg'):
            print "MJPEG Requested"
            self.send_response(200)
            self.send_header('Content-type','multipart/x-mixed-replace; boundary=--jpgboundary')
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
                    print "Saving image to %s", tmpFile
                    self.wfile.write("--jpgboundary")
                    self.send_header('Content-type','image/jpeg')
                    self.send_header('Content-length',str(tmpFile.len))
                    self.end_headers()
                    jpg.save(self.wfile,'JPEG')
                    time.sleep(1/framerate)
                except KeyboardInterrupt:
                    break
            return
        elif self.path == '/index.html' or self.path == '/' or self.path == '/index.html/':
            print "HTML Requested"
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.end_headers()
            self.wfile.write('<html><head></head><body>')
            self.wfile.write('<img src="/cam.mjpg"/>')
            self.wfile.write('</body></html>')
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

print " Write vehicle param 'GPS_HDOP_GOOD' : 300.0"
vehicle.parameters['GPS_HDOP_GOOD']=300.0
print " Read new value of param 'GPS_HDOP_GOOD': %s" % vehicle.parameters['GPS_HDOP_GOOD']

print " Read vehicle param 'ARMING_CHECK': %s" % vehicle.parameters['ARMING_CHECK'] #NOTE: ORIGINAL IS

print " Write vehicle param 'ARMING_CHECK' : 0"
vehicle.parameters['ARMING_CHECK']=0
print " Read new value of param 'ARMING_CHECK': %s" % vehicle.parameters['ARMING_CHECK']



# Check that vehicle is armable
# while not vehicle.is_armable:
#     print " Waiting for vehicle to initialise..."
#     print " GPS eph (HDOP): %s" % vehicle.gps_0.eph
#     time.sleep(1)
#     # If required, you can provide additional information about initialisation
#     # using `vehicle.gps_0.fix_type` and `vehicle.mode.name`.
#
# print "\nSet Vehicle.armed=True (currently: %s)" % vehicle.armed
# vehicle.armed = True
# while not vehicle.armed:
#     print " Waiting for arming..."
#     time.sleep(1)
# print " Vehicle is armed: %s" % vehicle.armed

# # Override the channel for roll and yaw
# vehicle.channel_override = { "3" : 900 }
# vehicle.flush()
#
# #print current override values
# print "Current overrides are:", vehicle.channel_override
#
# # Print channel values (values if overrides removed)
# print "Channel default values:", vehicle.channel_readback
#
# # Cancel override by setting channels to 0
# vehicle.channel_override = { "3" : 0}
# vehicle.flush()


thread = Thread(target = webserver_thread, args = (10, ))
thread.start()

while True:
    # Print channel values (values if overrides removed)
    # print "\n\n\nChannel default values:"
    # # Get all channel values from RC transmitter
    # print "Channel values from RC Tx:", vehicle.channels
    #
    # # Access channels individually
    # print "Read channels individually:"
    # print " Ch1: %s" % vehicle.channels['1']
    # print " Ch2: %s" % vehicle.channels['2']
    # print " Ch3: %s" % vehicle.channels['3']
    # print " Ch4: %s" % vehicle.channels['4']
    #
    # if vehicle.channels['3'] > 900:
    #     print "TX IS ON!"
    # else:
    #     print "TX IS OFF!"
    #
    # print "\n\n\nGet all vehicle attribute values:"
    # print " Global Location: %s" % vehicle.location.global_frame
    # print " Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
    # print " Local Location: %s" % vehicle.location.local_frame
    # print " Attitude: %s" % vehicle.attitude
    # print " Velocity: %s" % vehicle.velocity
    # gpsInf = vehicle.gps_0
    # print " GPS:"
    # print " eph: %s" % gpsInf.eph
    # print " epv: %s" % gpsInf.epv
    # print " fix_type: %s" % gpsInf.fix_type
    # print " satellites_visible : %s" % gpsInf.satellites_visible
    # print " Gimbal status: %s" % vehicle.gimbal
    # print " Battery: %s" % vehicle.battery
    # print " EKF OK?: %s" % vehicle.ekf_ok
    # print " Last Heartbeat: %s" % vehicle.last_heartbeat
    # print " Rangefinder: %s" % vehicle.rangefinder
    # print " Rangefinder distance: %s" % vehicle.rangefinder.distance
    # print " Rangefinder voltage: %s" % vehicle.rangefinder.voltage
    # print " Heading: %s" % vehicle.heading
    # print " Is Armable?: %s" % vehicle.is_armable
    # print " System status: %s" % vehicle.system_status.state
    # print " Groundspeed: %s" % vehicle.groundspeed    # settable
    # print " Airspeed: %s" % vehicle.airspeed    # settable
    # print " Mode: %s" % vehicle.mode.name    # settable
    # print " Armed: %s" % vehicle.armed    # settable
    time.sleep(1)
# Add and remove and attribute callbacks

