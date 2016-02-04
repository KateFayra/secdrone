"""
vehicle_state.py: 

Demonstrates how to get and set vehicle state and parameter information, 
and how to observe vehicle attribute (state) changes.

Full documentation is provided at http://python.dronekit.io/examples/vehicle_state.html
"""
from dronekit import connect, VehicleMode
import signal
import time

def handler(signum, frame):
    #Close vehicle object before exiting script
    print "\nSIGINT CLOSING CONNECTION!"
    vehicle.close()
    print("Completed, exiting")
    sys.exit()

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
#Bundle extraParams = new Bundle();
#extraParams.putInt(ConnectionType.EXTRA_USB_BAUD_RATE, 57600); // Set default baud rate to 57600
#ConnectionParameter connectionParams = new ConnectionParameter(ConnectionType.TYPE_USB, extraParams, null);
#vehicle = connect(connectionParams);
#vehicle = connect('/dev/ttyACM0', baud=57600)

# Get all vehicle attributes (state)
#while True:
print "\nGet all vehicle attribute values:"
print " Global Location: %s" % vehicle.location.global_frame
print " Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
print " Local Location: %s" % vehicle.location.local_frame
print " Attitude: %s" % vehicle.attitude
print " Velocity: %s" % vehicle.velocity
gpsInf = vehicle.gps_0
print " GPS:"
print " eph: %s" % gpsInf.eph
print " epv: %s" % gpsInf.epv
print " fix_type: %s" % gpsInf.fix_type
print " satellites_visible : %s" % gpsInf.satellites_visible
print " Gimbal status: %s" % vehicle.gimbal
print " Battery: %s" % vehicle.battery
print " EKF OK?: %s" % vehicle.ekf_ok
print " Last Heartbeat: %s" % vehicle.last_heartbeat
print " Rangefinder: %s" % vehicle.rangefinder
print " Rangefinder distance: %s" % vehicle.rangefinder.distance
print " Rangefinder voltage: %s" % vehicle.rangefinder.voltage
print " Heading: %s" % vehicle.heading
print " Is Armable?: %s" % vehicle.is_armable
print " System status: %s" % vehicle.system_status.state
print " Groundspeed: %s" % vehicle.groundspeed    # settable
print " Airspeed: %s" % vehicle.airspeed    # settable
print " Mode: %s" % vehicle.mode.name    # settable
print " Armed: %s" % vehicle.armed    # settable
time.sleep(1)


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

print "\nSet Vehicle.armed=True (currently: %s)" % vehicle.armed 
vehicle.armed = True
while not vehicle.armed:
    print " Waiting for arming..."
    time.sleep(1)
print " Vehicle is armed: %s" % vehicle.armed

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


while True:
    # Print channel values (values if overrides removed)
    print "\n\n\nChannel default values:"
    # Get all channel values from RC transmitter
    print "Channel values from RC Tx:", vehicle.channels

    # Access channels individually
    print "Read channels individually:"
    print " Ch1: %s" % vehicle.channels['1']
    print " Ch2: %s" % vehicle.channels['2']
    print " Ch3: %s" % vehicle.channels['3']
    print " Ch4: %s" % vehicle.channels['4']

    if vehicle.channels['3'] > 900:
        print "TX IS ON!"
    else:
        print "TX IS OFF!"

    print "\n\n\nGet all vehicle attribute values:"
    print " Global Location: %s" % vehicle.location.global_frame
    print " Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
    print " Local Location: %s" % vehicle.location.local_frame
    print " Attitude: %s" % vehicle.attitude
    print " Velocity: %s" % vehicle.velocity
    gpsInf = vehicle.gps_0
    print " GPS:"
    print " eph: %s" % gpsInf.eph
    print " epv: %s" % gpsInf.epv
    print " fix_type: %s" % gpsInf.fix_type
    print " satellites_visible : %s" % gpsInf.satellites_visible
    print " Gimbal status: %s" % vehicle.gimbal
    print " Battery: %s" % vehicle.battery
    print " EKF OK?: %s" % vehicle.ekf_ok
    print " Last Heartbeat: %s" % vehicle.last_heartbeat
    print " Rangefinder: %s" % vehicle.rangefinder
    print " Rangefinder distance: %s" % vehicle.rangefinder.distance
    print " Rangefinder voltage: %s" % vehicle.rangefinder.voltage
    print " Heading: %s" % vehicle.heading
    print " Is Armable?: %s" % vehicle.is_armable
    print " System status: %s" % vehicle.system_status.state
    print " Groundspeed: %s" % vehicle.groundspeed    # settable
    print " Airspeed: %s" % vehicle.airspeed    # settable
    print " Mode: %s" % vehicle.mode.name    # settable
    print " Armed: %s" % vehicle.armed    # settable
    time.sleep(1)
# Add and remove and attribute callbacks

#Define callback for `vehicle.attitude` observer
last_attitude_cache=None
def attitude_callback(self, attr_name, value):
    # `attr_name` - the observed attribute (used if callback is used for multiple attributes)
    # `self` - the associated vehicle object (used if a callback is different for multiple vehicles)
    # `value` is the updated attribute value.
    global last_attitude_cache
    # Only publish when value changes
    if value!=last_attitude_cache:
        print " CALLBACK: Attitude changed to", value
        last_attitude_cache=value

print "\nAdd `attitude` attribute callback/observer on `vehicle`"     
vehicle.add_attribute_listener('attitude', attitude_callback)

print " Wait 2s so callback invoked before observer removed"
time.sleep(2)

print " Remove Vehicle.attitude observer"    
# Remove observer added with `add_attribute_listener()` specifying the attribute and callback function
vehicle.remove_attribute_listener('attitude', attitude_callback)


        
# Add mode attribute callback using decorator (callbacks added this way cannot be removed).
print "\nAdd `mode` attribute callback/observer using decorator" 
@vehicle.on_attribute('mode')   
def decorated_mode_callback(self, attr_name, value):
    # `attr_name` is the observed attribute (used if callback is used for multiple attributes)
    # `attr_name` - the observed attribute (used if callback is used for multiple attributes)
    # `value` is the updated attribute value.
    print " CALLBACK: Mode changed to", value

print " Set mode=STABILIZE (currently: %s) and wait for callback" % vehicle.mode.name 
vehicle.mode = VehicleMode("STABILIZE")

print " Wait 2s so callback invoked before moving to next example"
time.sleep(2)

print "\n Attempt to remove observer added with `on_attribute` decorator (should fail)" 
try:
    vehicle.remove_attribute_listener('mode', decorated_mode_callback)
except:
    print " Exception: Cannot remove observer added using decorator"



 
# Demonstrate getting callback on any attribute change
def wildcard_callback(self, attr_name, value):
    print " CALLBACK: (%s): %s" % (attr_name,value)

print "\nAdd attribute callback detecting ANY attribute change"     
vehicle.add_attribute_listener('*', wildcard_callback)


print " Wait 1s so callback invoked before observer removed"
time.sleep(1)

print " Remove Vehicle attribute observer"    
# Remove observer added with `add_attribute_listener()`
vehicle.remove_attribute_listener('*', wildcard_callback)
    


# Get/Set Vehicle Parameters
print "\nRead and write parameters"
print " Read vehicle param 'THR_MIN': %s" % vehicle.parameters['THR_MIN']

print " Write vehicle param 'THR_MIN' : 10"
vehicle.parameters['THR_MIN']=10
print " Read new value of param 'THR_MIN': %s" % vehicle.parameters['THR_MIN']


print "\nPrint all parameters (iterate `vehicle.parameters`):"
for key, value in vehicle.parameters.iteritems():
    print " Key:%s Value:%s" % (key,value)
    

print "\nCreate parameter observer using decorator"
# Parameter string is case-insensitive
# Value is cached (listeners are only updated on change)
# Observer added using decorator can't be removed.
 
@vehicle.parameters.on_attribute('THR_MIN')  
def decorated_thr_min_callback(self, attr_name, value):
    print " PARAMETER CALLBACK: %s changed to: %s" % (attr_name, value)


print "Write vehicle param 'THR_MIN' : 20 (and wait for callback)"
vehicle.parameters['THR_MIN']=20
for x in range(1,5):
    #Callbacks may not be updated for a few seconds
    if vehicle.parameters['THR_MIN']==20:
        break
    time.sleep(1)


#Callback function for "any" parameter
print "\nCreate (removable) observer for any parameter using wildcard string"
def any_parameter_callback(self, attr_name, value):
    print " ANY PARAMETER CALLBACK: %s changed to: %s" % (attr_name, value)

#Add observer for the vehicle's any/all parameters parameter (defined using wildcard string ``'*'``)
vehicle.parameters.add_attribute_listener('*', any_parameter_callback)     
print " Change THR_MID and THR_MIN parameters (and wait for callback)"    
vehicle.parameters['THR_MID']=400  
vehicle.parameters['THR_MIN']=30


## Reset variables to sensible values.
print "\nReset vehicle attributes/parameters and exit"
vehicle.mode = VehicleMode("STABILIZE")
vehicle.armed = False
vehicle.parameters['THR_MIN']=130
vehicle.parameters['THR_MID']=500


#Close vehicle object before exiting script
print "\nClose vehicle object"
vehicle.close()

print("Completed")




