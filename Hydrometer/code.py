import busio
from adafruit_onewire.bus import OneWireBus
import time
import digitalio
from math import *
from board import *

#import ESP AT and MQTT
import adafruit_requests as requests
import adafruit_espatcontrol.adafruit_espatcontrol_socket as socket
from adafruit_espatcontrol import adafruit_espatcontrol
from adafruit_espatcontrol import adafruit_espatcontrol_wifimanager
import adafruit_minimqtt.adafruit_minimqtt as MQTT
from adafruit_io.adafruit_io import IO_MQTT

#import sensors
import adafruit_mpu6050
from adafruit_ds18x20 import DS18X20

#import Secrets
# Get wifi details and more from a secrets.py file
try:
    from secrets import secrets
except ImportError:
    print("All secret keys are kept in secrets.py, please add them there!")
    raise

#import Configs
try:
    from configs import configs
except ImportError:
    print("All configs are kept in configs.py, please add them there!")
    raise

#get bus
i2c = busio.I2C(GP5, GP4)
ow_bus = OneWireBus(GP6)
#Set sensor class
mpu6050 = adafruit_mpu6050.MPU6050(i2c, address = 0x68)
ds18 = DS18X20(ow_bus, ow_bus.scan()[0])

#Variables that will from config (later in dev)
temp_scale = configs['temp_scale'] #Celcius, Fahrenheit, Kelvin 
gravity_scale = configs['gravity_scale'] #SG, BRix, PLato
probe_time = configs['probe_time'] # Time in seconds from 900  (15 min) to Any 
OffsetX = configs['OffsetX']
OffsetY = configs['OffsetY']
OffsetZ = configs['OffsetZ']
#polynominal = configs['Polynominal'] 
# Calibration settings for polynominal. 
#  0.8664121875066527 
# +0.009562713003442422 *tilt
# -0.00018408281752418392 *tilt*tilt 
# +0.0000013556331354515971 *tilt*tilt*tilt

# Wifi Program


# Initialize UART connection to the ESP8266 WiFi Module.
RX = GP17 
TX = GP16
uart = busio.UART(TX, RX, receiver_buffer_size=2048)  # Use large buffer as we're not using hardware flow control.

esp = adafruit_espatcontrol.ESP_ATcontrol(uart, 115200, debug=False)

wifi = adafruit_espatcontrol_wifimanager.ESPAT_WiFiManager(esp, secrets)

# Initialize on-board LED.
led = digitalio.DigitalInOut(GP25)
led.direction = digitalio.Direction.OUTPUT

# Define callback functions which will be called when certain events happen.
# pylint: disable=unused-argument
def connected(client):
    # Connected function will be called when the client is connected to Adafruit IO.
    print("Connected to Adafruit IO! ")

def subscribe(client, userdata, topic, granted_qos):
    # This method is called when the client subscribes to a new feed.
    print("Subscribed to {0} with QOS level {1}".format(topic, granted_qos))

# pylint: disable=unused-argument
def disconnected(client):
    # Disconnected function will be called when the client disconnects.
    print("Disconnected from Adafruit IO!")

def on_led_msg(client, topic, message):
    # Method called whenever user/feeds/led has a new value
    print("New message on topic {0}: {1} ".format(topic, message))
    if message == "ON":
        led.value = True
    elif message == "OFF":
        led.value = False
    else:
        print("Unexpected message on LED feed.")

# Connect to WiFi
print("Connecting to WiFi...")
wifi.connect()
print("Connected!")

MQTT.set_socket(socket, esp)

# Initialize a new MQTT Client object
mqtt_client = MQTT.MQTT(
    broker="io.adafruit.com",
    username=secrets["aio_username"],
    password=secrets["aio_key"],
)

# Initialize an Adafruit IO MQTT Client
io = IO_MQTT(mqtt_client)

# Connect the callback methods defined above to Adafruit IO
io.on_connect = connected
io.on_disconnect = disconnected
io.on_subscribe = subscribe

# Set up a callback for the led feed
io.add_feed_callback("led", on_led_msg)

# Connect to Adafruit IO
print("Connecting to Adafruit IO...")
io.connect()

# Subscribe to all messages on the led feed
io.subscribe("led")

prv_refresh_time = 0.0



## - Functions - ##
#Get Offset Calibration
def Get_Calibration(_mpu):
  _OffsetX, _OffsetY, _OffsetZ = _mpu.acceleration
  io.publish("bmh-offsetx",_OffsetX)
  io.publish("bmh-offsety",_OffsetY)
  io.publish("bmh-offsetz",_OffsetZ)
  return
#Get Polynominal
def Get_Poly(_tilt): 
  #test value
  poly1 = 0.8664121875066527
  poly2 = 0.009562713003442422 * _tilt
  poly3 = -0.00018408281752418392 *_tilt*_tilt 
  poly4 = +0.0000013556331354515971 *_tilt*_tilt*_tilt
  return poly1 + poly2 + poly3 + poly4

#Temperature Scale
def Get_TMP_Scale(t, ts) :
  if "C" in ts :
    return t
  elif "F" in ts :
    return (1.8 * t + 32)
  elif "K" in ts:
    return t + 273.15
  else :
    return t #Default Celcius

#Gravity Scale
def Get_GRV_Scale(g, gr) :
  if "SG" in gr :
    return g
  elif "BR" in gr :
    gc = (g-1)/0.004
    return gc
  elif "PL" in gr :
    gc = 259-(259/g)
    return gc
  else :
    return g  #Default SG

#Calculate tilt 
def calcTilt(_mpu6050):
  _mpu6050.cycle = True
  ax, ay, az = _mpu6050.acceleration
  if ax == 0 and ay == 0 and az == 0:
    return 0
  else :
    _atilt = acos(abs(az) / (sqrt(ax * ax + ay * ay + az * az))) * 180.0 / pi
    return _atilt
  

 

#Get Temperature
def Get_Temp(): 
  temp = ds18.temperature
  return temp


#Config look-up LATER IN DEV

## - Program - ##

while True:
  try:
      io.loop()
  except (ValueError, RuntimeError) as e:
      print("Failed to get data, retrying\n", e)
      wifi.reset()
      io.reconnect()
      continue
  if (time.monotonic() - prv_refresh_time) > 30:
    tilt = calcTilt(mpu6050)
    print("{:6.2f}deg".format(tilt))
    gravity = Get_Poly(tilt) #Calibration Settings 
    print("{:6.4f} {}".format(gravity, gravity_scale))
    print("{:2.2f} Celcius".format(Get_Temp()))
    io.publish("brewmate-hydrometer.bmh-temperature", Get_Temp())
    io.publish("brewmate-hydrometer.bmh-gravity", gravity)
    


    prv_refresh_time = time.monotonic()




 