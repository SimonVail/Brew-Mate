import time
import busio
from adafruit_onewire.bus import OneWireBus
from math import *
import board
from digitalio import DigitalInOut

#import wifi and MQTT
import ipaddress
import ssl
import wifi
import socketpool
import adafruit_requests
import neopixel
import adafruit_minimqtt.adafruit_minimqtt as MQTT
from adafruit_io.adafruit_io import IO_MQTT

#import sensors
import adafruit_mpu6050
from adafruit_ds18x20 import DS18X20

#Import Memory test
import espidf
import gc

#Memory test
print("cp mem free", gc.mem_free())
print("idf total mem", espidf.heap_caps_get_total_size())
print("idf mem free", espidf.heap_caps_get_free_size())
print("idf largest block", espidf.heap_caps_get_largest_free_block())

#get bus
#i2c = busio.I2C(board.IO19, board.IO18)
#ow_bus = OneWireBus(board.IO11)
#Set sensor class
#mpu = adafruit_mpu6050.MPU6050(i2c, address = 0x68)
#ds18 = DS18X20(ow_bus, ow_bus.scan()[0])

#import secrets and configs files. 
# Get wifi details and more from a secrets.py file
try:
    from secrets import secrets
except ImportError:
    print("All secret keys are kept in secrets.py, please add them there!")
    raise

#Connect to internet
print("ESP32-C3 WebClient Test")

print("My MAC addr:", [hex(i) for i in wifi.radio.mac_address])

print("Available WiFi networks:")
for network in wifi.radio.start_scanning_networks():
    print("\t%s\t\tRSSI: %d\tChannel: %d" % (str(network.ssid, "utf-8"),
            network.rssi, network.channel))
wifi.radio.stop_scanning_networks()

print("Connecting to %s"%secrets["ssid"])
wifi.radio.connect(secrets["ssid"], secrets["password"])
print("Connected to %s!"%secrets["ssid"])
print("My IP address is", wifi.radio.ipv4_address)

ipv4 = ipaddress.ip_address("8.8.4.4")
print("Ping google.com: %f ms" % (wifi.radio.ping(ipv4)*1000))

pool = socketpool.SocketPool(wifi.radio)
requests = adafruit_requests.Session(pool, ssl.create_default_context())

# Initialize on-board LED.
neo_pin = board.NEOPIXEL
num_pix = 1
ORDER = neopixel.GRB
pixels = neopixel.NeoPixel(neo_pin, num_pix, brightness=0.2, auto_write=False, pixel_order=ORDER)
#Define neopixel Effect
def wheel(pos):
    # Input a value 0 to 255 to get a color value.
    # The colours are a transition r - g - b - back to r.
    if pos < 0 or pos > 255:
        r = g = b = 0
    elif pos < 85:
        r = int(pos * 3)
        g = int(255 - pos * 3)
        b = 0
    elif pos < 170:
        pos -= 85
        r = int(255 - pos * 3)
        g = 0
        b = int(pos * 3)
    else:
        pos -= 170
        r = 0
        g = int(pos * 3)
        b = int(255 - pos * 3)
    return (r, g, b) if ORDER in (neopixel.RGB, neopixel.GRB) else (r, g, b, 0)

def rainbow_cycle(wait):
    for j in range(255):
        for i in range(num_pix):
            pixel_index = (i * 256 // num_pix) + j
            pixels[0] = wheel(pixel_index & 255)
        pixels.show()
        time.sleep(wait)

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
        rainbow_cycle(0.001)
    elif message == "OFF":
        pixels.fill((0, 0, 0))
        pixels.show()
    else:
        print("Unexpected message on LED feed.")

# Initialize a new MQTT Client object
mqtt_client = MQTT.MQTT(
    broker="io.adafruit.com",
    username=secrets["aio_username"],
    password=secrets["aio_key"],)

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
#------------------Math functions --------------------
#Get Temperature
def Get_Temp(_ds18): 
  temp = _ds18.temperature
  return temp
#Calculate tilt 
def calcTilt(_mpu6050, _XYZ):
  _mpu6050.cycle = True
  ax, ay, az = _mpu6050.acceleration
  axo = ax + _XYZ['ox']
  ayo = ay + _XYZ['oy']
  azo = az + _XYZ['oz']
  try:
    _atilt = acos(abs(azo) / (sqrt(axo * axo + ayo * ayo + azo * azo))) * 180.0 / pi
    return _atilt
  except ZeroDivisionError :
    return 0
#Get Polynominal
def Get_Poly(_tilt): 
  #test value
  poly1 = 0.8664121875066527
  poly2 = 0.009562713003442422 * _tilt
  poly3 = -0.00018408281752418392 *_tilt*_tilt 
  poly4 = +0.0000013556331354515971 *_tilt*_tilt*_tilt
  return poly1 + poly2 + poly3 + poly4
#----------------Calibration--------------------------
#Get Offset Calibration
def Get_Calibration(_mpu):
    rx = _mpu._raw_accel_data[0][0]
    ry = _mpu._raw_accel_data[1][0]
    rz = _mpu._raw_accel_data[2][0]
    print("rx {} ry {} rz {}".format(rx,ry,rz))
    _OX, _OY, _OZ = _mpu.acceleration
    print(_mpu.acceleration)
    _OFSXYZ = {'ox': _OX, 'oy': _OY, 'oz': _OZ}
    return _OFSXYZ 
#-----------------------------------------------------
#--------------- Adafruit IO -------------------------
#Get message from MQTT
def message(client, topic, message):
    # This method is called when a topic the client is subscribed to
    # has a new message.
#    global offsetxyz
    if "calibration" in topic and "1" in message:
        print("New message on topic {0}: {1}".format(topic, message))
#        offsetxyz_str = str(Get_Calibration(mpu6050))
#        io.publish("brewmate-hydro-conf.bhmc-offsetxyz", offsetxyz_str)
#        print("Published")
    elif "offsetxyz" in topic :
        print("New message on topic {0}: {1}".format(topic, message))
#        offsetxyz = eval(message)
    elif "tempscale" in topic:
        print("New message on topic {0}: {1}".format(topic, message))
    elif "gravscale" in topic:
        print("New message on topic {0}: {1}".format(topic, message))
    elif "polynominal" in topic:
        print("New message on topic {0}: {1}".format(topic, message))
    elif "timepublish" in topic:
        print("New message on topic {0}: {1}".format(topic, message))

#Subscribe to feeds :: Also configs
io.subscribe("brewmate-hydro-conf.bmhc-calibration")
io.subscribe("brewmate-hydro-conf.bmhc-gravscale")
io.subscribe("brewmate-hydro-conf.bmhc-tempscale")
io.subscribe("brewmate-hydro-conf.bhmc-offsetxyz")
io.subscribe("brewmate-hydro-conf.bmhc-polynominal")
io.subscribe("brewmate-hydro-conf.bmhc-timepublish")
#Feed call back :: Config Calls
io.on_message = message
io.get("brewmate-hydro-conf.bhmc-offsetxyz")
io.on_message = message
io.get("brewmate-hydro-conf.bmhc-calibration")
io.on_message = message
io.get("brewmate-hydro-conf.bmhc-gravscale")
io.on_message = message
io.get("brewmate-hydro-conf.bmhc-tempscale")
io.on_message = message
io.get("brewmate-hydro-conf.bmhc-polynominal")
io.on_message = message
io.get("brewmate-hydro-conf.bmhc-timepublish")
io.on_message = message
## -------------- Program Loop ------------------- ##

while True:
  try:
      io.loop()
  except (ValueError, RuntimeError) as e:
      print("Failed to get data, retrying\n", e)
      wifi.reset()
      io.reconnect()
      continue
  
  io.on_message = message
  if (time.monotonic() - prv_refresh_time) > 30:
#    tilt = calcTilt(mpu, offsetxyz)
#    print("{:6.2f}deg".format(tilt))
#    gravity = Get_Poly(tilt) #Calibration Settings 
#    print("{:6.4f} {}".format(gravity, gravity_scale))
#    print("{:2.2f} Celcius".format(Get_Temp()))
#    io.publish("brewmate-hydrometer.bmh-temperature", Get_Temp())
#    io.publish("brewmate-hydrometer.bmh-gravity", gravity)
    prv_refresh_time = time.monotonic()
