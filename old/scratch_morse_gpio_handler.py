# This code is copyright Simon Walters under GPL v2
# This code is derived from scratch_handler by Thomas Preston
# Version 5dev 11Aug08 Much better looping supplied by Stein @soilandreyes
# and someone else @MCrRaspJam who've name I've forgotton!
# Version 6dev - Moved Allon/AllOff to be processed before single pins :)
# Vesion 7dev - start to tidy up changes
# Vesion 8dev - use gpio-output system and broadcast allon, 1on system
# V0.1 - change to 6 out 2 in and sanitise the code
# V0.2a - use global variable to trace Scratch disconnect
# V0.3a -   Change to Broadcom GPIO numbering for variables
#           Handle pin broadcasts correctly
# V0.4 - add in more gpio name variants that it can handle
# V0.5 - Use Pinon/off as well as high low and also gpio variables can
#       use high/low on/off as well
# V0.7 - Add in MotorA and MotorB (pin11 and 12) PWM control.
# V0.8b - Add in all pins and try to make in/out configurable and fix input bug
# v0.9c Ultrasonic and better handling of motor variable values
# v1.0  Tidy up Ultrasonic 
# V1.0app add in bit pattern broadcasts

from array import *
import threading
import socket
import time
import sys
import struct
import datetime as dt
import re
from scratch_morse_handler import MorseSender, MorseReceiver

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.cleanup()
#GPIO.setup(11,GPIO.OUT)
#GPIO.setup(12,GPIO.OUT)
#GPIO.setup(13,GPIO.OUT)
#GPIO.setup(15,GPIO.OUT)
#GPIO.setup(16,GPIO.OUT)
#GPIO.setup(18,GPIO.OUT)
#GPIO.setup(22,GPIO.IN,pull_up_down=GPIO.PUD_UP)
#GPIO.setup(7,GPIO.IN,pull_up_down=GPIO.PUD_UP)

def isNumeric(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

##def isIntegre(i):
##    import re
##    if not hasattr(isIntegre, '_re'):
##        print "I compile only once. Remove this line when you are confedent in that."
##        isIntegre._re = re.compile(r"[-+]?\d+(\.0*)?$")
##    return isIntegre._re.match(str(i)) is not None


'''
from Tkinter import Tk
from tkSimpleDialog import askstring
root = Tk()
root.withdraw()
'''

PORT = 42001
DEFAULT_HOST = '127.0.0.1'
#HOST = askstring('Scratch Connector', 'IP:')
BUFFER_SIZE = 240 #used to be 100
SOCKET_TIMEOUT = 1

#SCRATCH_SENSOR_NAME_INPUT = (
#    'gpio25',
#    'gpio04'
#)

#SCRATCH_SENSOR_NAME_PINS_INPUT = (
#    'pin22',
#    'pin07'
#)

#SCRATCH_SENSOR_NAME_OUTPUT = (
#    'gpio17',
#    'gpio18',
#    'gpio21',
#    'gpio22',
#    'gpio23',
#    'gpio24'
#)


#Map gpio to real connector P1 Pins
PIN_NUM = array('i',[11,12,13,15,16,18,22,7,8,10,24,26,19,21])
GPIO_NUM = array('i',[17,18,21,22,23,24,25,4,14,15,8,7,10,9])
PIN_USE = array('i',[1,1,1,1,1,1,0,0,0,0,0,0,0,1])
PINS = len(PIN_NUM)
PIN_DICT = {}

for i in range(PINS):
    PIN_DICT[str(PIN_NUM[i])] = (i, PIN_NUM[i], GPIO_NUM[i], PIN_USE[i])
    if (PIN_USE[i] == 1):
        GPIO.setup(PIN_NUM[i],GPIO.OUT)
        print 'pin' , PIN_NUM[i] , ' out'
    else:
        GPIO.setup(PIN_NUM[i],GPIO.IN,pull_up_down=GPIO.PUD_UP)
        print 'pin' , PIN_NUM[i] , ' in'
GPIO.setup(23,GPIO.OUT)

def pinValue(value):
    if value in ['1', 'on', 'high']:
        return 1
    else:
        return 0

def pinIndex(physical_pin):
    physical_pin = str(physical_pin)
    if PIN_DICT.has_key(physical_pin):
        (i, physical_pin, gpio_pin, pin_use) = PIN_DICT[physical_pin]
    else:
        print "Unable to find physical pin '%s'" % physical_pin
        i = -1
    return i

def gpioPin(physical_pin):
    physical_pin = str(physical_pin)
    if PIN_DICT.has_key(physical_pin):
        (i, physical_pin, gpio_pin, pin_use) = PIN_DICT[physical_pin]
    else:
        i = -1
    return gpio_pin

def pinUse(physical_pin):
    physical_pin = str(physical_pin)
    if PIN_DICT.has_key(physical_pin):
        (i, physical_pin, gpio_pin, pin_use) = PIN_DICT[physical_pin]
    else:
        i = -1
    return pin_use
        
#GPIO_PIN_OUTPUT = array('i')
#GPIO_PIN_INPUT = array('i')
#print "Output Pins are:"
#for i in range(0,len(SCRATCH_SENSOR_NAME_OUTPUT)):
#    print GPIO_PINS[i]
#    GPIO_PIN_OUTPUT.append(GPIO_PINS[i])
#    GPIO.output(GPIO_PINS[i], 0)
#print "Input Pins are:" 
#for i in range(len(SCRATCH_SENSOR_NAME_OUTPUT),8):
#    print GPIO_PINS[i]
#    GPIO_PIN_INPUT.append(GPIO_PINS[i])



class MyError(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)

class ScratchSender(threading.Thread):
    def __init__(self, socket):
        global morse_receiver
        
        threading.Thread.__init__(self)
        self.scratch_socket = socket
        self._stop = threading.Event()
        morse_receiver = MorseReceiver(self.send_morse_message_update, self.send_morse_word_update)

    def __del__(self):
        global morse_receiver
        morse_receiver = 0
    
    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        last_bit_pattern=0L
        for i in range(PINS):
            #print 'i %d' % i
            #print 'GPIO PIN %d' % GPIO_PIN_INPUT[i]
            if (PIN_USE[i] == 0):
                last_bit_pattern += GPIO.input(PIN_NUM[i]) << i
            #else:
                #last_bit_pattern += 1 << i
            #print 'lbp %s' % bin(last_bit_pattern)

        last_bit_pattern = last_bit_pattern ^ -1

        while not self.stopped():
            time.sleep(0.01) # be kind to cpu - not certain why :)
            pin_bit_pattern = 0L
            for i in range(PINS):
                if (PIN_USE[i] == 0):
                    #print 'pin' , PIN_NUM[i]
                    pin_bit_pattern += GPIO.input(PIN_NUM[i]) << i
                #else:
                    #pin_bit_pattern += 1 << i
            #print bin(pin_bit_pattern)
            # if there is a change in the input pins
            changed_pins = pin_bit_pattern ^ last_bit_pattern
            #print "changed pins" , bin(changed_pins)
            if changed_pins:
                #print 'pin bit pattern %d' % pin_bit_pattern

                try:
                    self.broadcast_changed_pins(changed_pins, pin_bit_pattern)
                except Exception as e:
                    print e
                    break

            last_bit_pattern = pin_bit_pattern
            

    def broadcast_changed_pins(self, changed_pin_map, pin_value_map):
        for i in range(PINS):
            # if we care about this pin's value
            if (changed_pin_map >> i) & 0b1:
                pin_value = (pin_value_map >> i) & 0b1
                if (PIN_USE[i] == 0):
                    self.broadcast_pin_update(i, pin_value)

    def broadcast_pin_update(self, pin_index, value):
        global morse_receiver, morse_input_pin
        
        #sensor_name = "gpio" + str(GPIO_NUM[pin_index])
        #bcast_str = 'sensor-update "%s" %d' % (sensor_name, value)
        #print 'sending: %s' % bcast_str
        #self.send_scratch_command(bcast_str)
        sensor_name = "pin" + str(PIN_NUM[pin_index])
        bcast_str = 'sensor-update "%s" %d' % (sensor_name, value)
        # print 'sending: %s' % bcast_str
        self.send_scratch_command(bcast_str)
        if pin_index == morse_input_pin:
            morse_receiver.update(value)

    def send_scratch_command(self, cmd):
        n = len(cmd)
        a = array('c')
        a.append(chr((n >> 24) & 0xFF))
        a.append(chr((n >> 16) & 0xFF))
        a.append(chr((n >>  8) & 0xFF))
        a.append(chr(n & 0xFF))
        self.scratch_socket.send(a.tostring() + cmd)

    def send_morse_message_update(self, value):
        self.send_scratch_command( 'sensor-update "morse_message" "%s"' % value )
        self.send_scratch_command( 'broadcast "morse_receive"' )

    def send_morse_word_update(self, value):
        self.send_scratch_command( 'sensor-update "morse_word" "%s"' % value )
        self.send_scratch_command( 'broadcast "morse_word"' )


class ScratchListener(threading.Thread):
    def __init__(self, socket):
        threading.Thread.__init__(self)
        self.scratch_socket = socket
        self._stop = threading.Event()
        self.wordre = re.compile('(?:"(?:""|[^"])*"|[^"\s]+)')
        self.quotere = re.compile('""')
        self.morse_output_pin = -1
        self.morse_message = ""
        pin_update_func = lambda value : self.physical_pin_update( self.morse_output_pin, value )
        self.morse_sender = MorseSender(pin_update_func)
        
    def send_scratch_command(self, cmd):
        n = len(cmd)
        a = array('c')
        a.append(chr((n >> 24) & 0xFF))
        a.append(chr((n >> 16) & 0xFF))
        a.append(chr((n >>  8) & 0xFF))
        a.append(chr(n & 0xFF))
        self.scratch_socket.send(a.tostring() + cmd)

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()
        
    def physical_pin_update(self, pin_index, value):
        if pin_index >= 0 and PIN_USE[pin_index] == 1:
            # print 'setting gpio %d (physical pin %d) to %d' % (GPIO_NUM[pin_index],PIN_NUM[pin_index],value)
            GPIO.output(PIN_NUM[pin_index], value)

    def strip_quotes(self, token):
        if token[0] == '"':
            token = self.quotere.sub('"', token[1:-1])
        return token
            
    def tokenise(self, data):
        return self.wordre.findall(data)
    
    def do_sensor_update(self, sensor, value):
        global motorA, motorB, morse_input_pin, morse_receiver
        
        # print "Sensor update '%s' = '%s'" % (sensor, value)
        if 'allpins' == sensor :
            # Globally set all ports.
            value = pinValue(value)
            for i in range(PINS):
                self.physical_pin_update(i,value)
        elif 'pinpattern' == sensor:
            num_of_bits = PINS
            bit_pattern = ('00000000000000000000000000'+value)[-num_of_bits:]
            j = 0
            for i in range(PINS):
                if PIN_USE[i] == 1:
                    if bit_pattern[-(j+1)] == '0':
                        self.physical_pin_update(i,0)
                    else:
                        self.physical_pin_update(i,1)
                    j = j + 1
        elif 'pin' == sensor[:3] and isNumeric(sensor[3:]):
            # Check for individual port commands.
            physical_pin = sensor[3:]
            i = pinIndex(physical_pin)
            value = pinValue(value)
            self.physical_pin_update(i,value)
        elif 'motora' == sensor:
            # Check for motor commands.
            if isNumeric(value):
                motorA = max(0, min(100, int(value)))
        elif 'motorb' == sensor:
            if isNumeric(value):
                motorB = max(0, min(100, int(value)))
        elif 'morse_output' == sensor:
            self.morse_output_pin = -1
            print "Request to set morse_output_pin pin%s" % value
            if isNumeric(value):
                self.morse_output_pin = pinIndex(value)
                if self.morse_output_pin > 0:
                    print "Set morse output pin to pin%s index %d" % (value, self.morse_output_pin)
        elif 'morse_input' == sensor:
            morse_input_pin = -1
            print "Request to set morse_input_pin pin%s" % value
            if isNumeric(value):
                morse_input_pin = pinIndex(value)
                if morse_input_pin > 0:
                    print "Set morse input pin to pin%s index %d" % (value, morse_input_pin)
        elif 'morse_message' == sensor:
            self.morse_message = value
    
    def do_broadcast(self, sensor):
        global morse_receiver
        
        # print "Broadcast '%s'" % sensor
        if sensor in ['allon', 'allhigh']:
            for i in range(PINS):
                self.physical_pin_update(i,1)
        elif sensor in ['alloff', 'alllow']:
            for i in range(PINS):
                self.physical_pin_update(i,0)
        elif 'pinpattern' == sensor[:10]:
            # print 'Found pinpattern broadcast'
            value = sensor[10:]
            num_of_bits = PINS
            bit_pattern = ('00000000000000000000000000'+value)[-num_of_bits:]
            j = 0
            for i in range(PINS):
                if PIN_USE[i] == 1:
                    if bit_pattern[-(j+1)] == '0':
                        self.physical_pin_update(i,0)
                    else:
                        self.physical_pin_update(i,1)
                    j = j + 1                    
        elif 'pin' == sensor[:3]:
            # Check for individual port commands.
            physical_pin = -1
            for value in ['high', 'low', 'on', 'off']:
                if value == sensor[-len(value):]:
                    physical_pin = sensor[3:-len(value)]
                    value = pinValue(value)
                    break
            i = pinIndex(physical_pin)
            self.physical_pin_update(i,value)
        elif 'morse_send' == sensor:
            # print "Sending message '%s'" % self.morse_message
            self.send_scratch_command('sensor-update "morse_wait" 0')
            self.morse_sender.send_message( self.morse_message )
            self.send_scratch_command('sensor-update "morse_sent" 1')
            self.send_scratch_command('sensor-update "morse_sent" 0')
        elif 'morse_clear' == sensor:
            morse_receiver.set_message('')
        elif 'sonar' == sensor[:5] and isNumeric(sensor[5:]):
            # Check for individual port commands.
            physical_pin = sensor[5:]
            i = pinIndex(physical_pin)
            if (PIN_USE[i] == 0):
                # print "sonar pulse", physical_pin
                GPIO.output(23, True)
                time.sleep(0.00001)
                GPIO.output(23, False)
                t0=dt.datetime.now()
                t1=t0
                while ((GPIO.input(physical_pin)==False) and ((t1-t0).microseconds < 100000)):
                    t1=dt.datetime.now()
                t1=dt.datetime.now()
                t2=t1
                while ((GPIO.input(physical_pin)==True) and ((t2-t1).microseconds < 100000)):
                    t2=dt.datetime.now()
                t2=dt.datetime.now()
                t3=(t2-t1).microseconds
                #print "t3 in secs" , float(t3/1000000.0)
                distance=t3/58
                if (distance < 500):# and (distance > 4):
                    # print'Distance:',distance,'cm'
                    sensor_name = "sonar" + str(physical_pin) 
                    bcast_str = 'sensor-update "%s" %d' % (sensor_name, distance)
                    # print 'sending: %s' % bcast_str
                    self.send_scratch_command(bcast_str)
                else:
                    sensor_name = "sonar" + str(physical_pin) 
                    bcast_str = 'sensor-update "%s" %d' % (sensor_name, 500)
                    # print 'sending: %s' % bcast_str
                    self.send_scratch_command(bcast_str)
    
    def process(self, data):
        command = ''
        sensor = ''
        for token in self.tokenise(data):
            token = self.strip_quotes(token)
            # print "Token read '%s'" % token
            if token in ['sensor-update', 'broadcast']:
                command = token
            elif command == 'broadcast':
                command = ''
                self.do_broadcast(token)
            elif command == 'sensor-update':
                # Sensor + value pairs.
                if sensor:
                    self.do_sensor_update(sensor, token)
                    sensor = ''
                else:
                    sensor = token
            else:
                print "Unrecognised scratch command '%s'" % token
    
    def run(self):
        global cycle_trace
        
        # This is main listening routine.
        while not self.stopped():
            try:
                packed_size = self.scratch_socket.recv(4)
                if len(packed_size) == 0:
                    #This is probably due to client disconnecting
                    #I'd like the program to retry connecting to the client
                    #tell outer loop that Scratch has disconnected
                    if cycle_trace == 'running':
                        cycle_trace = 'disconnected'
                        break
                
                (size,) = struct.unpack("!I", packed_size)
                data = self.scratch_socket.recv(size)
                # print 'Length: %d, Data: %s' % (size, data)

            except socket.timeout:
                # print "No data received: socket timeout"
                continue
            except:
                if cycle_trace == 'running':
                    cycle_trace = 'disconnected'
                    break

            self.process(data)

            if 'stop handler' in data:
                cleanup_threads((listener, sender))
                sys.exit()
            #else:
                # print 'received something: %s' % data


def create_socket(host, port):
    for attempts in range(0,4):
        try:
            print 'Trying'
            scratch_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            scratch_sock.connect((host, port))
            break
        except socket.error:
            print "There was an error connecting to Scratch!"
            print "I couldn't find a Mesh session at host: %s, port: %s" % (host, port) 
            time.sleep(3)
            
            if attempts == 3:
                sys.exit(1)

    return scratch_sock

def cleanup_threads(threads):
    for thread in threads:
        thread.stop()

    for thread in threads:
        thread.join()

if __name__ == '__main__':
    if len(sys.argv) > 1:
        host = sys.argv[1]
    else:
        host = DEFAULT_HOST

morse_receiver = 0
morse_input_pin = -1
cycle_trace = 'start'
motorA = 0;
motorB = 0;
motor_timing = array('i',[0,0,100])
motor_order = array('i',[0,1])
while True:

    if (cycle_trace == 'disconnected'):
        print "Scratch disconnected"
        cleanup_threads((listener, sender))
        time.sleep(1)
        cycle_trace = 'start'

    if (cycle_trace == 'start'):
        # open the socket
        print 'Starting to connect...' ,
        the_socket = create_socket(host, PORT)
        print 'Connected!'
        the_socket.settimeout(SOCKET_TIMEOUT)
        listener = ScratchListener(the_socket)
##        data = the_socket.recv(BUFFER_SIZE)
##        print "Discard 1st data buffer" , data[4:].lower()
        sender = ScratchSender(the_socket)
        cycle_trace = 'running'
        print "Running...."
        listener.start()
        sender.start()

    # wait for ctrl+c
    try:
        #just pause
        #print "motorA val:" , motorA

        if ((motorA > 0) or (motorB > 0)):
            if (motorA > motorB):
                motor_order[0]=0
                motor_order[1]=1
                #time before motorB goes off
                motor_timing[0]=motorB
                motor_timing[1]=motorA-motorB
                motor_timing[2]=100-motorA
            else:
                motor_order[0]=1
                motor_order[1]=0
                #time before motorA goes off
                motor_timing[0]=motorA
                motor_timing[1]=motorB-motorA
                motor_timing[2]=100-motorB


            #print 't0 t1 t2', motor_timing[0], motor_timing[1] , motor_timing[2]                
            #print 'pin: ' , GPIO_PINS[0]
            GPIO.output(PIN_NUM[motor_order[0]], 1)
            if (motor_timing[0] > 0 ):
                #print 'pin: ' , PIN_NUM[motor_order[1]]
                GPIO.output(PIN_NUM[motor_order[1]], 1)
                time.sleep(motor_timing[0]/10000.0)
            if (motor_timing[0] > 0 ):
                GPIO.output(PIN_NUM[motor_order[1]], 0)
            time.sleep(motor_timing[1]/10000.0)
            GPIO.output(PIN_NUM[motor_order[0]], 0)
            time.sleep(motor_timing[2]/10000.0)            
        else:
            time.sleep(0.1)
    except KeyboardInterrupt:
        cleanup_threads((listener,sender))
        sys.exit()

