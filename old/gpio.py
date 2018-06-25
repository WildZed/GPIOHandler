# This code is copyright Tom Lucas under GPL v2.
# This code is derived from scratch_gpio_handler by Simon Walters.
# This code is derived from scratch_handler by Thomas Preston.
#
# V5dev     - 11Aug08 Much better looping supplied by Stein @soilandreyes
#             and someone else @MCrRaspJam who've name I've forgotton!
# V6dev     - Moved Allon/AllOff to be processed before single pins:-)
# V7dev     - Start to tidy up changes.
# V8dev     - Use gpio-output system and broadcast allon, 1on system.
# V0.1      - Change to 6 out 2 in and sanitise the code.
# V0.2a     - Use global variable to trace Scratch disconnect.
# V0.3a     - Change to Broadcom GPIO numbering for variables.
#             Handle pin broadcasts correctly.
# V0.4      - Add in more gpio name variants that it can handle.
# V0.5      - Use Pinon/off as well as high low and also gpio variables can
#             use high/low on/off as well.
# V0.7      - Add in MotorA and MotorB ( pin11 and 12 ) PWM control.
# V0.8b     - Add in all pins and try to make in/out configurable and fix input bug.
# v0.9c     - Ultrasonic and better handling of motor variable values
# v1.0      - Tidy up Ultrasonic .
# V1.0app   - Add in bit pattern broadcasts.
# V2.0      - Complete refactor and re-design. Now has server modes as well as Scratch client mode.

from array import *
import traceback
import threading
import socket
import time
import sys
import struct
import datetime as dt
import re
import SocketServer
from morse import MorseSender, MorseReceiver

try:
    import RPi.GPIO as GPIO
    HAVE_GPIO = True
except:
    HAVE_GPIO = False


LOCAL_HOST = '127.0.0.1'
SERVER_HOST = '0.0.0.0'
# HOST_IP = '192.168.1.40'
DEFAULT_PORT = 3142
DEFAULT_SCRATCH_PORT = 42001
BUFFER_SIZE = 240 #used to be 100
CLIENT_SOCKET_TIMEOUT = 8
DEFAULT_SERVER_SOCKET_TIMEOUT = 8

'''
from Tkinter import Tk
from tkSimpleDialog import askstring
root = Tk()
root.withdraw()
HOST = askstring( 'Scratch Connector', 'IP:' )
'''


# Generic utility functions: 
def isNumeric( str ):
    try:
        float( str )
    except ValueError:
        return False
    
    return True


class GenericException( Exception ):
    def __init__( self, value ):
        self.value = value

    def __str__( self ):
        return repr( self.value )

def printException():
    exceptionInfo = sys.exc_info()
    print "Exception caught:", exceptionInfo[0]
    # print exceptionInfo[1]
    traceback.print_tb( exceptionInfo[2] )


# Single use class to handle the GPIO on a Raspberry PI.                
class GPIOController:
    def __init__( self ):
        # Map GPIO to real connector P1 pins.
        # Physical pin numbers.
        self.PIN_NUM = array( 'i', [ 11, 12, 13, 15, 16, 18, 22, 7, 8, 10, 24, 26, 19, 21 ] )
        # GPIO pin numbers.
        self.GPIO_NUM = array( 'i', [ 17, 18, 21, 22, 23, 24, 25, 4, 14, 15, 8, 7, 10, 9 ] )
        # Default programming of pins: 1 for output, 0 for input.
        self.PIN_USE = array( 'i', [ 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1 ] )
        self.PINS = len( self.PIN_NUM )
        self.PIN_DICT = {}
    
        self.motorA = 0
        self.motorB = 0
        self.motorTiming = array( 'i', [ 0, 0, 100 ] )
        self.motorOrder = array( 'i', [ 0, 1 ] )
        
        self.setupGPIOPinDict()
        self.setupGPIO()
        self.setupGPIOPins()

    # def __del__( self ):
    #     GPIOController.instance = None
    
    # @classmethod
    def setupGPIOPinDict( self ):
        if not self.PIN_DICT:
            for ii in range( self.PINS ):
                self.PIN_DICT[str( self.PIN_NUM[ii] )] = ( ii, self.PIN_NUM[ii], self.GPIO_NUM[ii], self.PIN_USE[ii] )
    
    def setupGPIO( self ):
        if not HAVE_GPIO:
            return

        GPIO.setmode( GPIO.BOARD )
        GPIO.setwarnings( False )
        GPIO.cleanup()
    
    def setupGPIOPins( self ):
        if not HAVE_GPIO:
            return

        for ii in range( self.PINS ):
            if self.PIN_USE[ii] == 1:
                GPIO.setup( self.PIN_NUM[ii], GPIO.OUT )
                print 'pin' , self.PIN_NUM[ii] , ' out'
            else:
                GPIO.setup( self.PIN_NUM[ii], GPIO.IN, pull_up_down=GPIO.PUD_UP )
                print 'pin' , self.PIN_NUM[ii] , ' in'
        
        # GPIO.setup( 11, GPIO.OUT )
        # GPIO.setup( 12, GPIO.OUT )
        # GPIO.setup( 13, GPIO.OUT )
        # GPIO.setup( 15, GPIO.OUT )
        # GPIO.setup( 16, GPIO.OUT )
        # GPIO.setup( 18, GPIO.OUT )
        # GPIO.setup( 22, GPIO.IN, pull_up_down=GPIO.PUD_UP )
        # GPIO.setup( 7, GPIO.IN, pull_up_down=GPIO.PUD_UP )
        GPIO.setup( 23, GPIO.OUT )

    def pinIndex( self, pin ):
        pin = str( pin )
        
        if self.PIN_DICT.has_key( pin ):
            ( pinIndex, pin, gpioPin, pinUse ) = self.PIN_DICT[pin]
        else:
            print "Unable to find physical pin '%s'." % pin
            pinIndex = -1
        
        return pinIndex

    # Convert physical pin to GPIO pin.
    def gpioPin( self, pin ):
        pin = str( pin )
        
        if self.PIN_DICT.has_key( pin ):
            ( pinIndex, pin, gpioPin, pinUse ) = self.PIN_DICT[pin]
        else:
            gpioPin = -1
        
        return gpioPin

    def pinByIndex( self, pinIndex ):
        return self.PIN_NUM[pinIndex]

    def pinNameByIndex( self, pinIndex ):
        return str( self.pinByIndex( pinIndex ) )
    
    # Check if physical pin is in use.
    def pinUse( self, pin ):
        pin = str( pin )
        
        if self.PIN_DICT.has_key( pin ):
            ( pinIndex, pin, gpioPin, pinUse ) = self.PIN_DICT[pin]
        else:
            pinUse = 0
        
        return pinUse
 
    def pinIndexUpdate( self, pinIndex, value ):
        if not HAVE_GPIO:
            return

        if pinIndex >= 0 and self.PIN_USE[pinIndex] == 1:
            # print 'setting gpio %d ( physical pin %d ) to %d' % ( GPIO_NUM[pinIndex],PIN_NUM[pinIndex],value )
            GPIO.output( self.PIN_NUM[pinIndex], value )
   
    def pinUpdate( self, pin, value ):
        pinIndex = self.pinIndex( pin )
        self.pinIndexUpdate( pinIndex, value )
   
    def allPinUpdate( self, value ):
        for ii in range( self.PINS ):
            self.pinIndexUpdate( ii, value )
    
    def allPinUpdatePattern( self, value ):
        numBits = self.PINS
        bitPattern = ( '00000000000000000000000000' + value )[-numBits:]
        jj = 0
        
        for ii in range( self.PINS ):
            if self.PIN_USE[ii] == 1:
                if bitPattern[-( jj+1 )] == '0':
                    self.pinIndexUpdate( ii, 0 )
                else:
                    self.pinIndexUpdate( ii, 1 )
                jj = jj + 1
    
    def getAllPinsBitPattern( self ):
        bitPattern = 0L
        
        if not HAVE_GPIO:
            return bitPattern
        
        for ii in range( self.PINS ):
            # print 'ii %d' % ii
            # print 'GPIO PIN %d' % gpioPin_INPUT[ii]
            if self.PIN_USE[ii] == 0:
                bitPattern += GPIO.input( self.PIN_NUM[ii] ) << ii
            # else:
            #     bitPattern += 1 << ii
            # print 'lbp %s' % bin( bitPattern )
        
        return bitPattern
        
    def forAllChangedPins( self, changedPinMap, pinValueMap, changePinFunc ):
        for ii in range( self.PINS ):
            # If we care about this pin's value, then call the
            # callback function to tell the caller it has changed.
            if ( changedPinMap >> ii ) & 0b1:
                pinValue = ( pinValueMap >> ii ) & 0b1
                
                # print "Changed pin index %d to value %d." % ( ii, pinValue )
                
                if self.PIN_USE[ii] == 0:
                    changePinFunc( ii, pinValue )

    def sonarPulse( self, pin ):
        if not HAVE_GPIO:
            return

        # print "sonar pulse", pin
        GPIO.output( 23, True )
        time.sleep( 0.00001 )
        GPIO.output( 23, False )
        
        t0 = dt.datetime.now()
        t1 = t0
        
        while GPIO.input( pin ) == False and ( t1 - t0 ).microseconds < 100000:
            t1 = dt.datetime.now()
        
        t1=dt.datetime.now()
        t2=t1
        
        while GPIO.input( pin ) == True and ( t2 - t1 ).microseconds < 100000:
            t2 = dt.datetime.now()
        
        t2 = dt.datetime.now()
        t3 = ( t2-t1 ).microseconds
        # print "t3 in secs" , float( t3/1000000.0 )
        distance = t3 / 58
        
        return distance
    
    def updateMotor( self, motor, value ) :
        if isNumeric( value ):
            if 'A' == motor:
                self.motorA = max( 0, min( 100, int( value )) )
            elif 'B' == motor:
                self.motorB = max( 0, min( 100, int( value )) )
                    
    def updateMotors( self ):
        updated = False
        
        if HAVE_GPIO and ( self.motorA > 0 or self.motorB > 0 ):
            if self.motorA > self.motorB:
                self.motorOrder[0] = 0
                self.motorOrder[1] = 1
                # Time before motorB goes off.
                self.motorTiming[0] = self.motorB
                self.motorTiming[1] = self.motorA-self.motorB
                self.motorTiming[2] = 100 - self.motorA
            else:
                self.motorOrder[0] = 1
                self.motorOrder[1] = 0
                # Time before motorA goes off.
                self.motorTiming[0] = self.motorA
                self.motorTiming[1] = self.motorB-self.motorA
                self.motorTiming[2] = 100 - self.motorB

            # print 't0 t1 t2', motorTiming[0], motorTiming[1] , motorTiming[2]                
            # print 'pin: ' , gpioPinS[0]
            GPIO.output( self.PIN_NUM[motorOrder[0]], 1 )
            
            if self.motorTiming[0] > 0:
                # print 'pin: ' , PIN_NUM[motorOrder[1]]
                GPIO.output( self.PIN_NUM[motorOrder[1]], 1 )
                time.sleep( self.motorTiming[0] / 10000.0 )
            
            if motorTiming[0] > 0:
                GPIO.output( self.PIN_NUM[motorOrder[1]], 0 )
            
            time.sleep( self.motorTiming[1] / 10000.0 )
            GPIO.output( self.PIN_NUM[motorOrder[0]], 0 )
            time.sleep( self.motorTiming[2] / 10000.0 )
            
            updated = True
        
        return updated

        
class ProtocolSenderThread( threading.Thread ):
    def __init__( self, protocol ):
        threading.Thread.__init__( self )
        self.protocol = protocol
        self._stop = threading.Event()
    
    def stop( self ):
        self._stop.set()

    def stopped( self ):
        return self._stop.isSet()

    def run( self ):
        # Initialise the protocol processor for sending.
        self.protocol.initialise()

        # Ask the processor to send updates until it tells us to stop.
        while not self.stopped() and self.protocol.sendUpdate():
            time.sleep( 0.01 ) # Be kind to CPU - not certain why:-)


class ProtocolListenerThread( threading.Thread ):
    def __init__( self, protocol ):
        threading.Thread.__init__( self )
        self.protocol = protocol
        self._stop = threading.Event()
 
    def stop( self ):
        self._stop.set()

    def stopped( self ):
        return self._stop.isSet()
    
    def run( self ):
        # This is main listening routine.
        # The process() does a read() which blocks while waiting for messages.
        while not self.stopped() and self.protocol.process():
            pass


class BaseProtocol():
    WORD_RE = re.compile( '(?:"(?:""|[^"])*"|[^"\s]+)' )
    QUOTE_RE = re.compile( '""' )
    
    def __init__( self ):
        self.listener = None
        self.sender = None
    
    def __del__( self ):
        BaseProtocol.stop( self )
 
    def stripQuotes( self, token ):
        if token[0] == '"':
            token = self.QUOTE_RE.sub( '"', token[1:-1] )
        
        return token
            
    def tokenise( self, data ):
        return self.WORD_RE.findall( data )

    # Initialise the protocol for sending.
    # Called by sender thread to initialise before sending updates.
    def initialise( self ):
        raise GenericException( "Pure virtual method called!" )       
    
    # Create a protocol message.
    def create( self, message ):
        return message
    
    # Send a protocol message.
    def send( self, message ):
        raise GenericException( "Pure virtual method called!" )

    # Receive a given number of bytes.
    def readBytes( self, size ):
        raise GenericException( "Pure virtual method called!" )
        
        return None
    
    # Read one protocol message.
    def read( self ):
        return self.readBytes( 1024 )

    def start( self ):
        self.listener = ProtocolListenerThread( self )
        self.sender = ProtocolSenderThread( self )
        self.listener.start()
        self.sender.start()

    def stop( self ):
        if self.listener:
            self.listener.stop()
        
        if self.sender:
            self.sender.stop()
 
        if self.listener:
            self.listener.join()
        
        if self.sender:
            self.sender.join()
        
        self.listener = self.sender = None

    def stopped( self ):
        if self.listener and self.sender:
            if not self.listener.stopped() and not self.sender.stopped():
                return False
        
        return True
        
    # Send an update of the protocol processor state to the remote host.
    # Called by sender thread to send an update after a time.
    # If False is returned, the sender thread will stop.
    def sendUpdate( self ):
        raise GenericException( "Pure virtual method called!" )

    # Update the protocol processor state.
    # Called by protocol handler loop to update its state.
    # Returns True if something was updated.
    def update( self ):
        raise GenericException( "Pure virtual method called!" )
        
        return False
    
    # Called from the listener thread to process an incoming message.
    # Reads and processes a protocol message.
    # Normally calls self.read() to get one incoming message.
    # Will enter a wait state while waiting for a message.
    # Returns False if the processing should stop.
    def process( self ):
        raise GenericException( "Pure virtual method called!" )
        
        message = self.read()
    
        if message == None:
            return False
        
        return True

        
class GPIOScratchProtocol( BaseProtocol ):
    def __init__( self ):
        BaseProtocol.__init__( self )
        
        self.gpio = GPIOController()
        self.lastPinBitPattern = 0
        self.morseInputPin = -1
        self.morseOutputPin = -1
        self.morseMessage = ""
        # Morse receiver, receives morse blip updates, converts into a message
        # and sends using the message update and word update functions.
        # In this case the state of the morseInputPin is read
        # and the MorseReceiver update function is called.
        self.morseReceiver = MorseReceiver( self.sendMorseMessageUpdate, self.sendMorseWordUpdate )
        # Morse sender, sends a morse message using the update function.
        # In this case it sends it to the GPIO morseOutputPin.
        self.morseSender = MorseSender( self.morseOutputPinUpdate )

    def create( self, message ):
        # Create a 4 byte integer for the length of the message.
        # size = len( message )
        # ary = array( 'c' )
        # ary.append( chr( (size >> 24 ) & 0xFF ))
        # ary.append( chr( (size >> 16 ) & 0xFF ))
        # ary.append( chr( (size >>  8 ) & 0xFF ))
        # ary.append( chr( size & 0xFF ))
        # ary.tostring()
        
        # Create a 4 byte integer for the length of the message.
        packedLen = struct.pack( "!I", len( message ) )
        
        return "{}{}".format( packedLen, message )
    
    # The inheriting class must call this to read a message.
    # It must override readChars() to read characters from a message stream of some kind.
    def read( self ):
        # Read from the TCP socket connected to the client.
        packedLen = self.readBytes( 4 )
        
        if packedLen:
            if len( packedLen ) != 4:
                raise GenericException( "Unable to read message size." )

            ( size, ) = struct.unpack( "!I", packedLen )

            return self.readBytes( size )
        else:
            return None
   
    def pinValue( self, value ):
        if value in [ '1', 'on', 'high' ]:
            return 1
        else:
            return 0
 
    def morseOutputPinUpdate( self, value ):
        self.gpio.pinUpdate( self.morseOutputPin, value )
 
    def sendMorseMessageUpdate( self, value ):
        self.send( 'sensor-update "morse_message" "%s"' % value )
        self.send( 'broadcast "morse_receive"' )

    def sendMorseWordUpdate( self, value ):
        self.send( 'sensor-update "morse_word" "%s"' % value )
        self.send( 'broadcast "morse_word"' )
 
    def broadcastPinUpdate( self, pinIndex, value ):
        pin = self.gpio.pinByIndex( pinIndex )
        sensorName = 'pin' + self.gpio.pinNameByIndex( pinIndex )
        broadcastStr = 'sensor-update "%s" %d' % ( sensorName, value )
        # print 'Sending: %s' % broadcastStr
        self.send( broadcastStr )
        print "Pin = '%s', morseInputPin = '%s'." % ( pin, self.morseInputPin )
        
        if pin == self.morseInputPin:
            self.morseReceiver.update( value )

    def broadcastChangedPins( self, changedPinMap, pinValueMap ):
        self.gpio.forAllChangedPins( changedPinMap, pinValueMap, self.broadcastPinUpdate )
  
    def doSensorUpdate( self, sensor, value ):
        # print "Sensor update '%s' = '%s'" % ( sensor, value )
        if 'allpins' == sensor:
            # Globally set all ports.
            self.gpio.allPinUpdate( self.pinValue( value ) )
        elif 'pinpattern' == sensor:
            self.gpio.allPinUpdatePattern( value )
        elif 'pin' == sensor[:3] and isNumeric( sensor[3:] ):
            # Check for individual port commands.
            pin = sensor[3:]
            value = self.pinValue( value )
            self.gpio.pinUpdate( pin, value )
        elif 'motora' == sensor:
            # Check for motor commands.
            self.gpio.updateMotor( 'A', value )
        elif 'motorb' == sensor:
            self.gpio.updateMotor( 'B', value )
        elif 'morse_output' == sensor:
            self.morseOutputPin = -1
            print "Request to set morse output pin to pin%s." % value
            
            if isNumeric( value ):
                self.morseOutputPin = value
                
                if self.morseOutputPin > 0:
                    print "Set morse output pin to pin%s index %d." % ( self.morseOutputPin, self.gpio.pinIndex( value ) )
        elif 'morse_input' == sensor:
            self.morseInputPin = -1
            print "Request to set morse input pin to pin%s." % value
            
            if isNumeric( value ):
                self.morseInputPin = value
                
                if self.morseInputPin > 0:
                    print "Set morse input pin to pin%s index %d." % ( self.morseInputPin, self.gpio.pinIndex( value ) )
        elif 'morse_message' == sensor:
            self.morseMessage = value
        else:
            print "Unrecognised sensor '%s'. Value is '%s'." % ( sensor, value )
    
    def doBroadcast( self, sensor ):
        # print "Broadcast '%s'" % sensor
        if sensor in ['allon', 'allhigh']:
            self.gpio.allPinUpdate( 1 )
        elif sensor in ['alloff', 'alllow']:
            self.gpio.allPinUpdate( 0 )
        elif 'pinpattern' == sensor[:10]:
            # print 'Found pinpattern broadcast'
            self.gpio.allPinUpdatePattern( sensor[10:] )
        elif 'pin' == sensor[:3]:
            # Check for individual port commands.
            pin = -1
            
            for value in ['high', 'low', 'on', 'off']:
                if value == sensor[-len( value ):]:
                    pin = sensor[3:-len( value )]
                    value = self.pinValue( value )
                    break
            
            self.gpio.pinUpdate( pin, value )
        elif 'morse_send' == sensor:
            # print "Sending message '%s'" % self.morseMessage
            self.send( 'sensor-update "morse_wait" 0' )
            self.morseSender.sendMessage( self.morseMessage )
            self.send( 'sensor-update "morse_sent" 1' )
            self.send( 'sensor-update "morse_sent" 0' )
        elif 'morse_clear' == sensor:
            self.morseReceiver.setMessage( '' )
        elif 'sonar' == sensor[:5] and isNumeric( sensor[5:] ):
            # Check for individual port commands.
            pin = sensor[5:]
            
            if self.gpio.pinUse( pin ):
                distance = self.gpio.sonarPulse( pin )
                
                if distance < 500: # and ( distance > 4 ):
                    # print 'Distance:', distance, 'cm'
                    sensorName = 'sonar' + str( pin ) 
                    broadcastStr = 'sensor-update "%s" %d' % ( sensorName, distance )
                    # print 'sending: %s' % broadcastStr
                    self.send( broadcastStr )
                else:
                    sensorName = 'sonar' + str( pin ) 
                    broadcastStr = 'sensor-update "%s" %d' % ( sensorName, 500 )
                    # print 'sending: %s' % broadcastStr
                    self.send( broadcastStr )
    
    # Read the initial state of the GPIO pins.
    # Called by sender thread to initialise before sending updates.
    def initialise( self ):
        self.lastPinBitPattern = self.gpio.getAllPinsBitPattern()
        # Flip the bit pattern, to broadcast all pins first time?
        self.lastPinBitPattern = self.lastPinBitPattern ^ -1
    
    # Send an update of the current state of the GPIO pins.
    # Called by sender thread to send an update after a time.
    # If False is returned, the thread will stop.
    def sendUpdate( self ):
        status = True
        currentPinBitPattern = self.gpio.getAllPinsBitPattern()
        # Get the bit pattern of changed pins.
        changedPins = currentPinBitPattern ^ self.lastPinBitPattern
        self.lastPinBitPattern = currentPinBitPattern
        # print "Changed pins: {}".format( bin( changedPins ) )
        
        if changedPins:
            # print "New pin bit pattern: {}".format( bin( currentPinBitPattern ) )
            
            try:
                self.broadcastChangedPins( changedPins, currentPinBitPattern )
            except:
                printException()
                status = False
        
        return status
    
    # Update the state of the GPIO.
    # Returns True if something was updated.
    def update( self ):
        # Update the state of any motors that are being controlled via GPIO.
        return self.gpio.updateMotors()
    
    # Read and process a message from Scratch.
    # Returns False if the processing should stop.
    def process( self ):
        message = self.read()
    
        if message == None:
            # Allow timeout on read from socket.
            return True

        status = True
        command = ''
        sensor = ''
        
        # print "process: message: ", message
        # print "process: tokenise: ", self.tokenise( message )
        
        for token in self.tokenise( message ):
            token = self.stripQuotes( token )
            # print "Token read '%s'" % token
            
            if token in ( 'sensor-update', 'broadcast', 'stop' ):
                command = token
            elif command == 'broadcast':
                command = ''
                self.doBroadcast( token )
            elif command == 'sensor-update':
                # Sensor + value pairs.
                if sensor:
                    self.doSensorUpdate( sensor, token )
                    sensor = ''
                else:
                    sensor = token
            elif command == 'stop':
                if token == 'handler':
                    # Stop the protocol handler.
                    status = False
            else:
                print "Unrecognised scratch command '%s'" % token
        
        return status


class GPIOScratchClient( GPIOScratchProtocol ):
    MAX_CONNECTION_ATTEMPTS = 3
    
    def __init__( self, host = LOCAL_HOST, port = DEFAULT_SCRATCH_PORT ):
        GPIOScratchProtocol.__init__( self )
        self.state = 'start'
        self.host = host
        self.port = port
        self.socket = None

    def connect( self ):
        print 'Connecting to Scratch server...'
        
        for attempt in range( 1, self.MAX_CONNECTION_ATTEMPTS + 1 ):
            try:
                print 'Connection attempt %s...' % attempt
                self.socket = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
                self.socket.connect( (self.host, self.port ) )
                break
            except socket.error:
                print "There was an error connecting to Scratch!"
                print "Couldn't find a Mesh session at host: %s, port: %s" % ( self.host, self.port ) 
                
                self.socket = None
            
                if attempt < self.MAX_CONNECTION_ATTEMPTS:
                    time.sleep( 3 )

        if self.socket:
            self.socket.settimeout( CLIENT_SOCKET_TIMEOUT )
            self.state = 'connected'
            print 'Connected!'
        
        return self.socket

    def disconnect( self ):
        if self.state in ( 'running', 'connected' ):
            self.state = 'disconnected'
            print "Disconnected."
        
        if self.socket:
            self.socket.close()
            self.socket = None
 
    def start( self ):
        # Start threads.
        GPIOScratchProtocol.start( self )
        self.state = 'running'
        print "Running..."
   
    def stop( self ):
        # Stop threads.
        GPIOScratchProtocol.stop( self )
        # Once we've stopped we're ready to start again.
        self.state = 'start'
        self.disconnect()
        print "Stopped."

    def shutdown( self ):
        self.stop()
        self.state = 'quit'
        print "Shutdown."
   
    # Send a scratch command.
    def send( self, message ):
        try:
            self.socket.sendall( self.create( message ) )
        except socket.error:
            pass
    
    # Read a set number of bytes.
    def readBytes( self, size ):
        if self.socket:
            return self.socket.recv( size )
        else:
            return None
      
    # Read a command from scratch.
    # GPIOScratchProtocol manages when this is called.
    # This is called from the listener thread of the BaseProtocol.
    def read( self ):
        message = None
        
        if self.state == 'running':
            try:
                # Call the protocol read function to read a message.
                message = GPIOScratchProtocol.read( self )
                
                if message:
                    print "Scratch wrote message, length {}:".format( len( message ) )
                    print message
                else:
                    print "Scratch read failed."
            except socket.timeout:
                # print "No Scratch command received: socket timeout."
                pass
            except socket.error:
                printException()
                # Socket was closed from the other end?
                self.disconnect()
            except:
                printException()
                self.disconnect()
        
        return message

    def run( self ):
        # Outer loop to connect the socket, start the listener and sender threads
        # and handle client state and GPIO state.
        while True:
            if self.state == 'quit':
                break
            elif self.state == 'disconnected':
                print "Scratch disconnected."
                self.stop()
            elif self.stopped():
                print "Scratch stopped."
                self.stop()

            # Try to connect and start the Scratch protocol client.
            try:
                if self.state == 'start':
                    # Open the socket.
                    if self.connect():
                        self.start()

                # Update the GPIO state.
                # Make sure we don't have a very busy loop.
                if not self.update():
                    time.sleep( 0.1 )
            except KeyboardInterrupt:
                print "Scratch client quit using keyboard interrupt."
                break
            except:
                printException()
                break
        
        print "Scratch client shutting down..."
        self.shutdown()


class GenericTCPRequestHandler( SocketServer.BaseRequestHandler ):
    # def __init__( self, *args, **kwargs ):
    #     SocketServer.BaseRequestHandler.__init__( self, *args, **kwargs )

    def read( self ):
        # Read from the socket - self.request is the TCP socket connected to the client.
        data = self.request.recv( 1024 )
        size = len( data )
        
        if size > 0:
            # print 'Length: %d, Data: %s' % ( size, data )
            print "{}@{} wrote:".format( self.threadName, self.clientAddress )
            print data
        else:
            # The client probably disconnected.
            data = None
        
        return data
    
    def send( self, message ):
        self.request.sendall( message )

    def handle( self ):
        self.request.settimeout( DEFAULT_SERVER_SOCKET_TIMEOUT )
        self.thread = threading.current_thread()
        self.threadName = self.thread.name
        self.clientAddress = self.client_address[0]
        
        print "Socket connected on thread {} from {}.".format( self.threadName, self.clientAddress )
        
        while True:
            try:
                data = self.read()
                
                if data == None:
                    print "Socket closed."
                else:
                    # Just send back the same data, prefixed by the thread name.
                    response = "{}: {}".format( self.threadName, data )
                    self.send( response )
            except socket.timeout:
                print "No data received on thread {}: Socket timeout.".format( self.threadName )
                
                # Could continue here. Not sure if socket is still open.
                break
            except:
                printException()
                break
     
        print "Socket disconnected to thread {} from {}.".format( self.threadName, self.clientAddress )


class GPIOScratchRequestHandler( SocketServer.BaseRequestHandler, GPIOScratchProtocol ):
    # def __init__( self, *args, **kwargs ):
    #     SocketServer.BaseRequestHandler.__init__( self, *args, **kwargs )

    def setup( self ):
        GPIOScratchProtocol.__init__( self )
        
        self.state = 'start'
        self.request.settimeout( DEFAULT_SERVER_SOCKET_TIMEOUT )
        self.thread = threading.current_thread()
        self.threadName = self.thread.name
        self.clientAddress = self.client_address[0]
    
    def finish( self ):
        self.shutdown()
        print "Scratch server request finished."
 
    def disconnect( self ):
        if self.state in ( 'running', 'connected' ):
            self.state = 'disconnected'
            print "Disconnected."
        
        if self.request:
            self.request.close()

    def start( self ):
        # Start threads.
        GPIOScratchProtocol.start( self )
        self.state = 'running'
        print "Running..."
   
    def stop( self ):
        # Stop threads.
        GPIOScratchProtocol.stop( self )
        self.state = 'start'
        self.disconnect()

    def shutdown( self ):
        self.stop()
        self.state = 'quit'
   
    # Send a scratch command.
    def send( self, message ):
        # socket.sendall( self.create( message ) )
        self.request.sendall( message + "\n" )
    
    # Read a set number of bytes.
    def readBytes( self, size ):
        return self.request.recv( size )

    # This is called from the listener thread of the BaseProtocol.
    def read( self ):
        data = None
        
        try:
            # Read from the socket - self.request is the TCP socket connected to the client.
            data = self.request.recv( 1024 )
            size = len( data )
            
            if size > 0:
                # print 'Length: %d, Data: %s' % ( size, data )
                print "{}@{} wrote:".format( self.threadName, self.clientAddress )
                print data
            else:
                # The client probably disconnected.
                data = None
        except socket.timeout:
            print "Scratch server request socket timed out."
            data = None
        # except KeyboardInterrupt:
            # print "Scratch server request quit using keyboard interrupt on read."
            # self.shutdown()
            # data = None
        except:
            printException()
            data = None
        
        return data

    def handle( self ):
        self.state = 'connected'
        
        print "Socket connected on thread {} from {}.".format( self.threadName, self.clientAddress )
        
        # Start the listener and sender threads.
        self.start()
        
        while True:
            if self.state == 'quit':
                print "Server request quitting."
                break
            elif self.state == 'disconnected':
                print "Client disconnected."
                break
            elif self.stopped():
                print "Client stopped."
                break

            try:
                # Update the GPIO state.
                # Make sure we don't have a very busy loop.
                if not self.update():
                    time.sleep( 0.1 )
            except socket.timeout:
                print "No data received on thread {}: Socket timeout.".format( self.threadName )
                
                # Could continue here. Not sure if socket is still open.
                break
            except KeyboardInterrupt:
                print "Scratch server request quit using keyboard interrupt."
                break
            except:
                printException()
                break
        
        print "Server request shutting down..."
        self.shutdown()
        print "Socket disconnected from thread {} from {}.".format( self.threadName, self.clientAddress )

        
class GPIOThreadedTCPServer( SocketServer.ThreadingMixIn, SocketServer.TCPServer ):
    pass

        
class GenericServer():
    def __init__( self, port = DEFAULT_PORT, handler = GenericTCPRequestHandler ):
        # Server host always be SERVER_HOST. 
        self.host = SERVER_HOST
        # Server port.
        self.port = port
        self.handler = handler
        self.socket = None
        self.server = None
        self.serverThread = None
 
    def create( self, threaded = True ):
        # Port 0 means to select an arbitrary unused port.
        # Create the server, binding to localhost on servicer port.
        if threaded:
            print "Starting multi-thread server on %s:%d..." % ( self.host, self.port )
            self.server = GPIOThreadedTCPServer( ( self.host, self.port ), self.handler )
        else:
            print "Starting single thread server on %s:%d..." % ( self.host, self.port )
            self.server = SocketServer.TCPServer( ( self.host, self.port ), self.handler )
        
        # host, port = server.server_address
        return self
   
    def run( self, background = False ):
        try:
            if background:
                # Start a thread with the server -- that thread will then start one
                # more thread for each request.
                self.serverThread = threading.Thread( target=self.server.serve_forever )
                # Exit the server thread when the main thread terminates.
                self.serverThread.daemon = True
                self.serverThread.start()
                print "Server loop running in thread:", self.serverThread.name
            else:
                # Activate the server. This will keep running until you
                # interrupt the program with Ctrl-C.
                self.server.serve_forever()
        except KeyboardInterrupt:
            print "Generic server quit using keyboard interrupt."
            self.shutdown()
        except:
            printException()
            self.shutdown()

    def shutdown( self ):
        if self.server:
            self.server.shutdown()
            self.server = None


class GenericClient():
    def __init__( self, host = LOCAL_HOST, port = DEFAULT_PORT ):
        self.host = host
        self.port = port

    def send( self, message ):
        response = None
        clientSocket = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
        clientSocket.connect( (LOCAL_HOST, self.port ))
        
        try:
            clientSocket.sendall( message )
            
            # Wait for one reply.
            packedLen = clientSocket.recv( 4 )
                
            if len( packedLen ) == 0:
                # This is probably due to client disconnecting.
                print "Socket closed."
            else:
                ( size, ) = struct.unpack( "!I", packedLen )
                response = self.request.recv( size )
                # print 'Length: %d, Data: %s' % ( size, response )
        finally:
            clientSocket.close()
        
        return response
       
    def test( self ):
        for message in ( "Hello.", "How are you?", "Goodbye." ):
            print "Sending message to server: {}".format( message )
            
            response = self.send( message )
            
            if response:
                print "Received: {}".format( response )
            else:
                print "No reply."
  
     

     
if __name__ == '__main__':
    kwargs = {}
    lenArgs = len( sys.argv )
    
    if lenArgs > 1:
        type = sys.argv[1]
        
        if lenArgs > 2:
            kwargs['host'] = sys.argv[2]
        
            if lenArgs > 3:
                kwargs['port'] = sys.argv[3]
    
    if "-server" == type:
        GenericServer( **kwargs ).create( threaded = False ).run() # Single thread.
        # GenericServer( **kwargs ).create().run() # Multi-thread server.
    elif "-test-client" == type:
        GenericClient( **kwargs ).test()
    elif "-scratch-server" == type:
        kwargs['handler'] = GPIOScratchRequestHandler
        GenericServer( **kwargs ).create( threaded = False ).run() # Single thread.
        # GenericServer( **kwargs ).create().run() # Multi-thread server.
    else: # -scratch-client
        GPIOScratchClient( **kwargs ).run()
