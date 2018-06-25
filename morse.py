# Scratch morse code handlers.

import threading
import time
import datetime as dt
import sys


letterMorse = {
    'a' : '.-',
    'b' : '-...',
    'c' : '-.-.',
    'd' : '-..',
    'e' : '.',
    'f' : '..-.',
    'g' : '--.',
    'h' : '....',
    'i' : '..',
    'j' : '.---',
    'k' : '-.-',
    'l' : '.-..',
    'm' : '--',
    'n' : '-.',
    'o' : '---',
    'p' : '.--.',
    'q' : '--.-',
    'r' : '.-.',
    's' : '...',
    't' : '-',
    'u' : '..-',
    'v' : '...-',
    'w' : '.--',
    'x' : '-..-',
    'y' : '-.--',
    'z' : '--..',
    '0' : '-----',
    '1' : '.----',
    '2' : '..---',
    '3' : '...--',
    '4' : '....-',
    '5' : '.....',
    '6' : '-....',
    '7' : '--...',
    '8' : '---..',
    '9' : '----.'
}

morseLetter = {}

for letter in letterMorse.keys():
    morse = letterMorse[letter]
    morseLetter[morse] = letter

dotLen = 0.1
dashLen = 0.4
postBlipDelay = 0.2
postLetterDelay = 0.4
postWordDelay = 0.8
maxDotLen = 0.2
clearLen = 1
maxInterLetterDelay = 0.4
maxInterWordDelay = 1.4


class MorseSender:
    def __init__( self, switchUpdate ):
        # Pin update lambda function.
        self.switchUpdate = switchUpdate
    
    def blip( self, length ):
        self.switchUpdate( 1 )
        time.sleep( length )
        self.switchUpdate( 0 )
        time.sleep( postBlipDelay )
    
    def sendMessage( self, msg ):
        inWord = 0
        print "> %s" % msg 
        for char in msg:
            if letterMorse.has_key(char):
                inWord = 1
                morse = letterMorse[char]
                # print "Sending morse '%s'" % morse
                for blip in morse:
                    if '.' == blip:
                        self.blip( dotLen )
                    else:
                        self.blip( dashLen )
                time.sleep( postLetterDelay )
            elif inWord:
                inWord = 0
                time.sleep( postWordDelay )

                
class MorseReceiver:
    def __init__( self, sendMessage, sendWord ):
        # Pin update lambda function.
        self.sendMessage = sendMessage
        self.sendWord = sendWord
        self.blipBuffer = []
        self.wordBuffer = []
        self.messageBuffer = ""
        self.time = dt.datetime.now()
        self.timer = 0
        self.value = 0
        self.space = ''

    def debugWrite( self, str ):
        sys.stdout.write( str )
        sys.stdout.flush()

    def setMessage( self, message ):
        self.messageBuffer = message
        self.wordBuffer = []
        self.blipBuffer = []
        self.space = ''
        self.sendMessage( self.messageBuffer )

    def storeBlip( self, elapsed ):
        # print "Elapsed %f" % elapsed
        if elapsed > clearLen: # Clear the message.
            self.setMessage( '' )
            
            print "Cleared morse message"
        elif elapsed > maxDotLen: # Dash.
            self.debugWrite( '-' )
            self.blipBuffer.append( '-' )
            self.startTimer()
        else: # Dot.
            self.debugWrite( '.' )
            self.blipBuffer.append( '.' )
            self.startTimer()

    def storeLetter( self ):
        if not self.blipBuffer:
            return
        
        morse = ''.join(self.blipBuffer)
        self.blipBuffer = []
        
        if morseLetter.has_key( morse ):
            letter = morseLetter[morse]
            self.wordBuffer.append( letter )
            self.messageBuffer += self.space + letter
            self.space = ''
            self.debugWrite( '%s|' % letter )
            self.sendMessage( self.messageBuffer )

    def storeWord( self ):
        word = ''.join(self.wordBuffer)
        self.wordBuffer = []
        self.debugWrite( ' %s\n' % word )
        # print "'%s'" % self.messageBuffer
        self.space = ' '
        self.sendWord( word )

    def letterTimeout( self ):
        self.timer.cancel()
        self.storeLetter()
        self.timer = threading.Timer(maxInterWordDelay-maxInterLetterDelay, self.wordTimeout)
        self.timer.start()

    def wordTimeout( self ):
        self.timer.cancel()
        self.timer = 0
        self.storeWord()

    def startTimer( self ):
        try:
            self.timer = threading.Timer(maxInterLetterDelay, self.letterTimeout)
            self.timer.start()
        except:
            print "Exception in MorseReceiver.startTimer()"

    def cancelTimer( self ):
        try:
            if self.timer:
                self.timer.cancel()
                self.timer = 0
        except:
            print "Exception in MorseReceiver.cancelTimer()"
           
    def update( self, value ):
        if value == self.value:
            return
        
        self.value = value
        last = self.time
        self.time = dt.datetime.now()
        
        # print "%d : %s" % (value, self.time)
        
        if value: # On.
            self.cancelTimer()
        else: # Off.
            delta = self.time - last
            elapsed = delta.seconds + ( delta.microseconds / 1e6 ) # Seconds.
            self.storeBlip(elapsed)



