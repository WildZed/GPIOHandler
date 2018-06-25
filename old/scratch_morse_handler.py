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
    def __init__( self, switch_update ):
        # Pin update lambda function.
        self.switch_update = switch_update
    
    def blip( self, length ):
        self.switch_update( 1 )
        time.sleep( length )
        self.switch_update( 0 )
        time.sleep( postBlipDelay )
    
    def send_message( self, msg ):
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
    def __init__( self, send_message, send_word ):
        # Pin update lambda function.
        self.send_message = send_message
        self.send_word = send_word
        self.blip_buffer = []
        self.word_buffer = []
        self.message_buffer = ""
        self.time = dt.datetime.now()
        self.timer = 0
        self.value = 0
        self.space = ''

    def debug_write( self, str ):
        sys.stdout.write( str )
        sys.stdout.flush()

    def set_message( self, message ):
        self.message_buffer = message
        self.word_buffer = []
        self.blip_buffer = []
        self.space = ''
        self.send_message( self.message_buffer )

    def store_blip( self, elapsed ):
        # print "Elapsed %f" % elapsed
        if elapsed > clearLen: # Clear the message.
            self.set_message( '' )
            print "Cleared morse message"
        elif elapsed > maxDotLen: # Dash.
            self.debug_write( '-' )
            self.blip_buffer.append( '-' )
            self.start_timer()
        else: # Dot.
            self.debug_write( '.' )
            self.blip_buffer.append( '.' )
            self.start_timer()

    def store_letter( self ):
        if not self.blip_buffer:
            return
        morse = ''.join(self.blip_buffer)
        self.blip_buffer = []
        if morseLetter.has_key( morse ):
            letter = morseLetter[morse]
            self.word_buffer.append( letter )
            self.message_buffer += self.space + letter
            self.space = ''
            self.debug_write( '%s|' % letter )
            self.send_message( self.message_buffer )

    def store_word( self ):
        word = ''.join(self.word_buffer)
        self.word_buffer = []
        self.debug_write( ' %s\n' % word )
        # print "'%s'" % self.message_buffer
        self.space = ' '
        self.send_word( word )

    def letter_timeout( self ):
        self.timer.cancel()
        self.store_letter()
        self.timer = threading.Timer(maxInterWordDelay-maxInterLetterDelay, self.word_timeout)
        self.timer.start()

    def word_timeout( self ):
        self.timer.cancel()
        self.timer = 0
        self.store_word()

    def start_timer( self ):
        try:
            self.timer = threading.Timer(maxInterLetterDelay, self.letter_timeout)
            self.timer.start()
        except:
            print "Exception in MorseReceiver.start_timer()"

    def cancel_timer( self ):
        try:
            if self.timer:
                self.timer.cancel()
                self.timer = 0
        except:
            print "Exception in MorseReceiver.cancel_timer()"
           
    def update( self, value ):
        if value == self.value:
            return
        self.value = value
        last = self.time
        self.time = dt.datetime.now()
        # print "%d : %s" % (value, self.time)
        if value: # On.
            self.cancel_timer()
        else: # Off.
            delta = self.time - last
            elapsed = delta.seconds + ( delta.microseconds / 1e6 ) # Seconds.
            self.store_blip(elapsed)



