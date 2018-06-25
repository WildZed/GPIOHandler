# GPIOHandler
A Raspberry PI GPIO handler to interface to Scratch.

Needs a bit of reorganising, but basically it builds on an earlier script (Authors mentioned in gpio.py)
to add some client and server classes and protocol classes to handle communication between Scratch
and a Raspberry PI with a GPIO board setup with some simple morse code LEDs and buttons.
It also has a different run mode to run as a server interfacing to a simple Android app
to send some simple GPIO commands.
