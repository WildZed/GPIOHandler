#!/bin/bash
#Version 0.2 - add in & to allow simulatenous running of handler and Scratch
#Version 0.3 - change sp launches rsc.sb from "/home/pi/Documents/Scratch Projects"
#Version 0.4 - 20Mar13 meltwater - change to use provided name for home
sudo ps aux | grep 'python.*scratch_gpio_handler.py' | grep -v grep | awk '{print $2}' | xargs sudo kill -9 
sudo python /home/pi/simplesi_scratch_handler/scratch_gpio_handler.py &
myproj="rsc"
# myproj="zed"
# myproj="morse"
scratch --document "/home/pi/Documents/Scratch Projects/${myproj}.sb" &
