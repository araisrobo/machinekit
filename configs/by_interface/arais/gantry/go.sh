#!/bin/bash

export FLAVOR=posix
linuxcnc_stop 
sudo cp /dev/null /var/log/linuxcnc.log 
sudo cp /dev/null /var/log/messages
sudo cp /dev/null /var/log/syslog

mklauncher . &
python run.py -f -c -iGANTRY.ini      # w/o debug
# python run.py -f -c -d -iGANTRY.ini # with debug

sleep 5 # delay 5 seconds for wosi_trans to be ready
# set wosi_trans with highest priority
sudo renice -n -20 -p `ps -C wosi_trans -o pid=`
