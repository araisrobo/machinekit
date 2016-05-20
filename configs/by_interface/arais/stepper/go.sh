#!/bin/bash

# export FLAVOR=rt-preempt
export FLAVOR=posix
linuxcnc_stop 
sudo cp /dev/null /var/log/linuxcnc.log 
mklauncher . &
python run.py -f -c -iSTEPPER.ini
# python run.py -f -c -d -iSTEPPER.ini
