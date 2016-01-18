#!/bin/bash

export FLAVOR=rt-preempt
linuxcnc_stop 
sudo cp /dev/null /var/log/linuxcnc.log 
mklauncher . &
python run.py -f -c -iGANTRY.ini
# python run.py -f -c -d -iGANTRY.ini
# python run.py -f -c -iMEINAN_noH.ini

