#!/bin/bash

# export FLAVOR=rt-preempt
export FLAVOR=posix
pkill -9 python
linuxcnc_stop 

sudo cp /dev/null /var/log/linuxcnc.log 
sudo cp /dev/null /var/log/messages
sudo cp /dev/null /var/log/syslog

mklauncher . &
# -f: --fpga, load fpga bit file
# -s: --startpy, start python scripts
# python run.py -f -c -s -d -iVBC3.ini
# python run.py -f -c -s -iRA605.ini
# python run.py -f -c -d -s -iRA605-TRIV.ini # -d: with debug
python run.py -f -c -s -iRA605-TRIV.ini
# python run.py -c -iRA605-SIM.ini
# python run.py -f -c -iRA605-TRIV-noH.ini    # no "-s", for updating abs-enc-pos
# python run.py -f -c -s -iVBC3_noH.ini
# python run.py -c -d -iVBC3-SIM.ini
