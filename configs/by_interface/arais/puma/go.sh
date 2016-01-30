#!/bin/bash

export FLAVOR=rt-preempt
pkill -9 python
linuxcnc_stop 
sudo cp /dev/null /var/log/linuxcnc.log 
mklauncher . &
# -f: --fpga, load fpga bit file
# -s: --startpy, start python scripts
# python run.py -f -c -s -d -iVBC3.ini
python run.py -f -c -s -iRA605.ini
# python run.py -f -c -s -iVBC3_noH.ini
# python run.py -c -d -iVBC3-SIM.ini
