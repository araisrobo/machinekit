# HAL file for BeagleBone + TCT paralell port cape with 5 steppers and 3D printer board
import os
import re

from machinekit import rtapi as rt
from machinekit import hal
from machinekit import config as c

from config import base
from config import vbc_param
from config import motion

from linuxcnc_control import  LinuxcncControl
import linuxcnc

inifile = None
lprint = open("/tmp/linuxcnc.print", "r")
for line in lprint:
    m = re.match("^INIFILE=(.*)", line)
    if (m):
        inifile = m.group(1)

if (inifile):
    ini = linuxcnc.ini(inifile)
else:
    print "No valid INI_FILE_NAME, please launch Machinekit"
    exit(1)

# testing
# base.setup_motion(ini)    # create motion-signals for motion-pins
# base.setup_io()           # connect i/o signals to wosi
# base.setup_analog()       # connect analog signals to wosi
# base.setup_debug()        # connect debug signals to wosi
base.setup_signals(ini)     # for Machineface.signals
vbc_param.setup_signals()

# start haltalk server after everything is initialized
# else binding the remote components on the UI might fail
# comment out the following line for AXIS:
hal.loadusr('haltalk', wait=True)
