# HAL file for BeagleBone + TCT paralell port cape with 5 steppers and 3D printer board
import os

from machinekit import rtapi as rt
from machinekit import hal
from machinekit import config as c

from config import base
from config import motion

# testing
base.setup_motion()     # create motion-signals for motion-pins
base.setup_io()         # connect i/o signals to wosi
base.setup_analog()     # connect analog signals to wosi
base.setup_debug()      # connect debug signals to wosi
base.setup_signals()    # for Machineface.signals

# start haltalk server after everything is initialized
# else binding the remote components on the UI might fail
# comment out the following line for AXIS:
hal.loadusr('haltalk', wait=True)
