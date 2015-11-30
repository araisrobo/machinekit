#!/usr/bin/env python
'''Copied from m61-test'''

import linuxcnc
from linuxcnc_control import  LinuxcncControl
import hal

from time import sleep
import sys
import os


"""Run the test"""

h = hal.component("python-ui")
h.ready() # mark the component as 'ready'

#
# connect to LinuxCNC
#

e = LinuxcncControl(1)
e.g_raise_except = False
e.set_mode(linuxcnc.MODE_MANUAL)
e.set_state(linuxcnc.STATE_ESTOP_RESET)
e.set_state(linuxcnc.STATE_ON)

# if (e.is_all_homed() == False):
e.do_home(-1) # home-all
# e.do_home(4) # homing for AXIS_4/JOINT_5/B
e.wait_homing()

if (e.ok_for_mdi() == False):
    print "ERROR: not ready for MDI, quit ..."
    quit()
else:
    print "Prepare for MDI ..."
    e.prepare_for_mdi()

print "about to issue MDI commands ..."
# e.g("G53G0X395")
# e.g("G53G0X0")
# e.g("G04 P1")
# e.g("G91 F65000 G1 X2889.25")
# e.g("G91 F35000 G1 Y1200.00")
# 
# e.g("G04 P1")
# e.g("G91 F65000 G1 X2889.25")
# e.g("G91 F35000 G1 Y1200.00")


# e.set_mode(linuxcnc.MODE_AUTO)
# if len(sys.argv)>1:
#     e.open_program(sys.argv[1])
#     e.run_full_program()
#     e.wait_running()
#     # e.do_pause(7)   # pause at line 5
#     # sleep(0.5)
#     # e.do_resume()
# else:
#     print "No G code specified, setup complete"


