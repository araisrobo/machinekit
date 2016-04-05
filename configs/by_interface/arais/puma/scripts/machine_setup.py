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
e.do_home(-1)
e.wait_homing()

if (e.ok_for_mdi() == False):
    print "ERROR: not ready for MDI, quit ..."
    quit()
else:
    print "Prepare for MDI ..."
    e.prepare_for_mdi()

print "about to issue MDI commands ..."
e.g("G21")
e.g("F4000")
# e.g("F12000")
e.g("G64 R1")   # CANON_JOINT_EXACT
# e.g("G64 P0 R1")  # CANON_JOINT_CONTINUOUS
e.g("G90")
a = 0
while (a < 10000):
    e.g("G1 X0.008 Y-0.004 Z-90.012 A-0.006 B-89.992 C-0.00184") # P0
    e.g("G1 X-145 Y-40 Z-115 A-180 B30 C145")
    e.g("G1 X0.007 Y37.484 Z-64.313 A-0.004 B8.916 C0.00486") #P1
    e.g("G1 X145 Y-65 Z-60 A180 B-100 C-100")
    e.g("G1 X0.007 Y37.484 Z-64.313 A-0.004 B8.916 C0.00486") #P1
    e.g("G1 X-90 Y-70 Z-55 A0 B100 C100")
    e.g("G1 X0.007 Y37.484 Z-64.313 A-0.004 B8.916 C0.00486") #P1
    e.g("G1 X90 Y-40 Z-125 A-180 B90 C100")
    e.g("G1 X0.007 Y37.484 Z-64.313 A-0.004 B8.916 C0.00486") #P1
    j = 0
    while (j < 5):
        e.g("G1 X-45 Y-15 Z-125 A0 B-40 C180")
        e.g("G1 X0.007 Y37.484 Z-64.313 A-0.004 B8.916 C0.00486") #P1
        e.g("G1 X45 Y-15 Z-125 A0 B-40 C180")
        e.g("G1 X0.007 Y37.484 Z-64.313 A-0.004 B8.916 C0.00486") #P1
        j+=1
    a+=1
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


