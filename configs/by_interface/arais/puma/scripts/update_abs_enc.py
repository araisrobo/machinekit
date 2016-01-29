#!/usr/bin/env python
# -*- coding: UTF-8 -*


import time
import re
import linuxcnc
from linuxcnc_control import  LinuxcncControl
from machinekit import hal

def update_abs_enc(ini):
#     print "e.s.task_state: ", e.s.task_state
    if (e.s.task_state != linuxcnc.STATE_ON):
        joints = ini.find("ARAIS", "JOINTS")
        # print ("JOINTS", joints)
        for i in xrange (0, int(joints)):
            enc_abs = ini.find("JOINT_%d" % i, "ENC_ABS")
            # print "J%d: enc_abs(%d)" % (i, int(enc_abs)) 
            if (enc_abs == "1"):
                hal.Pin("son_delay.out")
                # print("enc_abs == 1")
                if (("J%d-abs-enc-pos" % i) in hal.signals):
                    abs_enc_pos = hal.Signal("J%d-abs-enc-pos" % i).get()
                    enc_pos = hal.Pin("wosi.stepgen.%d.enc-pos" % i).get()
                    # print "abs_enc_pos", abs_enc_pos
                    # print "enc_pos", enc_pos

                    if (abs_enc_pos != enc_pos):
                        # print "TODO: we need to update enc-pos with abs-enc-pos"
                        hal.Pin("wosi.stepgen.%d.set-enc-req" % i).set(True)
                        """ TODO: do not allow machine-on when abs-encoder-is-not-updated """
                    else:
                        # print "TODO: enc-pos == abs-enc-pos"
                        if (hal.Pin("wosi.stepgen.%d.set-enc-ack" % i).get() == True):
                            hal.Pin("wosi.stepgen.%d.set-enc-req" % i).set(False)
                else:
                    print ("ERROR: no signal, J%d-abs-enc-pos" % i)
                    exit(1)
# end of update_abs_enc(ini)

e = LinuxcncControl(1)

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

while True:
    e.s.poll()
    update_abs_enc(ini)
    time.sleep(0.01)
