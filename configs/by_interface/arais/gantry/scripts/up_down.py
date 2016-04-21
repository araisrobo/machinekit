#!/usr/bin/env python
# -*- coding: UTF-8 -*

import time
import os
import re
import sys
import linuxcnc
from linuxcnc_control import  LinuxcncControl
from machinekit import hal
class OddPosition(object):

    def __init__(self, i, position, width, offset):
        self.pos_max = (position + (offset * i)) + width 
        self.pos_min = (position + (offset * i)) - width
        if (self.pos_min >  self.pos_max):
            tmp_pos = self.pos_max
            self.pos_max = self.pos_min
            self.pos_min = tmp_pos


def gantry_hal(ini):
    # adcs = []
    # for i in range(0,16):
    #     tmp_scale = ini.find("ARAIS", "ADC%d_SCALE" % i)
    #     tmp_offset = ini.find("ARAIS", "ADC%d_OFFSET" % i)
    #     if (tmp_scale != None and tmp_offset != None): 
    #         adc = AdcScale(float(tmp_scale), float(tmp_offset)) 
    #         adcs.append(adc)
    #         print 'ain_%d * %f + %f' % (i,adcs[i].scale, adcs[i].offset)
    #         tmp_pin = h.newpin("ain_%d" % i, hal.HAL_FLOAT, hal.HAL_IN)
    #         hal.Signal("ain_%d" % i).link(tmp_pin)

    tmp_pin = h.newpin("odd-platen", hal.HAL_BIT, hal.HAL_OUT)
    hal.Signal("dout_5").link(tmp_pin)
    h['odd-platen'] = False
    tmp_pin = h.newpin("even-platen", hal.HAL_BIT, hal.HAL_OUT)
    hal.Signal("dout_6").link(tmp_pin)
    h['even-platen'] = False
    tmp_pin = h.newpin("check-Ypos", hal.HAL_BIT, hal.HAL_IN)
    # hal.Pin("motion.digital-in-28").link(tmp_pin)
    hal.Signal("dout_28").link(tmp_pin)
    h['check-Ypos'] = False
    h.ready() # mark the component as 'ready'

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

h = hal.Component("gantry")
gantry_hal(ini)
# position: HOMING 完最靠近我們的奇數圓桿中心Y座標
# width: 圓桿中心到兩根圓桿中間的寬度
# offset: 奇數圓桿到奇數圓桿的距離
# odd_num: 奇數圓桿的數量
position = ini.find("SCRIPTS", "ODD_POSITION")
width = ini.find("SCRIPTS", "ODD_WIDTH")
offset = ini.find("SCRIPTS", "ODD_OFFSET") 
odd_num = ini.find("SCRIPTS", "ODD_NUMBER")
dwell = ini.find("SCRIPTS", "DWELL")
set_odd = True 
positions = []
for i in range(0, int(odd_num)):
    pos = OddPosition(float(i),float(position), float(width), float(offset)) 
    positions.append(pos)
    
while True:
    e.s.poll()
    set_odd = True  
    #  M64P_ => 虛擬的 OUT_28 在執行 GCODE 時，輸出是否要更新平台升降，
    # 避免Y軸移動的過程經過太多區塊平台上上下下
    if h['check-Ypos']:
        for i in range(0, int(odd_num)):
            if (e.s.position[1] < positions[i].pos_max and \
                e.s.position[1] > positions[i].pos_min):
                set_odd = False 
        if set_odd == False and h['odd-platen']:
            h['even-platen'] = True 
            time.sleep(float(dwell))
            h['odd-platen'] = False
            # print "In Odd section Y_pos(%f)" % e.s.position[1]
        elif set_odd == True and (not h['odd-platen']):
            h['odd-platen'] = True 
            time.sleep(float(dwell))
            h['even-platen'] = False
            # print "Out Odd section Y_pos(%f)" % e.s.position[1]

    time.sleep(0.1)

