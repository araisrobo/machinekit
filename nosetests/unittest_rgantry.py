#!/usr/bin/env python
# -*- coding: UTF-8 -*
# vim: sts=4 sw=4 et

# rgantry is the risc-gantry comp for the HAL instantiation API

from nose import with_setup
from machinekit.nosetests.realtime import setup_module ,teardown_module
from machinekit.nosetests.support import fnear
from unittest import TestCase
import time,os,ConfigParser

from machinekit import rtapi,hal

class TestRgantry(TestCase):
    def setUp(self):
        global rt
        self.cfg = ConfigParser.ConfigParser()
        self.cfg.read(os.getenv("MACHINEKIT_INI"))
        self.uuid = self.cfg.get("MACHINEKIT", "MKUUID")
        rt = rtapi.RTAPIcommand(uuid=self.uuid)
        # create rgantry instance for 2 joints
        rt.newinst("rgantry","gantry.y", "pincount=2");
        rt.newthread("servo-thread",1000000,fp=True)
        hal.addf("gantry.y.read","servo-thread")
        hal.addf("gantry.y.write","servo-thread")
        hal.start_threads()

        self.y_pos_cmd = hal.Pin("gantry.y.position-cmd")
        assert self.y_pos_cmd.type == hal.HAL_FLOAT
        assert self.y_pos_cmd.dir == hal.HAL_IN
        assert self.y_pos_cmd.linked == False
        
        self.y_pos_fb = hal.Pin("gantry.y.position-fb")
        assert self.y_pos_fb.type == hal.HAL_FLOAT
        assert self.y_pos_fb.dir == hal.HAL_OUT
        assert self.y_pos_fb.linked == False
        
        self.homing_abs_enc = hal.Pin("gantry.y.homing-abs-enc")
        assert self.homing_abs_enc.type == hal.HAL_BIT
        assert self.homing_abs_enc.dir == hal.HAL_IN
        assert self.homing_abs_enc.linked == False
        
        self.homing = hal.Pin("gantry.y.homing")
        assert self.homing.type == hal.HAL_BIT
        assert self.homing.dir == hal.HAL_IN
        assert self.homing.linked == False
        
        self.home_state = hal.Pin("gantry.y.home-state")
        assert self.home_state.type == hal.HAL_S32
        assert self.home_state.dir == hal.HAL_OUT
        assert self.home_state.linked == False
        
        self.j1_pos_cmd = hal.Pin("gantry.y.joint.00.pos-cmd")
        assert self.j1_pos_cmd.type == hal.HAL_FLOAT
        assert self.j1_pos_cmd.dir == hal.HAL_OUT
        assert self.j1_pos_cmd.linked == False
        
        self.j2_pos_cmd = hal.Pin("gantry.y.joint.01.pos-cmd")
        assert self.j2_pos_cmd.type == hal.HAL_FLOAT
        assert self.j2_pos_cmd.dir == hal.HAL_OUT
        assert self.j2_pos_cmd.linked == False
        
        self.j1_pos_fb = hal.Pin("gantry.y.joint.00.pos-fb")
        assert self.j1_pos_fb.type == hal.HAL_FLOAT
        assert self.j1_pos_fb.dir == hal.HAL_IN
        assert self.j1_pos_fb.linked == False
        
        self.j2_pos_fb = hal.Pin("gantry.y.joint.01.pos-fb")
        assert self.j2_pos_fb.type == hal.HAL_FLOAT
        assert self.j2_pos_fb.dir == hal.HAL_IN
        assert self.j2_pos_fb.linked == False
        
        self.j1_home_sw_id = hal.Pin("gantry.y.joint.00.home-sw-id")
        assert self.j1_home_sw_id.type == hal.HAL_S32
        assert self.j1_home_sw_id.dir == hal.HAL_IN
        assert self.j1_home_sw_id.linked == False
        
        self.j2_home_sw_id = hal.Pin("gantry.y.joint.01.home-sw-id")
        assert self.j2_home_sw_id.type == hal.HAL_S32
        assert self.j2_home_sw_id.dir == hal.HAL_IN
        assert self.j2_home_sw_id.linked == False
        
        self.j1_home_offset = hal.Pin("gantry.y.joint.00.home-offset")
        assert self.j1_home_offset.type == hal.HAL_FLOAT
        assert self.j1_home_offset.dir == hal.HAL_IN
        assert self.j1_home_offset.linked == False
        
        self.j2_home_offset = hal.Pin("gantry.y.joint.01.home-offset")
        assert self.j2_home_offset.type == hal.HAL_FLOAT
        assert self.j2_home_offset.dir == hal.HAL_IN
        assert self.j2_home_offset.linked == False
        
        self.home_sw_o = hal.Pin("gantry.y.home-sw-o")
        assert self.home_sw_o.type == hal.HAL_BIT
        assert self.home_sw_o.dir == hal.HAL_OUT
        assert self.home_sw_o.linked == False
        
        self.j1_home_sw_i = hal.Pin("gantry.y.joint.00.home-sw-i")
        assert self.j1_home_sw_i.type == hal.HAL_BIT
        assert self.j1_home_sw_i.dir == hal.HAL_IN
        assert self.j1_home_sw_i.linked == False
        
        self.j2_home_sw_i = hal.Pin("gantry.y.joint.01.home-sw-i")
        assert self.j2_home_sw_i.type == hal.HAL_BIT
        assert self.j2_home_sw_i.dir == hal.HAL_IN
        assert self.j2_home_sw_i.linked == False
        
        self.risc_probe_vel_i = hal.Pin("gantry.y.risc-probe-vel-i")
        assert self.risc_probe_vel_i.type == hal.HAL_FLOAT
        assert self.risc_probe_vel_i.dir == hal.HAL_IN
        assert self.risc_probe_vel_i.linked == False
        
        self.j1_risc_probe_pin_o = hal.Pin("gantry.y.joint.00.risc-probe-pin-o")
        assert self.j1_risc_probe_pin_o.type == hal.HAL_S32
        assert self.j1_risc_probe_pin_o.dir == hal.HAL_OUT
        assert self.j1_risc_probe_pin_o.linked == False
        
        self.j2_risc_probe_pin_o = hal.Pin("gantry.y.joint.01.risc-probe-pin-o")
        assert self.j2_risc_probe_pin_o.type == hal.HAL_S32
        assert self.j2_risc_probe_pin_o.dir == hal.HAL_OUT
        assert self.j2_risc_probe_pin_o.linked == False
        
        self.risc_probe_type_i = hal.Pin("gantry.y.risc-probe-type-i")
        assert self.risc_probe_type_i.type == hal.HAL_S32
        assert self.risc_probe_type_i.dir == hal.HAL_IN
        assert self.risc_probe_type_i.linked == False
        
        self.j1_risc_probe_type_o = hal.Pin("gantry.y.joint.00.risc-probe-type-o")
        assert self.j1_risc_probe_type_o.type == hal.HAL_S32
        assert self.j1_risc_probe_type_o.dir == hal.HAL_OUT
        assert self.j1_risc_probe_type_o.linked == False

        self.j2_risc_probe_type_o = hal.Pin("gantry.y.joint.01.risc-probe-type-o")
        assert self.j2_risc_probe_type_o.type == hal.HAL_S32
        assert self.j2_risc_probe_type_o.dir == hal.HAL_OUT
        assert self.j2_risc_probe_type_o.linked == False
        
    """ can only do this test_case with loadrt, 
        refer to Test Fixtures of 
        http://ivory.idyll.org/articles/nose-intro.html """
    def test_index_homing(self):
        """ RISC probe commands from sync_cmd.h """
        RISC_PROBE_LOW = 0
        RISC_PROBE_HIGH = 1
        RISC_PROBE_INDEX = 2
        
        """ gantry homing states from rgantry.icomp """
        GH_IDLE = 0
        GH_SWITCH_SEARCH = 1 
        GH_INDEX_SEARCH_SLAVE = 2
        GH_INDEX_BACK_TO_SWITCH = 3
        GH_INDEX_SEARCH_MASTER = 4
        GH_DONE = 5
        
        """
            及仁的 HOME SWITCH 是 LOW ACTIVE,
            RISC_PROBE_HIGH 意味著目前電壓是 LOW, 要找變成 HIGH 的位置
            電壓是 LOW 的 joint_home_sw_i(i) 的值是 TRUE，
            要等每一軸的 joint_home_sw_i(i) 都變成 FALSE
        """
        """ 初始化 jX_home_sw_i, 並確認 home_sw_o 數值正確 """
        self.j1_home_sw_i.set(True)
        self.j2_home_sw_i.set(True)
        time.sleep(0.1) # wait for executing rgantry.read() and rgantry.write()
        assert self.home_sw_o.get() == True
        assert self.home_state.get() == GH_IDLE
        
        """ Switch Homing for Incremental Encoder """
        self.homing_abs_enc.set(False)  
        self.homing.set(True)
        self.risc_probe_vel_i.set(10.0)
        self.risc_probe_type_i.set(RISC_PROBE_HIGH)
        time.sleep(0.1) # wait for executing rgantry.read() and rgantry.write()
        assert self.risc_probe_vel_i.get() == 10.0
        assert self.risc_probe_type_i.get() == RISC_PROBE_HIGH
        assert self.homing_abs_enc.get() == False
        assert self.homing.get() == True
        assert self.home_state.get() == GH_SWITCH_SEARCH
        assert self.j1_home_sw_i.get() == True
        assert self.j2_home_sw_i.get() == True
        assert self.home_sw_o.get() == True

        """ J1 先到達 SWITCH 位置，電壓變為 HIGH, j1_home_sw_i 的數值變為 False """
        self.j1_home_sw_i.set(False)
        self.j2_home_sw_i.set(True)
        time.sleep(0.1) # wait for executing rgantry.read() and rgantry.write()
        assert self.home_state.get() == GH_SWITCH_SEARCH
        assert self.j1_home_sw_i.get() == False
        assert self.j2_home_sw_i.get() == True
        assert self.home_sw_o.get() == True
        
        """ J2 也到達 SWITCH 位置，電壓變為 HIGH, j2_home_sw_i 的數值變為 False """
        self.j2_home_sw_i.set(False)
        time.sleep(0.1) # wait for executing rgantry.read() and rgantry.write()
        assert self.home_state.get() == GH_SWITCH_SEARCH
        assert self.j1_home_sw_i.get() == False
        assert self.j2_home_sw_i.get() == False
        print "home_sw_o: ", self.home_sw_o.get()
        assert self.home_sw_o.get() == False
        
        
(lambda s=__import__('signal'):
     s.signal(s.SIGTERM, s.SIG_IGN))()
