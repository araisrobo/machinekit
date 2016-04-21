#!/usr/bin/env python
# -*- coding: UTF-8 -*
# vim: sts=4 sw=4 et
import os,sys
from gladevcp.persistence import IniFile,widget_defaults,set_debug,select_widgets
import hal
import hal_glib
import gtk
import glib
import time
import gobject
import ConfigParser
import pango
import math
import shelve
import linuxcnc
import sqlite3
import zmq
# from scipy import mat
# from scipy import linalg
import linuxcnc

debug = 1

cords = "XYZABCUVW"

class EmcInterface(object):

    def __init__(self):
        try:
            # emcIniFile = linuxcnc.ini(os.environ['INI_FILE_NAME'])
            # print "debug: emcIniFile(%s)" % os.environ['INI_FILE_NAME']
            # linuxcnc.nmlfile = os.path.join(os.path.dirname(os.environ['INI_FILE_NAME']), emcIniFile.find("EMC", "NML_FILE"))
            self.s = linuxcnc.stat();
            self.c = linuxcnc.command()
        except Exception, msg:
            print "cant initialize EmcInterface: %s" % (msg)

    def active_codes(self):
        self.s.poll()
        return self.s.gcodes

    def get_current_system(self):
        for i in self.active_codes():
            if i >= 540 and i <= 590:
                return i/10 - 53
            elif i >= 590 and i <= 593:
                return i - 584
        return 1

    def mdi_command(self,command, wait=True):
        #ensure_mode(emself.c.MODE_MDI)
        self.c.mdi(command)
        if wait: self.c.wait_complete()
    
    def current_tool(self):
        self.s.poll()
        return self.s.tool_in_spindle

class SysState:
    tool_id = 0;
    gcode_pending = ""
    
class SysConfigs:
    work_height = 0
    safe_height = 0
    
stat = SysState()
stat.tool_id = 0
stat.gcode_pending = ""

configs = SysConfigs()
configs.safe_height = 35
configs.work_height = 0
configs.offset = 50
configs.home_orig = 71.5
configs.oil_dwell = 1.0
configs.blow_dwell = 1.0
# http://stackoverflow.com/questions/9698614/super-raises-typeerror-must-be-type-not-classobj-for-new-style-class
# must add (object) for new-style class definition
class Motion(object):
    def __init__(self, cur_pos):   # Constructor of the class
        self.cur_pos = cur_pos
    def convert(self):             # Abstract method, defined by convention only
        raise NotImplementedError("Subclass must implement abstract method")
    def description(self):
        raise NotImplementedError("Subclass must implement abstract method")
    def get_non_z_str(self):
        str = ""
        for k in "XYABCUVW":
            if k in self.cur_pos: # to check if key in dict
                str += "%s%.3f " % (k, self.cur_pos[k])
        return (str)
    def get_z_str(self):
        return ('Z%.3f' % (self.cur_pos["Z"]))
    def get_pos_str(self):
        return (self.get_non_z_str() + self.get_z_str())

class DispPoint(Motion):
    def __init__(self, cur_pos, dout, delay, lo_speed, lo_dist, clr_dist):   # Constructor of the class
        super(DispPoint,self).__init__(cur_pos)
#        Motion.__init__(self, cur_pos)
        self.dout = dout
        self.delay = delay
        self.lo_speed = lo_speed
        self.lo_dist = lo_dist
        self.clr_dist = clr_dist
        
    def description(self):
        return 'Dispense Point %s' % self.get_pos_str()
    
    def convert(self, id=-1):
        cmd = ("(begin: id[%d], %s)\n" % (id, self.description()),
#               "G90\n",
#               "G64P0", # Max blend mode
               "G0 %s\n" % (self.get_non_z_str()),
               "G61\n", # Exact stop mode
               "G0 %s\n" % (self.get_z_str()), 
               "M64P%d\n" % self.dout, # turn dispense nozzle on
               "G04P%.3f\n" % self.delay,
               "M65P%d\n" % self.dout, # turn dispense nozzle off
               "G04P%.3f\n" % self.delay,
               "G64P0\n", # Max blend mode
               "F%.3f\nG1 Z%.3f\n" % (self.lo_speed, (self.cur_pos["Z"] + self.lo_dist)),
               "G64P0\n", # Max blend mode
               "G0 Z%.3f\n" % (self.cur_pos["Z"] + self.clr_dist),
               "(end: id[%d])\n" % id
               )
        return ''.join(cmd)

class LineStart(Motion):
    def __init__(self, cur_pos, dout, delay, lo_speed, lo_dist, clr_dist):   # Constructor of the class
        super(LineStart,self).__init__(cur_pos)
#        Motion.__init__(self, cur_pos)
        self.dout = dout
        self.delay = delay
        self.lo_speed = lo_speed
        self.lo_dist = lo_dist
        self.clr_dist = clr_dist
        
    def description(self):
        return 'Line Start %s' % self.get_pos_str()
    
    def convert(self, id=-1):
        cmd = ("(begin: id[%d], %s)\n" % (id, self.description()),
#               "G90\n",
#               "G64P0", # Max blend mode
               "G0 %s\n" % (self.get_non_z_str()),
               "G61\n", # Exact stop mode
               "G0 %s\n" % (self.get_z_str()), 
               "M64P%d\n" % self.dout, # turn dispense nozzle on
               "G04P%.3f\n" % self.delay,
               "M65P%d\n" % self.dout, # turn dispense nozzle off
               "G04P%.3f\n" % self.delay,
               "G64P0\n", # Max blend mode
               "F%.3f\nG1 Z%.3f\n" % (self.lo_speed, (self.cur_pos["Z"] + self.lo_dist)),
               "G64P0\n", # Max blend mode
               "G0 Z%.3f\n" % (self.cur_pos["Z"] + self.clr_dist),
               "(end: id[%d])\n" % id
               )
        return ''.join(cmd)

class Drill(Motion):
    def __init__(self, cur_pos, drill_rpm, drill_feed, drill_depth, drill_work_height, plate_thk, confirm_din):   # Constructor of the class
        super(Drill,self).__init__(cur_pos)
        self.drill_rpm = drill_rpm
        self.drill_feed = drill_feed
        self.drill_depth = drill_depth
        self.drill_work_height = drill_work_height
        self.plate_thk = plate_thk
        self.confirm_din = confirm_din
    def description(self):
        return 'Drill %s' % (self.get_non_z_str())
    def set_tool_id(self, drill_id):
        self.drill_tool_id = drill_id
    def convert(self, id):
        global configs
        global stat
        gcode_buf = ""
        
        if (self.drill_tool_id != stat.tool_id):
            cmd = (stat.gcode_pending,# change to drill tool
                   "M6 T%s\n" % (self.drill_tool_id),
                   "G10 L2 P1 Z0\n",
                   "G10 L20 P1 Z%f\n"% (abs(self.drill_work_height)+configs.home_orig-self.plate_thk - configs.offset)) 
            gcode_buf = ''.join(cmd)
            stat.tool_id = self.drill_tool_id
            # stat.gcode_pending = "M6 T%s\n" % (self.tap_tool_id) +\
            #                       "G10 L2 P1 Z0\n" +\
            #                       "G10 L20 P1 Z%f\n"% (abs(self.tapping_work_height)+configs.home_orig-self.plate_thk - configs.offset)

        """ 1st pass: drill """
        cmd = (gcode_buf,
               "(begin: id[%d], %s, drill)\n" % (id, self.description()),
               "G90\n",
               "G0 X%f Y%f\n" % (self.cur_pos["X"], self.cur_pos["Y"]),
               "G0 Z%f\n" % (configs.work_height), 
#               "M66P%dL3Q10000\n" % self.confirm_din,
               "G91\n",
               "M3 S%f\n" % (self.drill_rpm), 
               "F%f\n" % (self.drill_feed),
               "G1 Z-%f\n" % (self.drill_depth), 
#               "G0 Z%f\n" % (self.drill_depth),
               "G90\n",
               "G0 Z%f\n" % (configs.safe_height),
               "M5\n",
               "(end: id[%d], %s, drill)\n" % (id, self.description()) 
               )
        
        return ''.join(cmd)

class ChangeTool(Motion):
    def description(self):
        return 'ChangeTool: %d ' % self.tool_id
    def set_tool_id(self, id):
        self.tool_id = id
    def convert(self, id):
        stat.tool_id = self.tool_id
        buf = "M6 T%d\n" % (self.tool_id)
        if (stat.gcode_pending != ""):
            cmd = (stat.gcode_pending,
                   buf)
            buf = ''.join(cmd)
            stat.gcode_pending = ""
        return buf
    
class RigidTap(Motion):
    def __init__(self, cur_pos, drill_rpm, drill_feed, drill_depth, drill_work_height, tap_rpm, tap_depth, tap_pitch, tapping_work_height, plate_thk, confirm_din):   # Constructor of the class
        super(RigidTap,self).__init__(cur_pos)
        self.drill_rpm = drill_rpm
        self.drill_feed = drill_feed
        self.drill_depth = drill_depth
        self.drill_work_height = drill_work_height
        self.tap_rpm = tap_rpm
        self.tap_depth = tap_depth
        self.tap_pitch = tap_pitch
        self.tapping_work_height = tapping_work_height
        self.plate_thk = plate_thk
        self.confirm_din = confirm_din
    def description(self):
        return 'RigidTap %s' % (self.get_non_z_str())
    def set_tool_id(self, drill_id, tap_id):
        self.drill_tool_id = drill_id
        self.tap_tool_id = tap_id
    def convert(self, id):
        global configs
        global stat
        gcode_buf = ""
        
        if (self.tap_tool_id != stat.tool_id):
            cmd = (stat.gcode_pending,# change to drill tool
                   "M6 T%s\n" % (self.drill_tool_id),
                   "G10 L2 P1 Z0\n",
                   "G10 L20 P1 Z%f\n"% (abs(self.drill_work_height)+configs.home_orig-self.plate_thk - configs.offset)) 
            gcode_buf = ''.join(cmd)
            stat.tool_id = self.drill_tool_id
            stat.gcode_pending = "M6 T%s\n" % (self.tap_tool_id) +\
                                  "G10 L2 P1 Z0\n" +\
                                  "G10 L20 P1 Z%f\n"% (abs(self.tapping_work_height)+configs.home_orig-self.plate_thk - configs.offset)

        """ 1st pass: drill """
        cmd = (gcode_buf,
               "(begin: id[%d], %s, drill)\n" % (id, self.description()),
               "G90\n",
               "G0 X%f Y%f\n" % (self.cur_pos["X"], self.cur_pos["Y"]),
               "G0 Z%f\n" % (configs.work_height), 
#               "M66P%dL3Q10000\n" % self.confirm_din,
               "G91\n",
               "M3 S%f\n" % (self.drill_rpm), 
               "F%f\n" % (self.drill_feed),
               "G1 Z-%f\n" % (self.drill_depth), 
#               "G0 Z%f\n" % (self.drill_depth),
               "G90\n",
               "G0 Z%f\n" % (configs.safe_height),
               "M5\n",
               "(end: id[%d], %s, drill)\n" % (id, self.description()) 
               )
        
        """ 2nd pass: tapping """
        cmd_tapping = (stat.gcode_pending,
                       "(begin: id[%d], %s, tapping)\n" % (id, self.description()),
                       "G90\n", 
                       "G0 X%f Y%f\n" % (self.cur_pos["X"], self.cur_pos["Y"]),
                       "G0 Z%f\n" % (configs.work_height), # TODO: work_height
#                       "M66P%dL3Q10000\n" % self.confirm_din,
                       "G91\n",
                       "M3 S%f\n" % (self.tap_rpm), # TODO: tap_rpm
                       "G33.1 Z-%f K%f\n" % (self.tap_depth, self.tap_pitch),
                       "M5\n",
                       "G90\n",
                       "G0 Z%f\n" % (configs.safe_height), # safe_height               
                       "(end: id[%d], tapping)\n" % id
                       )
        stat.gcode_pending = ''.join(cmd_tapping)
        return ''.join(cmd)

class CounterSink(Motion):
    def __init__(self, cur_pos, csk_rpm, csk_speed, csk_depth, init_height, csk_compensate, probe_din, probe_ain, probe_type, probe_condition, probe_alvl, plate_thk):   # Constructor of the class
        super(CounterSink,self).__init__(cur_pos)
        self.csk_rpm = csk_rpm
        self.csk_speed = csk_speed
        self.csk_depth = csk_depth
        self.init_height = init_height
        self.probe_din = probe_din
        self.probe_ain = probe_ain 
        self.probe_type = probe_type
        self.probe_condition = probe_condition
        self.probe_level = probe_alvl
        self.plate_thk = plate_thk
        self.csk_compensate = csk_compensate
        
    def description(self):
        return 'CounterSink %s' % (self.get_non_z_str())
    
    def set_tool_id(self, tool_id):
        self.tool_id = tool_id
        
    def set_probe_speed(self, speed):
        self.probe_speed = speed
        
    def set_probe_dist(self, dist):
        self.probe_dist = dist
                
    def convert(self, id):
        global configs
        global stat
        gcode_buf = ""

        """ 為何需要 plate_thk? 因為對刀器不是放在工件上對刀，所以需要知道工件厚度 """
        print "home_orig 是Z軸 safe-height，該從 TEACH 設定檔(DB) 來 "
        print "offset 是對刀器高度，該從 TEACH 設定檔(DB) 來 "
        print "TODO: sqlite export 介面 for merging DB settings "
        work_height = (abs(self.csk_compensate) +configs.home_orig) - self.plate_thk + configs.offset

        if (self.tool_id != stat.tool_id):
            """ change to CounterSink Tool """
            cmd = (stat.gcode_pending,
                   "M6 T%s\n" % (self.tool_id),
                   "G10 L2 P1 Z0\n",                    # 將 Z 軸的原點座標設為「機械塵標 0」 
                   "G10 L20 P1 Z%f\n"% (work_height))   # 將換刀完成後的 Z 軸高度設為 work_height 
            gcode_buf = ''.join(cmd)
            stat.tool_id = self.tool_id

        cmd = (gcode_buf,
               "(begin: id[%d], %s)\n" % (id, self.description()),
               "M64P28\n",
               "G04P%f" % (configs.blow_dwell),
               "M65P28\n",
               "G90\n", 
               "G0 X%f Y%f\n" % (self.cur_pos["X"], self.cur_pos["Y"]),
               "G0 Z%f\n" % (self.init_height),
               "M64P27\n",
               "G04P%f" % (configs.oil_dwell),
               "M65P27\n",
               "M111 P%d Q%d R%d J%d K%d\n" % (self.probe_din, self.probe_ain, self.probe_type, self.probe_condition, self.probe_level),
               "G91 ",
               "G4 P0.5 ",
               "F%f G38.2 Z-%f\n" % (self.probe_speed, self.probe_dist),
#                "(debug, id(%d): #5070 #5061 #5062 #5063 #5064 #5065 #5066 #5067 #5068 #5069)\n" % (id),
               "G4P0.1\n", # TODO: confirm if this G4 is necessary?
               "G90 G0 Z[#5063 + %f]\n" % (self.init_height),
               "M3 S%f\n" % (self.csk_rpm),
               "G91 F%f G1 Z%f\n" % (self.csk_speed, (-self.init_height - self.csk_depth)), # making counterSink hole
               "G90 G0 Z%f\n" % (self.init_height),
               "M5\n",
               "G90\n",
               "(end: id[%d], %s)\n" % (id, self.description()),
               )
        print "TODO: confirm do we need G4P0.1 for position sync?"
        return ''.join(cmd)
    

class Tap(Motion):
    def __init__(self, cur_pos, tap_rpm, tap_depth, tap_pitch, tapping_work_height, plate_thk, confirm_din):   # Constructor of the class
        super(Tap,self).__init__(cur_pos)
        self.tap_rpm = tap_rpm
        self.tap_depth = tap_depth
        self.tap_pitch = tap_pitch
        self.tapping_work_height = tapping_work_height
        self.plate_thk = plate_thk
        self.confirm_din = confirm_din        
    def description(self):
        return 'Tap %s' % (self.get_non_z_str())
    def set_tool_id(self, tap_id):
        self.tap_tool_id = tap_id
    def convert(self, id):
        global configs
        global stat
        gcode_buf = ""

        work_height = (configs.home_orig - self.tapping_work_height) - self.plate_thk + configs.offset
            
        if (self.tap_tool_id != stat.tool_id):
            cmd = (stat.gcode_pending,# change to drill tool
                   "M6 T%s\n" % (self.tap_tool_id),
                   "G10 L2 P1 Z0\n",
                   "G10 L20 P1 Z%f\n"% (work_height))
            gcode_buf = ''.join(cmd)
            stat.tool_id = self.tap_tool_id

        """ 1st pass: tap """
        cmd = (gcode_buf,
               "(begin: id[%d], %s, tapping)\n" % (id, self.description()),
               "G90\n", 
               "M64P28\n",
               "G04P%f" % (configs.blow_dwell),
               "M65P28\n",
               "G0 X%f Y%f\n" % (self.cur_pos["X"], self.cur_pos["Y"]),
               "G0 Z%f\n" % (configs.work_height), # TODO: work_height
               "M64P27\n",
               "G04P%f" % (configs.oil_dwell),
               "M65P27\n",
#                       "M66P%dL3Q10000\n" % self.confirm_din,
               "G91\n",
               "F300 G1 Z-0.5",
               "M3 S%f\n" % (self.tap_rpm), # TODO: tap_rpm
               "G33.1 Z-%f K%f\n" % (self.tap_depth, self.tap_pitch),
               "M5\n",
               "G90\n",
               "G0 Z%f\n" % (configs.safe_height), # safe_height               
               "(end: id[%d], tapping)\n" % id
               )
        return ''.join(cmd)
    
class EndOfProg(Motion):
    def __init__(self):   # Constructor of the class
        pass
    def description(self):
        return 'End of Program'
    def convert(self, id):
        buf = "M2\n"
        if (stat.gcode_pending != ""):
            cmd = (stat.gcode_pending,
                   buf)
            buf = ''.join(cmd)
            stat.gcode_pending = ""
        return buf
    
class HandlerClass:
    '''
    class with gladevcp callback handlers
    '''

    def on_ladder_clicked (self, widget, data=None):
        #TODO: cl name from parameter or ini
        os.system('halcmd loadusr -w classicladder &')

    def on_hal_clicked (self, widget, data=None):
        os.system('$EMC2_HOME/tcl/bin/halshow.tcl &')

    def _on_btn_b_change(self,hal_pin,data=None):
        print "_on_btn_b_change() - HAL pin value: %s" % (hal_pin.get())
        if (hal_pin.get() == True):
            self.learner.add_gcode()

    def _hal_setup(self, halcomp, builder):
        '''
        hal related initialisation
        '''
        # hal pins with change callback. Also unrelated to any HAL widget.
        # When the pin's value changes the callback is executed.
        self.btn_b_trigger = hal_glib.GPin(halcomp.newpin('btn-b-trigger', hal.HAL_BIT, hal.HAL_IN))
        self.btn_b_trigger.connect('value-changed', self._on_btn_b_change)        
    
    def get_cur_pos(self):
        s = self.e.s
        s.poll()
        
        p = s.actual_position
        x = p[0] - s.g5x_offset[0] - s.tool_offset[0]
        y = p[1] - s.g5x_offset[1] - s.tool_offset[1]
        z = p[2] - s.g5x_offset[2] - s.tool_offset[2]
        a = p[3] - s.g5x_offset[3] - s.tool_offset[3]
        b = p[4] - s.g5x_offset[4] - s.tool_offset[4]
        c = p[5] - s.g5x_offset[5] - s.tool_offset[5]
        u = p[6] - s.g5x_offset[6] - s.tool_offset[6]
        v = p[7] - s.g5x_offset[7] - s.tool_offset[7]
        w = p[8] - s.g5x_offset[8] - s.tool_offset[8]
        
        if s.rotation_xy != 0:
            t = math.radians(-s.rotation_xy)
            xr = x * math.cos(t) - y * math.sin(t)
            yr = x * math.sin(t) + y * math.cos(t)
            x = xr
            y = yr

        x -= s.g92_offset[0]
        y -= s.g92_offset[1]
        z -= s.g92_offset[2]
        a -= s.g92_offset[3]
        b -= s.g92_offset[4]
        c -= s.g92_offset[5]
        u -= s.g92_offset[6]
        v -= s.g92_offset[7]
        w -= s.g92_offset[8]

        relp = [x, y, z, a, b, c, u, v, w]
        cur_pos = {}
        am = s.axis_mask
        for i in range(0, 9):
            if am & (1<<i):
                letter = 'XYZABCUVW'[i]
                cur_pos[letter] = relp[i]
        return cur_pos

    def get_actual_pos(self):
        s = self.e.s
        s.poll()
        
        p = s.actual_position
        x = p[0]
        y = p[1]
        z = p[2]
        a = p[3]
        b = p[4]
        c = p[5]
        u = p[6]
        v = p[7]
        w = p[8]
        
        if s.rotation_xy != 0:
            t = math.radians(-s.rotation_xy)
            xr = x * math.cos(t) - y * math.sin(t)
            yr = x * math.sin(t) + y * math.cos(t)
            x = xr
            y = yr

        relp = [x, y, z, a, b, c, u, v, w]
        actual_pos = {}
        am = s.axis_mask
        for i in range(0, 9):
            if am & (1<<i):
                letter = 'XYZABCUVW'[i]
                actual_pos[letter] = relp[i]
        return actual_pos
        
        
    def on_disp_point_clicked (self, w=None):
        cur_pos = self.get_cur_pos()
        dp_dout = 15
        dp_delay = 0.5
        lo_speed = 100.0
        lo_dist = 3.0
        clr_dist = 50.0
        newMotion = DispPoint(cur_pos, dp_dout, dp_delay, lo_speed, lo_dist, clr_dist)
        self.motionList.append([newMotion, len(self.motionList), newMotion.description()])
        
    def on_line_start_clicked (self, w=None):
        cur_pos = self.get_cur_pos()
        dp_dout = 15
        dp_delay = 0.5
        lo_speed = 100.0
        lo_dist = 3.0
        clr_dist = 50.0
        newMotion = LineStart(cur_pos, dp_dout, dp_delay, lo_speed, lo_dist, clr_dist)
        self.motionList.append([newMotion, len(self.motionList), newMotion.description()])
        
    def on_drill_clicked (self, w):
        cur_pos = self.get_cur_pos()
        newMotion = Drill(cur_pos,self.drill_rpm, self.drill_feed, self.drill_depth,\
                          self.drill_compensate, self.plate_thk, confirm_din=60)
        self.drill.append(newMotion)
        newMotion.set_tool_id(self.drill_id) # drill_id(1)
        self.motionList.append([newMotion, len(self.motionList), newMotion.description()])

    def on_rigid_tap_clicked (self, w=None):
        cur_pos = self.get_cur_pos()
        newMotion = RigidTap(cur_pos,self.drill_rpm, self.drill_feed, self.drill_depth,\
                            self.drill_compensate, self.tap_rpm, self.tap_depth, self.tap_pitch,\
                            self.tap_compensate, self.plate_thk, confirm_din=60)
        self.rigid_tap.append(newMotion)
        newMotion.set_tool_id(self.drill_id, self.tap_id) # drill_id(1), tap_id(6)
        self.motionList.append([newMotion, len(self.motionList), newMotion.description()])

    def on_tap_clicked (self, w=None):
        cur_pos = self.get_cur_pos()
        newMotion = Tap(cur_pos, self.tap_rpm, self.tap_depth, self.tap_pitch, self.tap_compensate, self.plate_thk, confirm_din=60)
#        self.tap.append(newMotion)
        newMotion.set_tool_id(self.tap_id) # drill_id(1), tap_id(6)
        self.motionList.append([newMotion, len(self.motionList), newMotion.description()])
    
    def on_counterSink_clicked (self, w=None):
        cur_pos = self.get_cur_pos()
        newMotion = CounterSink(cur_pos, self.csk_rpm, self.csk_speed, self.csk_depth, \
                                self.csk_init_h, self.csk_compensate, \
                                self.probe_din, self.probe_ain, self.probe_type, \
                                self.probe_cond, self.probe_alvl, self.plate_thk)
        newMotion.set_tool_id(self.csk_id) # drill_id(1), tap_id(6)
        newMotion.set_probe_speed(self.csk_probe_speed) # drill_id(1), tap_id(6)
        newMotion.set_probe_dist(self.probe_dist)
        self.motionList.append([newMotion, len(self.motionList), newMotion.description()])

    def on_change_tool_clicked(self, w=None):
        cur_pos = self.get_cur_pos()
        newMotion = ChangeTool(cur_pos)
        newMotion.set_tool_id(0) # TODO: 
        self.motionList.append([newMotion, len(self.motionList), newMotion.description()])
        
    def on_end_prog_clicked (self, w=None):
        newMotion = EndOfProg()
        self.motionList.append([newMotion, len(self.motionList), newMotion.description()])
        
    def on_del_select_clicked (self, w):
        selection = self.motionView.get_selection()
        model, iter = selection.get_selected()
        if model: #result could be None
            model.remove(iter)
        # re-calculate "Id" column
        index = 0
        for motion in self.motionList:
            motion[self.cId] = index # update with new "Id"
            index = index + 1
    
    def on_convert_clicked (self, w):
        self.g_code = ''
#        self.update_params()
        for motion in self.motionList:
            id = motion[self.cId]
            self.g_code = "".join((self.g_code, motion[self.cMotionObject].convert(id), '\n'))
            print self.g_code
        # a = self.builder.get_object('hal_action_open1')
        teached_g_code = os.path.join(os.path.expanduser("../nc_files"), "teached.ngc")
        fout = open(teached_g_code,'w')
        fout.writelines(self.g_code)
        fout.close()
        # a.fixed_file = teached_g_code
        # a.activate()
        
        stat.tool_id = 0

    def on_clear_all_clicked (self, w, d=None):
        self.motionList.clear()

    def on_clear_plot_clicked(self, w, data=None):
        self.builder.get_object("hal_gremlin1").logger.clear()

    def on_load_clicked(self, widget):
        """Called when the user wants to open a wine"""

        # Get the file to open
        open_file = self.file_browse(gtk.FILE_CHOOSER_ACTION_OPEN)
        if (open_file != ""):
            # We have a path, open it for reading
            db = shelve.open(open_file,"r")
            try:
                # We have opened the file, so empty out our gtk.TreeView
                self.motionList.clear()
                """ Since the shelve file is not gaurenteed to be in order we
                move through the file starting at iter 0 and moving our
                way up"""
                count = 0;
                while db.has_key(str(count)):
                    newMotion = db[str(count)]
                    self.motionList.append([newMotion, count, newMotion.description()])
                    self.drill.append(newMotion)
                    # try:
                    #     self.drill.append(newMotion)
                    # except:
                    #     self.rigid_tap.append(newMotion)
                    count = count +1
                db.close();
                #set the project file
                root, self.project_file = os.path.split(open_file)
            except:
                self.show_error_dlg("Error processing file: " + open_file)
        else:
            self.show_error_dlg("EMPTY FILE NAME")

    def on_save_clicked(self, widget):
        """Called when the user wants to save a motion list"""

        # Get the File Save path
        save_file = self.file_browse(gtk.FILE_CHOOSER_ACTION_SAVE, self.project_file)
        if (save_file != ""):
            # We have a path, ensure the proper extension
            save_file, extension = os.path.splitext(save_file)
            save_file = save_file + "." + self.FILE_EXT
            """ Now we have the "real" file save loction create
            the shelve file, use "n" to create a new file"""
            db = shelve.open(save_file,"n")
            """Get the first item in the gtk.ListStore, and while it is not
            None, move forwqard through the list saving each item"""
            # Get the first item in the list
            iter = self.motionList.get_iter_root()
            while (iter):
                # Get the wine at the current gtk.TreeIter
                motion = self.motionList.get_value(iter, self.cMotionObject)
                # Use the iters position in the list as the key name
                db[self.motionList.get_string_from_iter(iter)] = motion
                # Get the next iter
                iter = self.motionList.iter_next(iter)
            #close the database and write changes to disk, we are done
            db.close();
            #set the project file
            root, self.project_file = os.path.split(save_file)
            
            ls = save_file.split('/')
            name = ls[len(ls)-1]
            filename = name.split('.')
            teached_g_code = os.path.join(os.path.expanduser("../nc_files"), "%s.ngc"%filename[0])
            fout = open(teached_g_code,'w')
            fout.writelines(self.g_code)
            fout.close()
            
    def file_browse(self, dialog_action, file_name=""):
        """This function is used to browse for a teach file.
        It can be either a save or open dialog depending on
        what dialog_action is.
        The path to the file will be returned if the user
        selects one, however a blank string will be returned
        if they cancel or do not select one.
        dialog_action - The open or save mode for the dialog either
        gtk.FILE_CHOOSER_ACTION_OPEN, gtk.FILE_CHOOSER_ACTION_SAVE"""

        if (dialog_action==gtk.FILE_CHOOSER_ACTION_OPEN):
            dialog_buttons = (gtk.STOCK_CANCEL
                                , gtk.RESPONSE_CANCEL
                                , gtk.STOCK_OPEN
                                , gtk.RESPONSE_OK)
        else:
            dialog_buttons = (gtk.STOCK_CANCEL
                                , gtk.RESPONSE_CANCEL
                                , gtk.STOCK_SAVE
                                , gtk.RESPONSE_OK)

        file_dialog = gtk.FileChooserDialog(title="Select Project"
                    , action=dialog_action
                    , buttons=dialog_buttons)
        """set the filename if we are saving"""
        if (dialog_action==gtk.FILE_CHOOSER_ACTION_SAVE):
            file_dialog.set_current_name(file_name)
        """Create and add the pywine filter"""
        filter = gtk.FileFilter()
        filter.set_name("teached files")
        filter.add_pattern("*." + self.FILE_EXT)
        file_dialog.add_filter(filter)
        """Create and add the 'all files' filter"""
        filter = gtk.FileFilter()
        filter.set_name("All files")
        filter.add_pattern("*")
        file_dialog.add_filter(filter)

        """Init the return value"""
        result = ""
        if file_dialog.run() == gtk.RESPONSE_OK:
            result = file_dialog.get_filename()
        file_dialog.destroy()

        return result

    def show_error_dlg(self, error_string):
        """This Function is used to show an error dialog when
        an error occurs.
        error_string - The error string that will be displayed
        on the dialog.
        """
        error_dlg = gtk.MessageDialog(type=gtk.MESSAGE_ERROR
                    , message_format=error_string
                    , buttons=gtk.BUTTONS_OK)
        error_dlg.run()
        error_dlg.destroy()


    def AddMotionListColumn(self, title, columnId):
        """This function adds a column to the list view.
        First it create the gtk.TreeViewColumn and then set
        some needed properties"""

        column = gtk.TreeViewColumn(title, gtk.CellRendererText()
            , text=columnId)
        column.set_resizable(True)
        column.set_sort_column_id(columnId)
        self.motionView.append_column(column)
        self.column = column

    def on_tap_pitch_changed(self,w):
        self.tap_pitch = float(self.builder.get_object("tap_pitch").get_text())
        self.sqlCursor.execute("UPDATE tools SET tap_pitch = ? WHERE tool_no = ?", (self.tap_pitch, self.tap_id))
        self.conn.commit()  # save to db file
        
    def on_tap_depth_changed(self,w):
        self.tap_depth = float(self.builder.get_object("tap_depth").get_text())
        self.sqlCursor.execute("UPDATE tools SET tap_depth = ? WHERE tool_no = ?", (self.tap_depth, self.tap_id))
        self.conn.commit()  # save to db file
        
    def on_tap_rpm_changed(self,w):
        self.tap_rpm = float(self.builder.get_object("tap_rpm").get_text())
        self.sqlCursor.execute("UPDATE tools SET tap_rpm = ? WHERE tool_no = ?", (self.tap_rpm, self.tap_id))
        self.conn.commit()  # save to db file
        
    def on_drill_depth_changed(self,w):
        self.drill_depth = float(self.builder.get_object("drill_depth").get_text())
        self.sqlCursor.execute("UPDATE tools SET drill_depth = ? WHERE tool_no = ?", (self.drill_depth, self.drill_id))
        self.conn.commit()  # save to db file
        
    def on_drill_feed_changed(self,w):
        self.drill_feed = float(self.builder.get_object("drill_feed").get_text())
        self.sqlCursor.execute("UPDATE tools SET drill_feed = ? WHERE tool_no = ?", (self.drill_feed, self.drill_id))
        self.conn.commit()  # save to db file
                
    def on_drill_rpm_changed(self,w):
        self.drill_rpm = float(self.builder.get_object("drill_rpm").get_text())
        self.sqlCursor.execute("UPDATE tools SET drill_rpm = ? WHERE tool_no = ?", (self.drill_rpm, self.drill_id))
        self.conn.commit()  # save to db file
        
    def on_csk_rpm_changed(self,w):
        self.csk_rpm = float(self.builder.get_object("csk_rpm").get_text())
        self.sqlCursor.execute("UPDATE tools SET csk_rpm = ? WHERE tool_no = ?", (self.csk_rpm, self.csk_id))
        self.conn.commit()  # save to db file

    def on_csk_depth_changed(self,w):
        self.csk_depth = float(self.builder.get_object("csk_depth").get_text())
        self.sqlCursor.execute("UPDATE tools SET csk_depth = ? WHERE tool_no = ?", (self.csk_depth, self.csk_id))
        self.conn.commit()  # save to db file

    def on_csk_feed_changed(self,w):
        self.csk_speed = float(self.builder.get_object("csk_feed").get_text())
        self.sqlCursor.execute("UPDATE tools SET csk_speed = ? WHERE tool_no = ?", (self.csk_speed, self.csk_id))
        self.conn.commit()  # save to db file
 
    def on_csk_probe_feed_changed(self,w):
        self.csk_probe_speed = float(self.builder.get_object("csk_probe_feed").get_text())
        self.sqlCursor.execute("UPDATE tools SET csk_probe_speed = ? WHERE tool_no = ?", (self.csk_probe_speed, self.csk_id))
        self.conn.commit()  # save to db file
        
    def on_csk_init_h_changed(self,w):
        self.csk_init_h = float(self.builder.get_object("csk_init_h").get_text())
        self.sqlCursor.execute("UPDATE tools SET csk_init_h = ? WHERE tool_no = ?", (self.csk_init_h, self.csk_id))
        self.conn.commit()  # save to db file
          
    def on_probe_dist_changed(self,w):
        self.probe_dist = float(self.builder.get_object("probe_dist").get_text())
        self.sqlCursor.execute("UPDATE tools SET probe_dist = ? WHERE tool_no = ?", (self.probe_dist, self.csk_id))
        self.conn.commit()  # save to db file
       
    def on_probe_din_changed(self,w):
        self.probe_din = float(self.builder.get_object("probe_din").get_text())
        self.sqlCursor.execute("UPDATE tools SET probe_din = ? WHERE tool_no = ?", (self.probe_din, self.csk_id))
        self.conn.commit()  # save to db file
        
    def on_probe_ain_changed(self,w):
        self.probe_ain = float(self.builder.get_object("probe_ain").get_text())
        self.sqlCursor.execute("UPDATE tools SET probe_ain = ? WHERE tool_no = ?", (self.probe_ain, self.csk_id))
        self.conn.commit()  # save to db file
        
    def on_probe_type_changed(self,w):
        self.probe_type = float(self.builder.get_object("probe_type").get_text())
        self.sqlCursor.execute("UPDATE tools SET probe_type = ? WHERE tool_no = ?", (self.probe_type, self.csk_id))
        self.conn.commit()  # save to db file
        
    def on_probe_cond_changed(self,w):
        self.probe_cond = float(self.builder.get_object("probe_cond").get_text())
        self.sqlCursor.execute("UPDATE tools SET probe_cond = ? WHERE tool_no = ?", (self.probe_cond, self.csk_id))
        self.conn.commit()  # save to db file
        
    def on_probe_lvl_changed(self,w):
        self.probe_alvl = float(self.builder.get_object("probe_lvl").get_text())
        self.sqlCursor.execute("UPDATE tools SET probe_alvl = ? WHERE tool_no = ?", (self.probe_alvl, self.csk_id))
        self.conn.commit()  # save to db file
                 
    def on_tap_id_changed(self,w):
        self.tap_id = (self.builder.get_object("tap_id").get_text())
        self.sqlCursor.execute("SELECT tap_rpm,tap_depth,tap_pitch,tap_compensate FROM tools WHERE tool_no = ?", (self.tap_id,))
        (self.tap_rpm, self.tap_depth, self.tap_pitch, self.tap_compensate) = self.sqlCursor.fetchone()
#        print "tap_rpm:(%f) tap_depth:(%f) tap_pitch:(%f)" % (self.tap_rpm, self.tap_depth, self.tap_pitch)
        self.builder.get_object('tap_rpm').set_text(str(self.tap_rpm))
        self.builder.get_object('tap_depth').set_text(str(self.tap_depth))
        self.builder.get_object('tap_pitch').set_text(str(self.tap_pitch))
        self.ini.save_state(self)
        
    def on_drill_id_changed(self,w):
        self.drill_id = (self.builder.get_object("drill_id").get_text())
        self.sqlCursor.execute("SELECT drill_rpm,drill_depth,drill_feed,drill_compensate FROM tools WHERE tool_no = ?", (self.drill_id,))
        (self.drill_rpm, self.drill_depth, self.drill_feed,self.drill_compensate) = self.sqlCursor.fetchone()
#        print "drill_rpm:(%f) drill_depth:(%f) drill_feedd:(%f)" % (self.drill_rpm, self.drill_depth, self.drill_feed)
        self.builder.get_object('drill_rpm').set_text(str(self.drill_rpm))
        self.builder.get_object('drill_depth').set_text(str(self.drill_depth))
        self.builder.get_object('drill_feed').set_text(str(self.drill_feed))
        self.ini.save_state(self)
        
    def on_csk_id_changed(self,w):
        self.csk_id = (self.builder.get_object("csk_id").get_text())
        self.sqlCursor.execute("SELECT csk_rpm,csk_depth,csk_init_h,csk_speed,csk_probe_speed,probe_din,probe_ain,probe_type,probe_cond,probe_alvl,csk_compensate,probe_dist FROM tools WHERE tool_no = ?", (self.csk_id,))
        (self.csk_rpm, self.csk_depth, self.csk_init_h, self.csk_speed, self.csk_probe_speed, self.probe_din, self.probe_ain, self.probe_type, self.probe_cond, self.probe_alvl,self.csk_compensate,self.probe_dist) = self.sqlCursor.fetchone()
        self.builder.get_object('csk_rpm').set_text(str(self.csk_rpm))
        self.builder.get_object('csk_depth').set_text(str(self.csk_depth))
        self.builder.get_object('csk_init_h').set_text(str(self.csk_init_h))
        self.builder.get_object('csk_feed').set_text(str(self.csk_speed))        
        self.builder.get_object('csk_probe_feed').set_text(str(self.csk_probe_speed))        
        self.builder.get_object('probe_din').set_text(str(self.probe_din))        
        self.builder.get_object('probe_ain').set_text(str(self.probe_ain))        
        self.builder.get_object('probe_type').set_text(str(self.probe_type))        
        self.builder.get_object('probe_cond').set_text(str(self.probe_cond))        
        self.builder.get_object('probe_lvl').set_text(str(self.probe_alvl))        
        self.builder.get_object('probe_dist').set_text(str(self.probe_dist))        

        self.ini.save_state(self)    
#    def update_params(self):
#        for a in self.rigid_tap:
#            a.tap_pitch = self.tap_pitch
#            a.tap_depth = self.tap_depth
#            a.tap_rpm = self.tap_rpm
#            a.drill_depth = self.drill_depth
#            a.drill_feed = self.drill_feed
#            a.drill_rpm = self.drill_rpm

    def on_plate_thk_button_press_event(self,w=None,d=None):
        actual_pos = self.get_actual_pos()
        plate_height = actual_pos["Z"]
        tool = self.e.current_tool()
        if tool == self.drill_id:
            comp = self.drill_compensate
        elif  tool == self.tap_id:
            comp = self.tap_compensate
        elif tool == self.csk_id:
            comp = self.csk_compensate
        self.plate_thk = abs(plate_height - comp)
        self.builder.get_object('p_thk').set_text('%.2f'%self.plate_thk)
        
    def on_btn_drill_touch_off_button_press_event(self,w,d):
        actual_pos = self.get_actual_pos()
        self.drill_compensate = actual_pos["Z"]
        self.sqlCursor.execute("UPDATE tools SET drill_compensate = ? WHERE tool_no = ?", (self.drill_compensate, self.drill_id))
        self.conn.commit()  # save to db file
#        print self.drill_compensate
        return self.drill_compensate
        
    def on_btn_tapping_touch_off_button_press_event(self,w,d):
        actual_pos = self.get_actual_pos()
        self.tap_compensate = actual_pos["Z"]
        self.sqlCursor.execute("UPDATE tools SET tap_compensate = ? WHERE tool_no = ?", (self.tap_compensate, self.tap_id))
        self.conn.commit()  # save to db file
#        print self.tap_compensate
        return self.tap_compensate
        
    def on_btn_csk_touch_off_button_press_event(self,w,d):
        actual_pos = self.get_actual_pos()
        self.csk_compensate = actual_pos["Z"]
        self.sqlCursor.execute("UPDATE tools SET csk_compensate = ? WHERE tool_no = ?", (self.csk_compensate, self.csk_id))
        self.conn.commit()  # save to db file
#        print self.tap_compensate
        return self.csk_compensate     
    def on_mdi_go_button_press_event(self,w=None,d=None):
        x_pos = self.builder.get_object('x_pos').get_text()
        y_pos = self.builder.get_object('y_pos').get_text()
        cmd = "G90G0X%sY%s" % (x_pos, y_pos)
        print "X(%s) Y(%s)" % (x_pos, y_pos)
        self.socket.send("teach.mdi.%s" % cmd, zmq.NOBLOCK)
        
    def on_mdi_go_z_button_press_event(self,w=None,d=None):
        print"TODO: if x in dangerous pos return"
#        actual_pos = self.get_actual_pos()
#         x_pos = actual_pos["X"]
#         if (x_pos < -700):
#             return
        self.z_pos = self.builder.get_object('z_pos').get_text()
        cmd = "G53G90G0Z%s" % (self.z_pos)
        print "Z(%s)" % (self.z_pos)
        self.ini.save_state(self)
        self.socket.send("teach.mdi.%s" % cmd, zmq.NOBLOCK)

    def on_mdi_go_tool_change_button_press_event(self,w=None,d=None):
        tool = self.builder.get_object('entry_tool').get_text()
        if (tool not in "0123456789") and tool > 10:
#            print "NUM ERROR"
            return "NUM ERROR"
        cmd = "M6T%s" % (tool)
        self.socket.send("teach.mdi.%s" % cmd, zmq.NOBLOCK)
#         if tool != '0':
#             self.builder.get_object("tap_id").set_text(tool)
#             self.builder.get_object("drill_id").set_text(tool)
#             self.builder.get_object("csk_id").set_text(tool)
#             self.on_tap_id_changed(w=None)
#             self.on_drill_id_changed(w=None)
#             self.on_csk_id_changed(w=None)
        return cmd
            
    def on_zero_button_press_event(self,w=None,d=None):
        cmd = "G10L20P1X0Y0"
        self.socket.send("teach.mdi.%s" % cmd, zmq.NOBLOCK)
    
    def _init_sql(self, db):
        if(os.path.exists(db)):
            self.conn = sqlite3.connect(db)
            self.sqlCursor = self.conn.cursor()
        else:
            self.conn = sqlite3.connect(db)
            self.sqlCursor = self.conn.cursor()
            sqlf = open('scripts/reset_teach.sql', 'r')
            self.sqlCursor.executescript(sqlf.read())
            sqlf = open('scripts/init_teach.sql', 'r')
            self.sqlCursor.executescript(sqlf.read())

    def update_current_tool(self):
        tool = self.e.current_tool()
        self.builder.get_object('curr_tool').set_text('%d' % tool)

    def _on_update_status(self):
        self.update_current_tool()
        self.count += 1
        # if (self.count == 5):
        #     self.set_dynamic_config_tabs()
        return True
        
    def set_theme_font(self):
        config = ConfigParser.ConfigParser()
        fn = os.path.expanduser("./.touchy_preferences")
        if (os.path.exists(fn)):
            config.read(fn) 
        else:
            print "WARN: cannot find ./.touchy_preferences"
            return
        o = config.get("DEFAULT", 'control_font')
        
        control_font = pango.FontDescription(o)
        
        # control_font = pango.FontDescription("San 24")

        # set spin label font
        for i in ["label1", "label4", "label3",
                  "label2","curr_tool"]:
            w = self.builder.get_object(i)
            if w:
#                 w = w.child
                w.modify_font(control_font)
         
        # set ui theme
        theme_name = config.get("DEFAULT", 'gtk_theme')
        settings = gtk.settings_get_default()
        settings.set_string_property("gtk-theme-name", theme_name, "")

    def set_dynamic_config_tabs(self):
        from subprocess import Popen
        tab_names = ['CAMERA']
        try:
            if(self.tisopts):
                tab_cmd   = ['./tiscam.py -x {XID} -d %s -f %s' % (self.tisopts[0], self.tisopts[1])]        
        except:
            tab_cmd   = ['python tiscam.py -x {XID} -d /dev/videoTIS']
#         tab_cmd   = ['./tiscam.py -x {XID} -d %s -f %s' % (self.tisopts[0], self.tisopts[1])]
            
        if len(tab_names) != len(tab_cmd):
            print "Invalid config-tab configuration" # Complain somehow

        nb = self.builder.get_object('nb_webcam')
        xid = None
        for t,c in zip(tab_names, tab_cmd):
            xid = self._dynamic_tab(nb, t)
            if not xid: continue
            cmd = c.replace('{XID}', str(xid))
            child = Popen(cmd.split())
            self._dynamic_childs[xid] = child
            # the child has to terminate when main window is terminated
            
        if nb:
            nb.show_all()
            # let the default page be judged by NBCFG_PAGE
    def _dynamic_tab(self, notebook, text):
        s = gtk.Socket()
        label = gtk.Label(text)
#         config = ConfigParser.ConfigParser()
#         fn = os.path.expanduser(".touchy_preferences")
#         config.read(fn) 
#         l = config.get("DEFAULT", 'listing_font')
#         font = pango.FontDescription(l)
#         label.modify_font(font)
        notebook.append_page(s, label)
        return s.get_id()
    
    def treeview_changed(self, widget, event, data=None):
        self.scrolled_window = self.builder.get_object("scrolledwindow1")
        adj = self.scrolled_window.get_vadjustment()
        adj.set_value( adj.upper - adj.page_size )

    def __init__(self, halcomp,builder,useropts):
        '''
        Handler classes are instantiated in the following state:
        - the widget tree is created, but not yet realized (no toplevel window.show() executed yet)
        - the halcomp HAL component is set up and the widhget tree's HAL pins have already been added to it
        - it is safe to add more hal pins because halcomp.ready() has not yet been called at this point.

        after all handlers are instantiated in command line and get_handlers() order, callbacks will be
        connected with connect_signals()/signal_autoconnect()

        The builder may be either of libglade or GtkBuilder type depending on the glade file format.
        '''
        self.halcomp = halcomp
        self.builder = builder
        self._hal_setup(halcomp,builder)
        self.window = self.builder.get_object('window1')
        self.e = EmcInterface()
            
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUSH)
        self.socket.connect('tcp://127.0.0.1:5555')
        self._dynamic_childs = {}

        if (useropts):
            ''' 透過 -U 傳遞 useropts 參數 '''
            # for example: execute "self.sim = True"
            for cmd in useropts:
                exec cmd
#         self.drawingarea = gtk.DrawingArea()
#         self.drawingarea.set_size_request(1024, 600)
#         self.sw = gtk.ScrolledWindow()
#         self.sw.add_with_viewport(self.drawingarea)
#         self.table = self.builder.get_object('table6')
#         self.table.attach(self.sw, 2, 5, 0, 1)

    
#        if (useropts == []):
#            self.conn = sqlite3.connect("teach.db")
#        else:
#            self.conn = sqlite3.connect(useropts)
#        self.sqlCursor = self.conn.cursor()
        
        #Here are some variables that can be reused later
        self.cMotionObject = 0
        self.cId = 1
        self.cMotion = 2

        self.sId = "Id"
        self.sMotion = "Motion"
        self.FILE_EXT = "teach"
        
        #Get the treeView from the widget Tree
        self.motionView = self.builder.get_object("motionView")
        #Add all of the List Columns to the wineView
        self.AddMotionListColumn(self.sId, self.cId)
        self.AddMotionListColumn(self.sMotion, self.cMotion)
        #Create the listStore Model to use with the wineView
        self.motionList = gtk.ListStore(gobject.TYPE_PYOBJECT,
                                        gobject.TYPE_INT,
                                        gobject.TYPE_STRING)
        #Attache the model to the treeView
        self.motionView.set_model(self.motionList)
        self.project_file = ""
        self.ini_filename = __name__+ '.var'  # for injector.var
        self.defaults = {  # these will be saved/restored as method attributes
                    IniFile.vars: { #'q3_particle_level' : 13.0,
                    'tap_id':1,
                    'drill_id':1,
                    'csk_id':1,
                    'z_pos':-43.0},
                    # we're interested restoring state to output HAL widgets only
                    # NB: this does NOT restore state pf plain gtk objects - set hal_only to False to do this
                    IniFile.widgets: widget_defaults(select_widgets(self.builder.get_objects(), hal_only=True,output_only = True)),
               }
        self.ini = IniFile(self.ini_filename,self.defaults, self.builder)
        self.ini.restore_state(self)
        self._init_sql("teach.db")
        self.sqlCursor.execute("SELECT tap_rpm,tap_depth,tap_pitch,tap_compensate FROM tools WHERE tool_no = ?", (self.tap_id,))
        (self.tap_rpm, self.tap_depth, self.tap_pitch, self.tap_compensate) = self.sqlCursor.fetchone()
        self.builder.get_object('tap_id').set_text(str(self.tap_id))
        self.builder.get_object('tap_rpm').set_text(str(self.tap_rpm))
        self.builder.get_object('tap_depth').set_text(str(self.tap_depth))
        self.builder.get_object('tap_pitch').set_text(str(self.tap_pitch))
        
        self.sqlCursor.execute("SELECT drill_rpm,drill_depth,drill_feed,drill_compensate FROM tools WHERE tool_no = ?", (self.drill_id,))
        (self.drill_rpm, self.drill_depth, self.drill_feed, self.drill_compensate) = self.sqlCursor.fetchone()
        self.builder.get_object('drill_id').set_text(str(self.drill_id))
        self.builder.get_object('drill_rpm').set_text(str(self.drill_rpm))
        self.builder.get_object('drill_depth').set_text(str(self.drill_depth))
        self.builder.get_object('drill_feed').set_text(str(self.drill_feed))

        self.sqlCursor.execute("SELECT csk_rpm,csk_depth,csk_init_h,csk_speed,csk_probe_speed,probe_din,probe_ain,probe_type,probe_cond,probe_alvl,csk_compensate,probe_dist FROM tools WHERE tool_no = ?", (self.csk_id,))
        (self.csk_rpm, self.csk_depth, self.csk_init_h, self.csk_speed, self.csk_probe_speed, self.probe_din, self.probe_ain, self.probe_type, self.probe_cond, self.probe_alvl,self.csk_compensate,self.probe_dist) = self.sqlCursor.fetchone()
        self.builder.get_object('csk_id').set_text(str(self.csk_id))
        self.builder.get_object('csk_rpm').set_text(str(self.csk_rpm))
        self.builder.get_object('csk_depth').set_text(str(self.csk_depth))
        self.builder.get_object('csk_init_h').set_text(str(self.csk_init_h))
        self.builder.get_object('csk_feed').set_text(str(self.csk_speed))        
        self.builder.get_object('csk_probe_feed').set_text(str(self.csk_probe_speed))        
        self.builder.get_object('probe_din').set_text(str(self.probe_din))        
        self.builder.get_object('probe_ain').set_text(str(self.probe_ain))        
        self.builder.get_object('probe_type').set_text(str(self.probe_type))        
        self.builder.get_object('probe_cond').set_text(str(self.probe_cond))        
        self.builder.get_object('probe_lvl').set_text(str(self.probe_alvl))        
        self.builder.get_object('probe_dist').set_text(str(self.probe_dist))        

        self.plate_thk = 0.0
        self.builder.get_object('p_thk').set_text('%.2f'%self.plate_thk)
        self.builder.get_object('tap_pitch').set_text(str(self.tap_pitch))
        self.builder.get_object('z_pos').set_text(str(self.z_pos))

#         self.set_dynamic_config_tabs()

#        self.tap_pitch = float(self.builder.get_object("tap_pitch").get_text())
#        self.tap_depth = float(self.builder.get_object("tap_depth").get_text())
#        self.tap_rpm = float(self.builder.get_object("tap_rpm").get_text())
#        self.drill_depth = float(self.builder.get_object("drill_depth").get_text())
#        self.drill_feed = float(self.builder.get_object("drill_feed").get_text())
#        self.drill_rpm = float(self.builder.get_object("drill_rpm").get_text())
        self.drill = []
        self.rigid_tap = []
#        self.tap = []
#         self.builder.get_object('scrolledwindow2').hide()
#        configs.drill_work_height = self.drill_compensate
#        configs.tapping_work_height = self.tap_compensate
        self.set_theme_font()
        self.motionView.connect('size-allocate', self.treeview_changed)
        
        self.count = 0
        glib.timeout_add(100, self._on_update_status)

        
def get_handlers(halcomp,builder,useropts,compname):
    '''
    this function is called by gladevcp at import time (when this module is passed with '-u <modname>.py')

    return a list of object instances whose methods should be connected as callback handlers
    any method whose name does not begin with an underscore ('_') is a  callback candidate

    the 'get_handlers' name is reserved - gladevcp expects it, so do not change
    '''

#     global debug
#     for cmd in useropts:
#         exec cmd in globals()
# 
#     set_debug(debug)

    return [HandlerClass(halcomp,builder,useropts)]
