#!/usr/bin/env python
# -*- coding: UTF-8 -*

from machinekit import hal
from machinekit import rtapi as rt
from machinekit import config as c

import rcomps
import motion


def usrcomp_status(compname, signame, thread, resetSignal='estop-reset'):
    sigIn = hal.newsig('%s-error-in' % signame, hal.HAL_BIT)
    sigOut = hal.newsig('%s-error' % signame, hal.HAL_BIT)
    sigOk = hal.newsig('%s-ok' % signame, hal.HAL_BIT)

    sigIn.link('%s.error' % compname)

    safetyLatch = rt.newinst('safety_latch', 'safety-latch.%s-error' % signame)
    hal.addf(safetyLatch.name, thread)
    safetyLatch.pin('error-in').link(sigIn)
    safetyLatch.pin('error-out').link(sigOut)
    safetyLatch.pin('reset').link(resetSignal)
    safetyLatch.pin('threshold').set(500)  # 500ms error
    safetyLatch.pin('latching').set(True)

    notComp = rt.newinst('not', 'not.%s-no-error' % signame)
    hal.addf(notComp.name, thread)
    notComp.pin('in').link(sigOut)
    notComp.pin('out').link(sigOk)


def usrcomp_watchdog(comps, enableSignal, thread,
                     okSignal=None, errorSignal=None):
    count = len(comps)
    watchdog = rt.loadrt('watchdog', num_inputs=count)
    hal.addf('watchdog.set-timeouts', thread)
    hal.addf('watchdog.process', thread)
    for n, comp in enumerate(comps):
        compname = comp[0]
        comptime = comp[1]
        sigIn = hal.newsig('%s-watchdog' % compname, hal.HAL_BIT)
        hal.Pin('%s.watchdog' % compname).link(sigIn)
        watchdog.pin('input-%i' % n).link(sigIn)
        watchdog.pin('timeout-%i' % n).set(comptime)
    watchdog.pin('enable-in').link(enableSignal)

    if not okSignal:
        okSignal = hal.newsig('watchdog-ok', hal.HAL_BIT)
    watchdog.pin('ok-out').link(okSignal)

    if errorSignal:
        notComp = rt.newinst('not', 'not.watchdog-error')
        hal.addf(notComp.name, thread)
        notComp.pin('in').link(okSignal)
        notComp.pin('out').link(errorSignal)


def setup_stepper(stepgenIndex, section, axisIndex=None,
                  stepgenType='hpg.stepgen', gantry=False,
                  gantryJoint=0, velocitySignal=None, thread=None):
    hasStepgen = False
    if (stepgenType != 'sim'):
        stepgen = '%s.%02i' % (stepgenType, stepgenIndex)
        hasStepgen = True
    if axisIndex is not None:
        axis = 'axis.%i' % axisIndex
    hasMotionAxis = (axisIndex is not None) and (not gantry or gantryJoint == 0)
    velocityControlled = velocitySignal is not None

    # axis enable chain
    enableIndex = axisIndex
    if axisIndex is None:
        enableIndex = 0  # use motor enable signal
    enable = hal.Signal('emcmot-%i-enable' % enableIndex, hal.HAL_BIT)
    if hasMotionAxis:
        enable.link('%s.amp-enable-out' % axis)
    if hasStepgen:
        enable.link('%s.enable' % stepgen)

    # position command and feedback
    if not velocityControlled:
        if hasMotionAxis:  # per axis fb and cmd
            posCmd = hal.newsig('emcmot-%i-pos-cmd' % axisIndex, hal.HAL_FLOAT)
            posCmd.link('%s.motor-pos-cmd' % axis)
            if hasStepgen:
                if not gantry:
                    posCmd.link('%s.position-cmd' % stepgen)
                else:
                    posCmd.link('gantry.%i.position-cmd' % axisIndex)
            else:
                posCmd.link('%s.motor-pos-fb' % axis)


            if hasStepgen:
                posFb = hal.newsig('emcmot-%i-pos-fb' % axisIndex, hal.HAL_FLOAT)
                posFb.link('%s.motor-pos-fb' % axis)
                if not gantry:
                    posFb.link('%s.position-fb' % stepgen)
                else:
                    posFb.link('gantry.%i.position-fb' % axisIndex)


        if gantry:  # per joint fb and cmd
            posCmd = hal.newsig('emcmot-%i-%i-pos-cmd' % (axisIndex, gantryJoint), hal.HAL_FLOAT)
            posCmd.link('gantry.%i.joint.%02i.pos-cmd' % (axisIndex, gantryJoint))
            if hasStepgen:
                posCmd.link('%s.position-cmd' % stepgen)
 
            posFb = hal.newsig('emcmot-%i-%i-pos-fb' % (axisIndex, gantryJoint), hal.HAL_FLOAT)
            posFb.link('gantry.%i.joint.%02i.pos-fb' % (axisIndex, gantryJoint))
            if hasStepgen:
                posFb.link('%s.position-fb' % stepgen)

    else:  # velocity control
        print "ERROR: not support velocity control yet"
#         hal.net(velocitySignal, '%s.velocity-cmd' % stepgen)

    # limits
    if hasMotionAxis:
        limitHome = hal.newsig('limit-%i-home' % axisIndex, hal.HAL_BIT)
        limitMin = hal.newsig('limit-%i-min' % axisIndex, hal.HAL_BIT)
        limitMax = hal.newsig('limit-%i-max' % axisIndex, hal.HAL_BIT)
        limitHome.link('%s.home-sw-in' % axis)
        limitMin.link('%s.neg-lim-sw-in' % axis)
        limitMax.link('%s.pos-lim-sw-in' % axis)

#     if gantry:
#         if gantryJoint == 0:
#             axisHoming = hal.newsig('emcmot-%i-homing' % axisIndex, hal.HAL_BIT)
#             axisHoming.link('%s.homing' % axis)
# 
#             hal.Pin('gantry.%i.search-vel' % axisIndex).set(c.find(section, 'HOME_SEARCH_VEL'))
#             hal.Pin('gantry.%i.homing' % axisIndex).link(axisHoming)
#             hal.Pin('gantry.%i.home' % axisIndex).link(limitHome)
# 
#             or2 = rt.newinst('or2', 'or2.limit-%i-min' % axisIndex)
#             hal.addf(or2.name, thread)
#             or2.pin('out').link(limitMin)
# 
#             or2 = rt.newinst('or2', 'or2.limit-%i-max' % axisIndex)
#             hal.addf(or2.name, thread)
#             or2.pin('out').link(limitMax)
# 
#         limitHome = hal.newsig('limit-%i-%i-home' % (axisIndex, gantryJoint),
#                                hal.HAL_BIT)
#         limitMin = hal.newsig('limit-%i-%i-min' % (axisIndex, gantryJoint),
#                               hal.HAL_BIT)
#         limitMax = hal.newsig('limit-%i-%i-max' % (axisIndex, gantryJoint),
#                               hal.HAL_BIT)
#         homeOffset = hal.Signal('home-offset-%i-%i' % (axisIndex, gantryJoint),
#                                 hal.HAL_FLOAT)
#         limitHome.link('gantry.%i.joint.%02i.home' % (axisIndex, gantryJoint))
#         limitMin.link('or2.limit-%i-min.in%i' % (axisIndex, gantryJoint))
#         limitMax.link('or2.limit-%i-max.in%i' % (axisIndex, gantryJoint))
#         homeOffset.link('gantry.%i.joint.%02i.home-offset' % (axisIndex, gantryJoint))

#         storage.setup_gantry_storage(axisIndex, gantryJoint)

    # stepper pins configured in hardware setup


def setup_stepper_multiplexer(stepgenIndex, sections, selSignal, thread):
    num = len(sections)
    sigBase = 'stepgen-%i' % stepgenIndex

    unsignedSignals = [['dirsetup', 'DIRSETUP'],
                       ['dirhold', 'DIRHOLD'],
                       ['steplen', 'STEPLEN'],
                       ['stepspace', 'STEPSPACE']]

    floatSignals = [['scale', 'SCALE'],
                    ['max-vel', 'STEPGEN_MAX_VEL'],
                    ['max-acc', 'STEPGEN_MAX_ACC']]

    for item in unsignedSignals:
        signal = hal.Signal('%s-%s' % (sigBase, item[0]), hal.HAL_U32)
        mux = rt.newinst('muxn_u32', 'mux%i.%s' % (num, signal.name), pincount=num)
        hal.addf(mux.name, thread)
        for n, section in enumerate(sections):
            mux.pin('in%i' % n).set(c.find(section, item[1]))
        mux.pin('sel').link(selSignal)
        mux.pin('out').link(signal)

    for item in floatSignals:
        signal = hal.Signal('%s-%s' % (sigBase, item[0]), hal.HAL_FLOAT)
        mux = rt.newinst('muxn', 'mux%i.%s' % (num, signal.name), pincount=num)
        hal.addf(mux.name, thread)
        for n, section in enumerate(sections):
            mux.pin('in%i' % n).set(c.find(section, item[1]))
        mux.pin('sel').link(selSignal)
        mux.pin('out').link(signal)


def setup_estop(errorSignals, thread):
    # Create estop signal chain
    estopUser = hal.Signal('estop-user', hal.HAL_BIT)
#     estopReset = hal.Signal('estop-reset', hal.HAL_BIT)
#     estopOut = hal.Signal('estop-out', hal.HAL_BIT)
#     estopIn = hal.Signal('estop-in', hal.HAL_BIT)
#     estopError = hal.Signal('estop-error', hal.HAL_BIT)

#     num = len(errorSignals)
#     orComp = rt.newinst('orn', 'or%i.estop-error' % num, pincount=num)
#     hal.addf(orComp.name, thread)
#     for n, sig in enumerate(errorSignals):
#         orComp.pin('in%i' % n).link(sig)
#     orComp.pin('out').link(estopError)

#     estopLatch = rt.newinst('estop_latch', 'estop-latch')
#     hal.addf(estopLatch.name, thread)
#     estopLatch.pin('ok-in').link(estopUser)
#     estopLatch.pin('fault-in').link(estopError)
#     estopLatch.pin('reset').link(estopReset)
#     estopLatch.pin('ok-out').link(estopOut)

    estopUser.link('iocontrol.0.user-enable-out')
#     estopReset.link('iocontrol.0.user-request-enable')

    # Monitor estop input from hardware
#     estopIn.link('iocontrol.0.emc-enable-in')
    estopUser.link('iocontrol.0.emc-enable-in')


def setup_tool_loopback():
    # create signals for tool loading loopback
    hal.net('iocontrol.0.tool-prepare', 'iocontrol.0.tool-prepared')
    hal.net('iocontrol.0.tool-change', 'iocontrol.0.tool-changed')


def setup_estop_loopback():
    # create signal for estop loopback
    hal.net('iocontrol.0.user-enable-out', 'iocontrol.0.emc-enable-in')

def setup_io():
    ### connect i/o signals to wosi
    for i in range(0,96):
        dout_pin = hal.newsig("dout_%d" % i, hal.HAL_BIT)
        hal.Pin("wosi.gpio.out.%d" % i).link(dout_pin)
        if (i == 0):    # dout_0 is for servo-on
            hal.Pin("axis.0.amp-enable-out").link(dout_pin)
            hal.Pin("servo_tick.amp-enable").link(dout_pin)
            hal.Pin("son_delay.in").link(dout_pin)
        elif (i == 1):  # dout_1 is for machine-on and brake-release
            # it is with 2 sec delay after servo-on,
            # to earn a 2 sec RISC-ON delay after AC-SVO-ON
            hal.Pin("son_delay.out").link(dout_pin)
            # 將延時過的 amp-enable 接給 FPGA/RISC.machine-on
            hal.Pin("wosi.machine-on").link(dout_pin) 
        elif (i == 32):  # dout_32 is for spindle-on
            hal.Pin("motion.spindle-on").link(dout_pin)
        else:            # default to M64Pxx operations
            hal.Pin("motion.digital-out-%02d" % i).link(dout_pin)

    for i in range(0,96):
        if ("din_%d"%i in hal.signals):
            din_pin = hal.Signal("din_%d" % i)
        else:
            din_pin = hal.newsig("din_%d" % i, hal.HAL_BIT)
        hal.Pin("wosi.gpio.in.%d" % i).link(din_pin)
        hal.Pin("motion.digital-in-%02d" % i).link(din_pin)
        if (i == 0):    # connect din_0(ESTOP) to motion.enable
            hal.Pin("motion.enable").link(din_pin)
        if ("din_%d_not"%i in hal.signals):
            din_not_pin = hal.Signal("din_%d_not" % i)
        else:
            din_not_pin = hal.newsig("din_%d_not" % i, hal.HAL_BIT)
        hal.Pin("wosi.gpio.in.%d.not" % i).link(din_not_pin)

def setup_motion():
    ### create motion signals
    spindle_forward_pin = hal.newsig("spindle_forward", hal.HAL_BIT)
    hal.Pin("motion.spindle-forward").link(spindle_forward_pin)
    spindle_reverse_pin = hal.newsig("spindle_reverse", hal.HAL_BIT)
    hal.Pin("motion.spindle-reverse").link(spindle_reverse_pin)
    spindle_at_speed_pin = hal.newsig("spindle_at_speed", hal.HAL_BIT)
    hal.Pin("motion.spindle-at-speed").link(spindle_at_speed_pin)
    spindle_brake_pin= hal.newsig("spindle_brake", hal.HAL_BIT)
    hal.Pin("motion.spindle-brake").link(spindle_brake_pin)
    spindle_speed_in_pin= hal.newsig("spindle_speed_in", hal.HAL_FLOAT)
    hal.Pin("motion.spindle-speed-in").link(spindle_speed_in_pin)
    spindle_speed_out_pin= hal.newsig("spindle_speed_out", hal.HAL_FLOAT)
    hal.Pin("motion.spindle-speed-out").link(spindle_speed_out_pin)
    spindle_revs_pin= hal.newsig("spindle_revs", hal.HAL_FLOAT)
    hal.Pin("motion.spindle-revs").link(spindle_revs_pin)

    # net xuu-per-rev motion.spindle.xuu-per-rev => wosi.stepgen.0.uu-per-rev
    xuu_pin= hal.newsig("xuu-per-rev", hal.HAL_FLOAT)
    hal.Pin("motion.spindle.xuu-per-rev").link(xuu_pin)
    hal.Pin("wosi.stepgen.0.uu-per-rev").link(xuu_pin)

    for i in range(0,6):
        enc_pos_pin = hal.newsig("enc_pos_j%d" % i, hal.HAL_S32)
        hal.Pin("wosi.stepgen.%d.enc_pos" % i).link(enc_pos_pin)
        cmd_pos_pin = hal.newsig("cmd_pos_j%d" % i, hal.HAL_S32)
        hal.Pin("wosi.stepgen.%d.cmd-pos" % i).link(cmd_pos_pin)
        ferror_pin = hal.newsig("ferror_j%d" % i, hal.HAL_FLOAT)     
        hal.Pin("wosi.stepgen.%d.ferror" % i).link(ferror_pin)

def setup_analog():
    ### connect analog signals to wosi
    for i in range(0,16):
        ain_pin = hal.newsig("ain_%d" % i, hal.HAL_FLOAT)
        hal.Pin("wosi.analog.in.%d" % i).link(ain_pin)
    
    for i in range(0,4):
        aout_pin = hal.newsig("aout_%d" % i, hal.HAL_FLOAT)
        hal.Pin("wosi.analog.out.%d" % i).link(aout_pin)
        aoutfb_pin = hal.newsig("aout_%d_fb" % i, hal.HAL_FLOAT)
        hal.Pin("wosi.analog.out.%d.fb" % i).link(aoutfb_pin)

def setup_debug():
    ### connect debug signals to wosi
    for i in range(0,8):
        debug_pin = hal.newsig("debug_%d" % i, hal.HAL_S32)
        hal.Pin("wosi.debug.value-%02d" % i).link(debug_pin)

def setup_signals():
    ### setup signals component as hal-remote-component
    name = 'signals'
    comp = hal.RemoteComponent(name, timer=100)
    
    # create signals.xxx...
    comp.newpin('bptick', hal.HAL_U32, hal.HAL_IN)

    for i in range(0,8):
        comp.newpin('joint.%d.enc-pos' % (i), hal.HAL_S32, hal.HAL_IN)
        comp.newpin('joint.%d.cmd-pos' % (i), hal.HAL_S32, hal.HAL_IN)
        comp.newpin('joint.%d.vel-fb' % (i), hal.HAL_FLOAT, hal.HAL_IN)
        comp.newpin('joint.%d.pos-cmd' % (i), hal.HAL_FLOAT, hal.HAL_IN)
        comp.newpin('joint.%d.pos-fb' % (i), hal.HAL_FLOAT, hal.HAL_IN)
        comp.newpin('joint.%d.ferror' % (i), hal.HAL_FLOAT, hal.HAL_IN)

    for i in range(0,96):
        comp.newpin('gpio.in.%d' % (i), hal.HAL_BIT, hal.HAL_IN)
    
    for i in range(0,96):
        comp.newpin('gpio.out.%d' % (i), hal.HAL_BIT, hal.HAL_IN)
    
    for i in range(0,16):
        comp.newpin('analog.in.%d' % (i), hal.HAL_FLOAT, hal.HAL_IN)
    
    for i in range(0,4):
        comp.newpin('analog.out.%d' % (i), hal.HAL_FLOAT, hal.HAL_IN)
        comp.newpin('analog.out.%d.fb' % (i), hal.HAL_FLOAT, hal.HAL_IN)
    
    for i in range(0,8):
        comp.newpin('debug.value.%d' % (i), hal.HAL_S32, hal.HAL_IN)
    
    comp.ready()

    comp.pin('bptick').link('bp-tick')

    for i in range(0,8):
        comp.pin('joint.%d.enc-pos' % (i)).link("enc_pos_j%d" % (i))
        comp.pin('joint.%d.cmd-pos' % (i)).link('cmd_pos_j%d' % (i))
        comp.pin('joint.%d.vel-fb' % (i)).link('J%dvel-fb' % i)
        comp.pin('joint.%d.pos-cmd' % (i)).link('j%d-pos-cmd' % i)
        comp.pin('joint.%d.pos-fb' % (i)).link('j%d-pos-fb' % i)
        comp.pin('joint.%d.ferror' % (i)).link('ferror_j%d' % i)

    for i in range(0,96):
        comp.pin('gpio.in.%d' % (i)).link('din_%d' % i)
    for i in range(0,96):
        comp.pin('gpio.out.%d' % (i)).link('dout_%d' % i)
    for i in range(0,16):
        comp.pin('analog.in.%d' % (i)).link('ain_%d' % i)
    for i in range(0,4):
        comp.pin('analog.out.%d' % (i)).link('aout_%d' % i)
        comp.pin('analog.out.%d.fb' % (i)).link('aout_%d_fb' % i)
    for i in range(0,8):
        comp.pin('debug.value.%d' % (i)).link('debug_%d' % i)

def init_gantry(axisIndex, joints=2, latching=True):
    if latching:
        comp = 'lgantry'
    else:
        comp = 'gantry'
    rt.newinst(comp, 'gantry.%i' % axisIndex, pincount=joints)
    rcomps.create_gantry_rcomp(axisIndex=axisIndex)


def gantry_read(gantryAxis, thread):
    hal.addf('gantry.%i.read' % gantryAxis, thread)


def gantry_write(gantryAxis, thread):
    hal.addf('gantry.%i.write' % gantryAxis, thread)
