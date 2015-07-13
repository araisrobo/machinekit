#!/usr/bin/python

import sys
import os
import subprocess
import importlib
from machinekit import launcher
from time import *

launcher.register_exit_handler()
launcher.set_debug_level(5)
os.chdir(os.path.dirname(os.path.realpath(__file__)))

try:
    launcher.check_installation()                                     # make sure the Machinekit installation is sane
    launcher.cleanup_session()                                        # cleanup a previous session
    # launcher.load_bbio_file('myoverlay.bbio')                       # load a BBB universal overlay
    launcher.start_process("configserver ~/proj/remote-ui/Machineface")   # start the configserver
    launcher.start_process('linuxcnc mkwrapper_mm.ini')               # start linuxcnc
except subprocess.CalledProcessError:
    launcher.end_session()
    sys.exit(1)

while True:
    sleep(1)
    launcher.check_processes()
