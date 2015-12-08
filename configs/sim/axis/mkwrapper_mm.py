#!/usr/bin/python

import sys
import os
import subprocess
import argparse
import time
from machinekit import launcher
from machinekit import rtapi

launcher.register_exit_handler()
# launcher.set_debug_level(5)
home_dir = os.path.expanduser('~')
configs_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(configs_dir)

parser = argparse.ArgumentParser(description='This is the motorctrl demo run script '
                                 'it demonstrates how a run script could look like '
                                 'and of course starts the motorctrl demo')
parser.add_argument('-c', '--config', help='Starts the config server', action='store_true')
parser.add_argument('-d', '--debug', help='Enable debug mode', action='store_true')
parser.add_argument('-i', '--ini', type=str, help='Specify .ini file', default='MEINAN.ini', action='store')

args = parser.parse_args()
    
# set default debug level as 3 (INFO)
# launcher.set_debug_level(3)
# set default debug level as 1(ERR), log error message to /var/log/linuxcnc.log
launcher.set_debug_level(1)
if args.debug:
    launcher.set_debug_level(5)

try:
    launcher.check_installation()
    launcher.cleanup_session()

    # the point-of-contact for QtQUickVCP
    machineface_dir = os.path.join(home_dir, "proj/remote-ui/Machineface")
    launcher.start_process("configserver -d -n MKSim %s" % machineface_dir)
    # launcher.start_process('linuxcnc %s' % (args.ini))
    launcher.start_process('linuxcnc %s' % ("mkwrapper_mm.ini"))

    while True:
        launcher.check_processes()
        time.sleep(1)

except subprocess.CalledProcessError:
    launcher.end_session()
    sys.exit(1)

sys.exit(0)
