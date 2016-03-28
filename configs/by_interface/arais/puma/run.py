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
configs_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(configs_dir)

parser = argparse.ArgumentParser(description='This is the motorctrl demo run script '
                                 'it demonstrates how a run script could look like '
                                 'and of course starts the motorctrl demo')
parser.add_argument('-c', '--config', help='Starts the config server', action='store_true')
parser.add_argument('-f', '--fpga', help='Starts the FPGA bitstream loader', action='store_true')
parser.add_argument('-l', '--local', help='Enable local mode only', action='store_true')
parser.add_argument('-g', '--gladevcp', help='Starts the GladeVCP user interface', action='store_true')
parser.add_argument('-s', '--startpy', help='Starts python scripts after launching linuxcnc server', action='store_true')
parser.add_argument('-m', '--halmeter', help='Starts the halmeter', action='store_true')
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

    if args.fpga:
        fpga_dir = os.path.join(configs_dir, "../../../../../ar-tools/rpi2_fpga")
        os.chdir(fpga_dir)
        subprocess.check_call(['./ar_loader', 'ar11_top.bit'])
        os.chdir(configs_dir)

    if args.config:
        # the point-of-contact for QtQUickVCP
        machineface_dir = os.path.join(configs_dir, "../../../../../remote-ui/RA605face")
        launcher.start_process("configserver -d -n RA605 %s" % machineface_dir)

        os.chdir(configs_dir)
        launcher.start_process('linuxcnc %s' % (args.ini))


    if args.startpy:
        os.chdir(configs_dir)
        subprocess.check_call(['sleep', '5']) # important do not delete
        os.system("python ./scripts/update_abs_enc.py &")
#         launcher.start_process('python scripts/meinan.py')
#         launcher.start_process('python scripts/css.py')

    if args.halmeter:
        # launcher.start_process('halmeter')
        launcher.start_process('halshow')

    if args.gladevcp:
        # start the gladevcp version
        if args.local:
            # no -N flag - local case, use IPC sockets, no zeroconf resolution
            # launcher.start_process('gladevcp -E -u motorctrl.py motorctrl.ui')
            launcher.start_process('python utility.py -x {XID}')
        else:
            # -N - remote case, use zeroconf resolution
            launcher.start_process('gladevcp -N -E -u motorctrl.py motorctrl.ui')
            launcher.start_process('python utility.py')

    while True:
        launcher.check_processes()
        time.sleep(1)

except subprocess.CalledProcessError:
    launcher.end_session()
    sys.exit(1)

sys.exit(0)
