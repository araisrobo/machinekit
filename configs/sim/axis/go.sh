linuxcnc_stop
sudo cp /dev/null /var/log/linuxcnc.log

DEBUG=5 linuxcnc axis_jerk_mm.ini | tee run.log

# for remote-ui
# mklauncher . &
# python mkwrapper_mm.py -c -i mkwrapper_mm.ini
