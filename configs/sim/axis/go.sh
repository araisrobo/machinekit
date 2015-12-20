linuxcnc_stop.sh
sudo cp /dev/null /var/log/linuxcnc.log
DEBUG=5 linuxcnc axis_mm.ini | tee run.log
