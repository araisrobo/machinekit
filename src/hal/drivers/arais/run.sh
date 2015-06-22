export FLAVOR=rt-preempt
export DEBUG=5    # INFO,  for realtime to set RTAPI_MSG_LEVEL
sudo cp /dev/null /var/log/linuxcnc.log # empty linuxcnc.log
# export DEBUG=1    # ERROR, for realtime to set RTAPI_MSG_LEVEL
realtime status
realtime stop
realtime start
halcmd -f servo_tick.hal 
realtime status
halcmd show comp

# run check program
sudo $EMC2_HOME/bin/wosi_trans -r ring_jcmd -I meinan.ini
# or execute wosi_trans after "make setuid"
