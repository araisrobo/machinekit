
For checking , execute run.sh:
    export FLAVOR=rt-preempt
    realtime start
    halcmd -f stream_write.hal
    ULAPI_DEBUG=5 ./check_wosi_trans

To inspecting hal components:
    halcmd show # show all
    halcmd show comp # show status of components

To exit:
    realtime stop
