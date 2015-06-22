wosi.reload-params
    hal bit set by external component, and reset after reloading parameters
    When set, the WOSI will reload parameters from INIFILE
    Only valid for MACHINE-OFF state
    usage:
        AXIS GUI: MACHINE-OFF
        halcmd setp wosi.reload-params TRUE
        halcmd getp wosi.reload-params  # confirm it has been reset to FALSE

For checking , execute run.sh:
    export FLAVOR=rt-preempt
    realtime start
    halcmd -f stream_write.hal
    ULAPI_DEBUG=5 ./check_wosi_trans -r ring_jcmd -I meinan.ini

To inspecting hal components:
    halcmd show # show all
    halcmd show comp # show status of components

To exit:
    realtime stop
