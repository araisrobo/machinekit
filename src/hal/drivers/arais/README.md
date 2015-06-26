
## For testing, load FPGA bitfile with ar11_fpga.sh and execute run.sh

## Reload system parameters from INI file
    wosi.reload-params -- hal bit set by external component, 
                          and reset after reloading parameters
    When set, the WOSI will reload parameters from INIFILE
    Only valid at MACHINE-OFF state
    usage:
        AXIS GUI: MACHINE-OFF
        halcmd setp wosi.reload-params TRUE
        halcmd getp wosi.reload-params  # confirm it has been reset to FALSE

## To inspecting hal components:
    halcmd show # show all
    halcmd show comp # show status of components

## To exit:
    realtime stop
