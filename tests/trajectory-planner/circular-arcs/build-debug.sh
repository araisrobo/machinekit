#/bin/bash
cd ../../../src
#Ugly way to force rebuild of kinematics, which assumes that tp_debug isn't
#used anywhere else...
touch emc/tp/t[cp]*.[ch]
CONCURRENCY_LEVEL=`getconf _NPROCESSORS_ONLN`
#make EXTRA_DEBUG='-DTP_DEBUG'
#make EXTRA_DEBUG='-DTC_DEBUG -DTP_DEBUG'
#make EXTRA_DEBUG='-DTP_DEBUG -DTP_INFO_LOGGING'
#make EXTRA_DEBUG='-DTC_DEBUG -DTP_DEBUG -DTP_INFO_LOGGING'
make -j${CONCURRENCY_LEVEL} OPT='-O0' EXTRA_DEBUG='-DTC_DEBUG -DTP_DEBUG -DTP_INFO_LOGGING' V=1 && sudo make setuid
cd -
