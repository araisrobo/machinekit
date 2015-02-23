sudo cp /dev/null /var/log/linuxcnc.log
cp position.blank position.txt
mv constraints.log constraints_old.log
linuxcnc configs/axis_jerk_mm.ini | tee test.log &
# python machine_setup.py ../nc_files/quick-tests/square.ngc
python machine_setup.py ../nc_files/quick-tests/square_g64_tol.ngc
fg
./process_runlog.sh test.log movement.txt
if [ -a movement.txt ] 
then
    octave --persist ./octave/plot_movement.m
fi
