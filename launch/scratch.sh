#!/bin/bash


for i in {1..10}
do
    tdstamp=$(date "+%Y%m%d%H%M%S")
    outdir=/home/henry/exps/20221102_ipcai22_firstpass/$tdstamp/ 
    mkdir $outdir
    for cam in $(seq 0 5 20 | shuf)
    do
        for slice in $(seq 0 1 15 | shuf)
        do
            echo "c: $cam, s: $slice" 
            roslaunch continuum_manip_volumetric_drilling_plugin ambf_curved_drilling_ipcai2022TEST.launch &
            pid=$!            
            sleep 5
            timeout -k 10 10m python3 /home/henry/bigss/catkin_ws/src/bigss_spine/spine_python/ambf_highlevel_ipcai2022_script.py -t /home/henry/bigss/catkin_ws/src/bigss_spine/spine_robot_control/config/sim_ur5/points.txt -o $outdir -e $cam -s $slice --no-stamp 
            kill $pid
            pkill -f "AMBF" -9
            sleep 5
        done
    done

    python3 /home/henry/bigss/catkin_ws/src/continuum-manip-volumetric-drilling-plugin/scripts/htp_scripts/evaluate_simcurvedrill_ipcai22.py -d $outdir

done