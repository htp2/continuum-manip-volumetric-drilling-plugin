#!/bin/bash
ids=()
list=$(ls ~/exps/20221113_ipcai_planned_trajs/trajectories/robot/*csv); for x in $list; do v=${x##*/}; ids+=(${v%%_traj*}); done
rob_path=/home/henry/exps/20221113_ipcai_planned_trajs/trajectories/robot/
ambf_path=/home/henry/exps/20221113_ipcai_planned_trajs/trajectories/ambf/

for id in ${ids[@]}
do 
    tdstamp=$(date "+%Y%m%d%H%M%S")
    outdir=/home/henry/exps/20221113_ipcai_planned_trajs/trajectories/simul/$tdstamp/$id/ 
    mkdir -p $outdir
    for cam in $(seq 0 5 10 | shuf)
    do
        for slice in $(seq 0 2 10 | shuf)
        do
            echo "c: $cam, s: $slice" 
            roslaunch continuum_manip_volumetric_drilling_plugin ambf_curved_drilling_ipcai2022TEST2.launch constraint_config:=${rob_path}${id}_constraints_config.json trace_path:=${ambf_path}${id}.csv &
            pid=$!            
            sleep 5
            timeout -k 10 10m python3 /home/henry/bigss/catkin_ws/src/bigss_spine/spine_python/ambf_highlevel_ipcai2022_script_traj_select.py -t ${rob_path}${id}_traj.csv -o $outdir -e $cam -s $slice --no-stamp 
            kill $pid
            pkill -f "AMBF" -9
            sleep 5
        done
    done

    python3 /home/henry/bigss/catkin_ws/src/continuum-manip-volumetric-drilling-plugin/scripts/htp_scripts/evaluate_simcurvedrill_ipcai22.py -d $outdir

done


