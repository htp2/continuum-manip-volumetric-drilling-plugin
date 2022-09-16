#!/bin/bash

/home/$USER/ambf/bin/lin-x86_64/ambf_simulator \
    --launch_file $CATKIN_WS/src/continuum-manip-volumetric-drilling-plugin/launch.yaml \
    -l 24,31 \
    --name_body_to_trace Burr \
    --anatomy_volume_name cube \
    --csv_filename_static_traces $CATKIN_WS/src/continuum-manip-volumetric-drilling-plugin/resources/axis.csv
