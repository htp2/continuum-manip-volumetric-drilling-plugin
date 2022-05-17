# START_IMAGER=""
source ~/continuum-manip-volumetric-drilling-plugin/build/devel/setup.bash

PLAY_OLD="python3 /home/henry/continuum-manip-volumetric-drilling-plugin/scripts/htp_scripts/free_space_calibration.py ~/snake_registration/simulation/rosbags/testing1_2022-05-13-16-49-32.bag"
RECORD_NEW="rosbag record -a -o ~/snake_registration/simulation/rosbags/test_full2.bag"
$RECORD_NEW & $PLAY_OLD && fg