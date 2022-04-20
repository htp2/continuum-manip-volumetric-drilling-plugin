NRRD_FILE="/home/henry/snake_registration/simulation/anatomy/built_model.nrrd"
VOLUME_NAME="built_model"
YAML_SAVE_LOCATION="/home/henry/continuum-manip-volumetric-drilling-plugin/ADF/"
PNG_IMG_SAVE_LOCATION="/home/henry/continuum-manip-volumetric-drilling-plugin/resources/volumes"
echo "setup_files_for_nrrd_volume.py -n $NRRD_FILE -v $VOLUME_NAME -y $YAML_SAVE_LOCATION -i $PNG_IMG_SAVE_LOCATION"
python3 setup_files_for_nrrd_volume.py -n $NRRD_FILE -v $VOLUME_NAME -y $YAML_SAVE_LOCATION -i $PNG_IMG_SAVE_LOCATION
