#!/bin/bash

snake_model_path="/home/bigss/henry/snake_registration/simulation/JustinSnakeModel_huatt"
output_path="/home/bigss/henry/snake_registration/simulation/output/"
anatomy_path="/home/bigss/henry/snake_registration/simulation/anatomy/" 

xreg-snake-htp-imager-ros-main $snake_model_path $output_path $anatomy_path