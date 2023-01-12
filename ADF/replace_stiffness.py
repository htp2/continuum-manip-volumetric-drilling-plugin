import yaml

# Load the yaml file
filename = "/home/henry/bigss/catkin_ws/src/continuum-manip-volumetric-drilling-plugin/ADF/bigss_snake_centered_collision_meshes_burr.yaml"
with open(filename, "r") as file:
    data = yaml.load(file, Loader=yaml.FullLoader)

# Define the stiffness values for each joint
stiffness_values = [0.784098994, 0.76095515, 0.737811307, 0.714667463, 0.691523619, 0.668379776, 0.645235932, 0.622092089, 0.598948245, 0.575804401, 0.552660558, 0.529516714, 0.50637287, 0.483229027, 0.460085183, 0.436941339, 0.413797496, 0.390653652, 0.367509808, 0.344365965, 0.321222121, 0.305776968, 0.305776968, 0.305776968, 0.305776968, 0.305776968, 0.305776968]

# Iterate through the data and find occurrences of 'joint1' to 'joint26'
for key, value in data.items():
    if key.startswith("JOINT joint") and key[11:].isdigit():        
        joint_num = int(key[11:]) # Extract the joint number from the key
        if joint_num in range(1, 27):
            # Find the 'stiffness' key and update the value
            for subkey, subvalue in value.items():
                if subkey == "stiffness":
                    value[subkey] = stiffness_values[joint_num-1]

# Save the updated data to the yaml file
# put "_changed" before yaml
new_filename = filename.replace(".yaml", "_changed.yaml")

with open(new_filename, "w") as file:
    yaml.dump(data, file)



