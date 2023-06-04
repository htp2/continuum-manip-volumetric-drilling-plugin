# python script that reads a yaml file and replaces certain values
# BODY seg{i} where i is a number between 1 and len(mass)
# going to be replacing value for "mass"
# JOINT joint{i} where i is a number between 1 and len(stiffness)
# going to be replacing value for "stiffness"

import yaml
import sys
import argparse

# command line arguments: first argument is the yaml file to read, second argument is optional and is the yaml file to write
parser = argparse.ArgumentParser(description='Reads a yaml file and replaces certain values')
parser.add_argument('input', help='input yaml file')
# make output name optional (default is input name)
parser.add_argument('-o', '--output', help='output yaml file (default is input name)')
# parse unknown arguments
args, unknown = parser.parse_known_args()

# get input and output file names
filename = args.input
if args.output:
    output = args.output
else:
    output = filename

# check file exists
try:
    with open(filename, 'r') as f:
        data = yaml.load(f, Loader=yaml.SafeLoader)
except FileNotFoundError:
    print("File {} not found".format(filename))
    sys.exit()

# current values were approximated based on FEA calculations, with adjustments made to lead to a reasonable simulation behavior
stiffness = [0.013685109, 0.013281173, 0.012877237, 0.0124733, 0.012069364, 0.011665428, 0.011261491, 0.010857555, 0.010453619, 0.010049683, 0.009645746, 0.00924181, 0.008837874, 0.008433938, 0.008030001, 0.007626065, 0.007222129, 0.006818192, 0.006414256, 0.00601032, 0.005606384, 0.005336815, 0.005336815, 0.005336815, 0.005336815, 0.005336815, 0.005336815]
# make array called mass with 27 elements, each element is 0.01
mass = [0.001 for _ in range(27)]

# replace the values in the json file
for i in range(len(mass)):
    data["BODY seg{}".format(i+1)]["mass"] = mass[i]

for i in range(len(stiffness)):
    data["JOINT joint{}".format(i+1)]["stiffness"] = stiffness[i]

# write the new yaml
with open(output, 'w') as f:
    yaml.dump(data, f)
