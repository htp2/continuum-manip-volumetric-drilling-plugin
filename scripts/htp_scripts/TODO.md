Write paper
-figures

Debugging
Change timeout to much larger (10 mins? at least)

Multiple targets:
Design new trajectories
- should probably all have same length? Or change to takin images every x mm of traj
- can it handle curving the other way? Not sure it's worth the risk

- Calculate new traj
- calculate what that would be in robot coords
- make any changes to constraints as needed

path constraints - less than total length of the CM

another metric
-total volume removed

another variable
- total curvature




curve starts in certain region
reaches target in femoral head
has length of Xmm
curves in only one direction



Walkthrough

In 3D slicer make markups of the following and save as .mrk.json type

'EntryVecs': pairs of points (must be one after the other) defining the approx init traj you want to take.
'Targets': target points where you want to end
'plane_top': points that lie on the 'top' of the femoral neck 
'plane_bottom': points that lie on the 'bottom' of the femoral neck
The mid point of each plane_top, plane_bottom pair will be used to try to keep the trajectory towards the center of the femoral neck

Now make sure directories match and then run:
simcurvedrill_trajgen.py
Bit slow for now. Worked well on 'wrong' leg. Not sure why, but having some issues at the moment.


[X.mrk.json] = Define points in slicer(manual)
[traj_entryi_targetj.fcsv, traj_entryi_targetj.csv] = simcurvedrill_trajgen.py(X.mrk.json)
[ambf_traj_file.csv] = xreg-setup_traj_from_3dslicer_traj(< volume path > < 3dslicer traj path > < output path >)
[ambf_constraints_config.json, points.txt] = use_trajgen_to_get_constraints(ambf_traj_file.csv)

NOTE: Still need to add that initial point in the python high level code to get everything lined up.

[Results.csv] = simulation(ambf_constraints_config.json, points.txt)






source ~/bigss/catkin_ws/devel/setup.bash && python3 ~/bigss/catkin_ws/src/continuum-manip-volumetric-drilling-plugin/launch/main_ipcai.py 
