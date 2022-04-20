import rospy

from vdrilling_msgs.msg import points
n = rospy.init_node("vox_rem_pub")
pub = rospy.Publisher("/ambf/volumetric_drilling/voxels_removed", points)

r = rospy.Rate(10000)
msg = points()
for i in range(30,100):
    for j in range(30,100):
        for k in range(250,300):
            msg.voxel_removed.x = i
            msg.voxel_removed.y = j
            msg.voxel_removed.z = k
            pub.publish(msg)
            r.sleep()