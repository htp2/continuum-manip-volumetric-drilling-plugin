#!/usr/bin/env python3
import time
import os
import argparse
import sys
import numpy as np

import rosbag
import rospy
from std_msgs.msg import Float32
from std_srvs.srv import Empty

from ambf_msgs.msg import RigidBodyState
from vdrilling_msgs.msg import points
from vdrilling_msgs.srv import SendString
class Topic:
    def __init__(self, topic_name, topic_type):
        self.topic_name = topic_name
        self.publisher = rospy.Publisher(topic_name, topic_type, queue_size=100)
        self.topic_seen = False

# def take_image_client():
#     service_name = "/take_xray"
#     print(f"Waiting for service: {service_name}")
#     rospy.wait_for_service(service_name)
#     try:
#         take_image = rospy.ServiceProxy(service_name, Empty)
#         resp1 = take_image()
#         return resp1
#     except rospy.ServiceException as e:
#         print("Service call failed: %s"%e)

# def register_image_client():
#     service_name = "/register_image"
#     print(f"Waiting for service: {service_name}")
#     rospy.wait_for_service(service_name)
#     try:
#         register_image = rospy.ServiceProxy(service_name, Empty)
#         resp1 = register_image()
#         return resp1
#     except rospy.ServiceException as e:
#         print("Service call failed: %s"%e)

def call_ros_service(service_name, request=Empty):
    # Defaults to std_srv::Empty
    print(f"Waiting for service: {service_name}")
    rospy.wait_for_service(service_name)
    try:
        service_client = rospy.ServiceProxy(service_name, request)
        resp1 = service_client()
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def main():
    

    topics = [
        "/ambf/env/snake_stick/State",
        "/ambf/env/seg27/State",
        "/ambf/env/carm/State",
        "/ambf/volumetric_drilling/bend_motor/measured_js",
        "/ambf/volumetric_drilling/voxels_removed"
        ]

    types = [
        RigidBodyState,
        RigidBodyState,
        RigidBodyState,
        Float32,
        points
        ]
    
    topic_dict = {}
    for name,type in zip(topics,types):
        topic_dict[name] = Topic(name, type)
    
    rospy.init_node('bag_republisher_async', anonymous=True)
    args_stripped_ros = rospy.myargv(argv=sys.argv)[1:] # this will strip off all of the extra args added when ros calls this python script (e.g. with launch file)
    timestamp = time.strftime("%Y%m%d%H%M%S")
    output_directory = "/home/henry/snake_registration/simulation/output/" + timestamp + "/"
    req = SendString
    req.data = output_directory
    print(f"req: {req.data}" )
    if not os.path.isdir(output_directory):
        os.mkdir(output_directory)
    call_ros_service("/change_output_dir", request=req)
    # print(f"args_stripped_ros: {args_stripped_ros}")

    parser = argparse.ArgumentParser(description='Reorder a bagfile based on header timestamps.')
    parser.add_argument('bagfile', nargs=1, help='input bag file')
    args = parser.parse_args(args_stripped_ros)
    bagfile = args.bagfile[0]
    bag = rosbag.Bag(bagfile)

    stop_freq_hz = 0.1 
    run_duration = rospy.Duration(secs=1.0/stop_freq_hz)
    topic_seen = np.array(len(topics),dtype=bool)
    all_topics_seen = False
    for topic, msg, t in bag.read_messages(topics=topics):
        if rospy.is_shutdown():
            break
        if not all_topics_seen:
            topic_dict[topic].topic_seen=True
            # print([x.topic_seen for x in topic_dict.values()])
            all_topics_seen = np.all([x.topic_seen for x in topic_dict.values()])
            if all_topics_seen:
                start_time = t
            continue
        elapsed = t-start_time
        # print(f"elapsed: {elapsed}")
        # print(f"run_duration: {run_duration}")
        if elapsed < run_duration:
            topic_dict[topic].publisher.publish(msg)
            
        else:
            start_time = t
            call_ros_service("/take_xray")
            call_ros_service("/register_image")
            # user_in = input('paused')


if (__name__ == "__main__"):
    main()
