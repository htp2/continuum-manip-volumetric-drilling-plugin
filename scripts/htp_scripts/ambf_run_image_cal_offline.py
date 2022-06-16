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
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image

from ambf_msgs.msg import RigidBodyState
from vdrilling_msgs.msg import points
from vdrilling_msgs.srv import SendString
class TopicWithPub:
    def __init__(self, topic_name, topic_type):
        self.topic_name = topic_name
        self.publisher = rospy.Publisher(topic_name, topic_type, queue_size=100)
        self.topic_seen = False

class PeriodicActionAsync:
    def __init__(self, rate_hz, start_time):
        self.wait_duration = rospy.Duration(1.0/rate_hz)
        self.last_trigger_time = start_time
    
    def is_ready_to_do_action(self, current_time):
        elapsed = current_time - self.last_trigger_time
        return elapsed > self.wait_duration

    def reset_last_triggered_time(self, current_time):
        self.last_trigger_time = current_time

    def do_action_if_time(self, current_time):
        pass

class PeriodicServiceCallAsync(PeriodicActionAsync):
    def __init__(self, rate_hz, start_time, service_name):
        PeriodicActionAsync.__init__(self, rate_hz, start_time)
        self.service_name = service_name
    
    def do_action_if_time(self, current_time):
        if self.is_ready_to_do_action(current_time):
            call_ros_service(self.service_name)
            self.reset_last_triggered_time(current_time)

class WriterFromTopicToBag(PeriodicActionAsync):
    # time_offset to allow simulation of certiain data taking some amount of time to be generated and published
    def __init__(self, rate_hz, start_time, bag_write, topic, msg_type, time_offset=0):
        PeriodicActionAsync.__init__(self, rate_hz, start_time)
        self.bag_write = bag_write
        self.last_msg = msg_type()
        self.topic = topic
        self.time_offset = rospy.Duration(secs=time_offset)
        self.subscriber = rospy.Subscriber(topic, msg_type, self.sub_callback, queue_size=1)
    
    def sub_callback(self, msg):
        self.last_msg = msg
    
    def do_action_if_time(self, current_time):
        if self.is_ready_to_do_action(current_time):
            self.bag_write.write(self.topic, self.last_msg, current_time + self.time_offset)
            self.reset_last_triggered_time(current_time)

# class PeriodicPublisherAsync(PeriodicActionAsync):
#     def __init__(self, rate_hz, start_time):
#         PeriodicActionAsync.__init__(rate_hz, start_time)
    
#     def publish_msg_if_time(self, current_time, msg, publisher):
#         if self.is_ready_to_do_action():
#             publisher.publish(msg)
#             self.reset_last_triggered_time(current_time)



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
        topic_dict[name] = TopicWithPub(name, type)
    
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

    

    parser = argparse.ArgumentParser(description='Replay a bag file asyncronously to generate images and get registration')
    parser.add_argument('bagfile', nargs=1, help='input bag file')
    args = parser.parse_args(args_stripped_ros)
    bagfile = args.bagfile[0]
    bag = rosbag.Bag(bagfile)

    new_bag = rosbag.Bag(bagfile+"_a"+timestamp+".bag", 'w')
    imaging_rate_hz = 1/10
    est_rate_hz = 20
    epoch_time = rospy.Time()

    psca_take_image = PeriodicServiceCallAsync(imaging_rate_hz, epoch_time, "/take_xray")
    psca_register_image = PeriodicServiceCallAsync(imaging_rate_hz, epoch_time, "/register_image")
    pcsa_publish_estimate = PeriodicServiceCallAsync(est_rate_hz, epoch_time, "/force_publish_estimate")
    psca_list = [psca_take_image, psca_register_image, pcsa_publish_estimate]

    new_write_bag_topic_type_rate = [
        ("/ambf/volumetric_drilling/base_to_tip", TransformStamped, est_rate_hz),
        ("/ambf/volumetric_drilling/tip_pos_estimate_calmodelonly", Pose, est_rate_hz),
        ("/ambf/volumetric_drilling/tip_pos_estimate_calmodelonly_relative", Pose, est_rate_hz),
        ("/snake_imager/image", Image, imaging_rate_hz),
        ("/snake_imager/tip_pos_estimate_imgonly_relative", Pose, imaging_rate_hz),
        ("/ambf/volumetric_drilling/tip_pos_estimate_imgonly", Pose, imaging_rate_hz)
    ]
    psca_writebag_list = [WriterFromTopicToBag(rate, epoch_time, new_bag, topic, type) for topic, type, rate in new_write_bag_topic_type_rate]
    for psca in psca_writebag_list: psca_list.append(psca)

    stop_freq_hz = 1.0/10.0; 
    run_duration = rospy.Duration(secs=1.0/stop_freq_hz)
    topic_seen = np.array(len(topics),dtype=bool)
    all_topics_seen = False

    for topic, msg, t in bag.read_messages(topics=topics):
        """ Catches ctrl-c"""
        if rospy.is_shutdown():
            break

        """Direct copy all items to new bag file"""
        new_bag.write(topic, msg, t) # keep everything old in this new bagfile
        
        """Loops until all topics seen before continuing to code below"""
        if not all_topics_seen:
            topic_dict[topic].topic_seen=True
            # print([x.topic_seen for x in topic_dict.values()])
            all_topics_seen = np.all([x.topic_seen for x in topic_dict.values()])
            if all_topics_seen:
                psca_take_image.last_trigger_time = t
                psca_register_image.last_trigger_time = t
            continue

        """ Update topic to ROS """
        topic_dict[topic].publisher.publish(msg)
        
        """ Give a short wait if any of our actions need to be called, to allow estimator to run """
        # TODO: Switch estimator over to also take service call?
        # for psca in psca_list:
        #     if psca.is_ready_to_do_action(t):
        #         time.sleep(0.001*50)
        #         break
        
        """Do action if triggered"""
        for psca in psca_list: psca.do_action_if_time(t)

        # elapsed = t-start_time
        # if elapsed < run_duration:
            
        # else:
        #     start_time = t
        #     call_ros_service("/take_xray")
        #     call_ros_service("/register_image")
        #     # user_in = input('paused')


    call_ros_service("/end_xray")
    bag.close()
    new_bag.close()
if (__name__ == "__main__"):
    main()
