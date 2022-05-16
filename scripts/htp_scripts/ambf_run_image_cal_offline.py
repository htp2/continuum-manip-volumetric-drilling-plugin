import argparse
import rosbag
import rospy
from std_msgs.msg import Float32
from vdrilling_msgs.msg import points

from ambf_msgs.msg import RigidBodyState
import numpy as np
from std_srvs.srv import Empty

class Topic:
    def __init__(self, topic_name, topic_type):
        self.topic_name = topic_name
        self.publisher = rospy.Publisher(topic_name, topic_type, queue_size=100)
        self.topic_seen = False

def take_image_client():
    service_name = "/take_xray"
    print(f"Waiting for service: {service_name}")
    rospy.wait_for_service(service_name)
    try:
        take_image = rospy.ServiceProxy(service_name, Empty)
        resp1 = take_image()
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

        # topic_dict["/ambf/env/snake_stick/State"] = Topic("/ambf/env/snake_stick/State",Float32)
        # topic_dict["/ambf/env/carm/State"] = Topic("/ambf/env/carm/State",Float32)
        # topic_dict["/ambf/env/Seg27/State"] = Topic("/ambf/env/snake_stick/Seg27",Float32)
        # topic_dict["/ambf/volumetric_drilling/bend_motor/measured_js"] = Topic("/ambf/volumetric_drilling/bend_motor/measured_js",Float32)
    
    rospy.init_node('bag_republisher_async', anonymous=True)

    parser = argparse.ArgumentParser(description='Reorder a bagfile based on header timestamps.')
    parser.add_argument('bagfile', nargs=1, help='input bag file')
    args = parser.parse_args()
    bagfile = args.bagfile[0]
    print(bagfile)
    bag = rosbag.Bag(bagfile)

    stop_freq_hz = 1 # sec
    run_duration = rospy.Duration(secs=1.0/stop_freq_hz)
    topic_seen = np.array(len(topics),dtype=bool)
    all_topics_seen = False
    for topic, msg, t in bag.read_messages(topics=topics):
        if not all_topics_seen:
            topic_dict[topic].topic_seen=True
            print([x.topic_seen for x in topic_dict.values()])
            all_topics_seen = np.all([x.topic_seen for x in topic_dict.values()])
            if all_topics_seen:
                start_time = t
            continue
        elapsed = t-start_time
        print(f"elapsed: {elapsed}")
        print(f"run_duration: {run_duration}")
        if elapsed < run_duration:
            topic_dict[topic].publisher.publish(msg)
            
        else:
            start_time = t
            take_image_client()
            user_in = input('paused')
        

        # # print(msg)
        # if(i>0):
        #     print(t-t_old)
        # t_old = t
        # # dict[msg.wall_time] = (topic,msg,t )
        # user_in = input()
        # i+=1






if (__name__ == "__main__"):
    main()
