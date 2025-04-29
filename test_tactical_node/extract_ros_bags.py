#!/usr/bin/env python3

import os
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


def get_topic_types(bag_path: str, storage_id: str):
    """
    Retrieve a mapping from topic name to ROS message type string in the bag.
    """
    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    topics_and_types = reader.get_all_topics_and_types()
    return {t.name: t.type for t in topics_and_types}


def extract_messages(bag_path: str, storage_id: str, target_topics: list):
    """
    Iterate through the bag and print timestamp, topic name, and selected field value
    for the specified target topics.
    """
    # Prepare storage and reader
    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Build a map from topic to message class for deserialization
    topic_types = get_topic_types(bag_path, storage_id)
    msg_type_map = {}
    for topic in target_topics:
        type_str = topic_types.get(topic)
        if type_str:
            msg_type_map[topic] = get_message(type_str)

    out_values = {"ref_spd":{"t":list(), "val":list()},
                  "aeb_trg":{"t":list(), "val":list()},
                  "odom_filt":{"t":list(), "vel_fw":list(), "pos_x":list(), "pos_y":list(), "orient_q":list()}}

    # Read and print messages
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic not in msg_type_map:
            continue
        msg_cls = msg_type_map[topic]
        msg = deserialize_message(data, msg_cls)
        # Timestamp is in nanoseconds
        milli = timestamp // (1000*1000)

        # Selective field extraction
        if topic == '/ref_spd':
            # Assuming std_msgs/Float32
            value = msg.data
            out_values["ref_spd"]["t"].append(milli)
            out_values["ref_spd"]["val"].append(value)

        elif topic == '/odometry/filtered':
            # nav_msgs/Odometry: extract pose x and y
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            orientation_q = msg.pose.pose.orientation
            vel_fw = msg.twist.twist.linear.x
            value = [(x, y), vel_fw, [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]]
            out_values["odom_filt"]["t"].append(milli)
            out_values["odom_filt"]["pos_x"].append(x)
            out_values["odom_filt"]["pos_y"].append(y)
            out_values["odom_filt"]["orient_q"].append([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            out_values["odom_filt"]["vel_fw"].append(vel_fw)
        elif topic == '/aeb_triggered':
            # Assuming std_msgs/Bool
            value = msg.data
            out_values["aeb_trg"]["t"].append(milli)
            out_values["aeb_trg"]["val"].append(value)
        else:
            # Fallback to string representation
            value = msg
            print("filetered topic error msg type")

        #print(f"{milli}, {topic}, {value}")
    return out_values


def get_msg_dict(bag_path):
    storage_id = 'sqlite3'

    # Only these topics will be extracted
    target_topics = [
        '/ref_spd',
        '/odometry/filtered',
        '/aeb_triggered'
    ]

    if not os.path.exists(bag_path):
        print(f"Bag path '{bag_path}' does not exist.")
        raise Exception("Path to bag not found")

    out_dict = extract_messages(bag_path, storage_id, target_topics)
    return out_dict

def get_messages(bag_path):
    storage_id = 'sqlite3'

    # Only these topics will be extracted
    target_topics = [
        '/ref_spd',
        '/odometry/filtered',
        '/aeb_triggered'
    ]

    if not os.path.exists(bag_path):
        print(f"Bag path '{bag_path}' does not exist.")
        raise Exception("Path to bag not found")

    out_dict = extract_messages(bag_path, storage_id, target_topics)
    
    ref_map  = dict(zip(out_dict["ref_spd"]["t"], out_dict["ref_spd"]["val"]))
    aeb_map  = dict(zip(out_dict["aeb_trg"]["t"], out_dict["aeb_trg"]["val"]))
    odom_map = {
        t: (x, y, v, q)
        for t, x, y, v, q in zip(
            out_dict["odom_filt"]["t"],
            out_dict["odom_filt"]["pos_x"],
            out_dict["odom_filt"]["pos_y"],
            out_dict["odom_filt"]["vel_fw"],
            out_dict["odom_filt"]["orient_q"]
        )
    }

    # Compute the sorted union of all timestamps
    all_times = set(out_dict["ref_spd"]["t"]) | set(out_dict["aeb_trg"]["t"]) | set(out_dict["odom_filt"]["t"])
    master_t = sorted(all_times)

    # Walk through master_t and pull from each map (defaulting to None)
    msgs = {
        "t": master_t,
        "ref_spd":  [],
        "aeb_trg":  [],
        "odom":    []
    }

    for t in master_t:
        msgs["ref_spd"].append(ref_map.get(t, None))
        msgs["aeb_trg"].append(aeb_map.get(t, None))
        x, y, v, oq = odom_map.get(t, (None, None, None, None))
        msgs["odom"].append([x, y, v, oq])

    return msgs






def main():
    # >>>>> Set your ROS2 bag path here <<<<<
    bag_path = '/home/gianfi/Documents/KvaserCar_outdoor_test/recorded_rosbag/rosbag2_1970_01_01-02_30_13'
    storage_id = 'sqlite3'

    # Only these topics will be extracted
    target_topics = [
        '/ref_spd',
        '/odometry/filtered',
        '/aeb_triggered'
    ]

    if not os.path.exists(bag_path):
        print(f"Bag path '{bag_path}' does not exist.")
        return

    #out_dict = extract_messages(bag_path, storage_id, target_topics)

    #print(out_dict)
    di = get_messages(bag_path)
    print(di)


if __name__ == '__main__':
    main()
