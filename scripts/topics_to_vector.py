#!/usr/bin/env python2
'''
    Since the roscpp library does not provide a way of doing introspection
    of ROS messages (i.e., accessing the fields of a message like a dictionary)
    I wrote this code to subscribe to a list of topics, get a subset of the
    fields from the listed topics, and publish them as a vector of doubles.
'''
import sys
import numpy as np
import traceback
from collections import OrderedDict, Iterable
from numbers import Number

import roslib
import rospy
import message_filters
from message_filters import ApproximateTimeSynchronizer

from robot_learning.msg import ExperienceData
from robot_learning.srv import T2VInfo
from std_msgs.msg import Header

import math


# load_package and load message are shamelessly copied from the
# rosserial_python SerialClient code
def load_package(package_name, directory):
    # check if its in the python path
    in_path = False
    path = sys.path
    # check for the source directory which
    # is added to path by roslib boostrapping
    pkg_src = package_name+'/src'
    for entry in sys.path:
        if pkg_src in entry:
            in_path = True
    if not in_path:
        roslib.load_manifest(package_name)
    try:
        package_module = __import__(package_name + '.' + directory)
    except Exception:
        rospy.logerr("Cannot import package : %s" % package_name)
        rospy.logerr("sys.path was " + str(path))
        return None
    return package_module


def load_message(package_name, message_name):
    package_module = load_package(package_name, 'msg')
    msg_submodule = getattr(package_module, 'msg')
    return getattr(msg_submodule, message_name)
# end of stolen code


def get_all_numeric_fields(msg, ignore_header=True):
    # if we have a number type, return a List with a single element
    if isinstance(msg, Number):
        return [msg]
    # else if it is already a list or tuple, return it
    elif (type(msg) is tuple) or (type(msg) is list):
        return msg
    # else, recurse, append to List, and returnd that List
    else:
        return_val = []
        for field_name in msg.__slots__:
            return_val.extend(
                get_all_numeric_fields(getattr(msg, field_name)))
        return return_val


def recursive_getattr(obj, attr):
    if '.' not in attr and '[' not in attr:
        return getattr(obj, attr)
    elif '.' not in attr and '[' in attr:
        attr_data = attr.split('[', 1)
        index = int(attr_data[1].split(']', 1)[0])
        attr_list = getattr(obj, attr_data[0])
        # return eval('attr_list['+index+']')
        return attr_list[index]
    else:
        attr_data = attr.split('.', 1)
        return recursive_getattr(getattr(obj, attr_data[0]), attr_data[1])


def recursive_setattr(obj, attr, value):
    if '.' not in attr and '[' not in attr:
        try:
            obj_attr = getattr(obj, attr)
            if isinstance(obj_attr, list):
                if isinstance(value, Iterable):
                    setattr(obj, attr, [x for x in value])
                else:
                    for i in xrange(len(obj_attr)):
                        obj_attr[i] = value
                    setattr(obj, attr, obj_attr)
            else:
                setattr(obj, attr, value)
            return True
        except AttributeError:
            print "Attribute "+str(value)+" not found in message class"
            return False
    elif '.' not in attr and '[' in attr:
        try:
            attr_data = attr.split('[', 1)
            index = int(attr_data[1].split(']', 1)[0])
            attr_list = getattr(obj, attr_data[0])
            attr_list[index] = value
            setattr(obj, attr_data[0], attr_list)
            return True
        except AttributeError:
            print "Attribute "+str(value)+" not found in message class"
            return False
    else:
        attr_data = attr.split('.', 1)
        return recursive_getattr(
            getattr(obj, attr_data[0]), attr_data[1], value)


class SubscriptionManager:
    STATE_SUB = 0
    COMMAND_SUB = 1

    def __init__(self, slop):
        self.state_subscribers = OrderedDict()
        self.command_subscribers = OrderedDict()
        self.state_time_sync = None
        self.command_time_sync = None
        self.filtered_fields = OrderedDict()
        self.operations = OrderedDict()
        self.topic_types = OrderedDict()
        self.last_msg = OrderedDict()
        self.experience_data = ExperienceData()
        self.publish_on_command = False
        self.slop = slop

        # setup the experience data publisher
        self.experience_pub = rospy.Publisher(
            "/rl/experience_data", ExperienceData, queue_size=1)

        # setup service to query state info
        self.state_dims_srv = rospy.Service(
            '/rl/state_dims', T2VInfo, self.state_dims)
        self.command_dims_srv = rospy.Service(
            '/rl/command_dims', T2VInfo, self.command_dims)

        # setup a timer for publishing
        dt = rospy.Duration(1.0/(rospy.get_param("~rate", 50)))
        rospy.loginfo('dt: %s' % (1/(dt.secs+1e-9*dt.nsecs)))
        rospy.Timer(dt, self.publishExperience)

        # setup the preprocessing operations
        self.operations = rospy.get_param("~preprocessing_operations", {})
        self.preprocess = {'square': lambda x: x**2,
                           'abs':  math.fabs,
                           'absolute':  math.fabs}
        for fname in math.__dict__:
            f = math.__dict__[fname]
            if callable(f):
                self.preprocess[fname] = f
        for fname in np.__dict__:
            if fname not in self.preprocess:
                f = np.__dict__[fname]
                if callable(f):
                    self.preprocess[fname] = f

    def setupSubscriber(self, topic_data, subscriber_type=0):
        # load message type
        topic_name = topic_data['topic_name']
        message_type = topic_data['type']
        m = load_message(message_type['package'], message_type['name'])

        # init subscriber
        if subscriber_type == self.STATE_SUB:
            self.state_subscribers[topic_name] = message_filters.Subscriber(
                topic_name, m)
        elif subscriber_type == self.COMMAND_SUB:
            self.command_subscribers[topic_name] = message_filters.Subscriber(
                topic_name, m)
        else:
            return

        # store the topic_type
        self.topic_types[topic_name] = m
        # setup the topic field filters
        self.filtered_fields[topic_name] = topic_data.get(
            'filter', m.__slots__)

    def startListening(self):
        if len(self.state_subscribers) > 0:
            self.state_time_sync = ApproximateTimeSynchronizer(
                self.state_subscribers.values(), 1, self.slop)
            self.state_time_sync.registerCallback(self.stateCallback)
        if len(self.command_subscribers) > 0:
            self.command_time_sync = ApproximateTimeSynchronizer(
                self.command_subscribers.values(), 1, self.slop)
            self.command_time_sync.registerCallback(self.commandCallback)
            self.publish_on_command = True

    def topicsToVector(self, subscribers, preprocess_enabled, *args):
        # for each of the received messages in args, we are going to retrieve
        # the fields specified in self.filtered_fields['topic_name'] and put
        # them in a List of floating point numbers
        # Warning: this assumes that the arguments, which are synchronized
        # messages from multiple topics,  will come in the same order as
        # specified in the constructor of the self.time_sync object (which
        # corresponds to the order of insertion into the subscribers
        # OrderedDict. This behaviour could be broken in any subsequent ROS
        # releases
        vector_data = []
        for msg, topic_name in zip(args, subscribers.keys()):
            self.last_msg[topic_name] = msg
            try:
                # get the list of fields we want to get from corresponding to
                # the current topic
                filtered_fields = self.filtered_fields[topic_name]
                for field in filtered_fields:
                    field_value = recursive_getattr(msg, field)
                    if hasattr(field_value, '_type'):
                        # ignore the header type
                        if field_value._type != 'std_msgs/Header':
                            numeric_fields = get_all_numeric_fields(
                                field_value)
                            if preprocess_enabled:
                                for i in xrange(len(numeric_fields)):
                                    operations = self.operations.get(field, [])
                                    for op in operations:
                                        numeric_fields[i] = self.preprocess[op](
                                            numeric_fields[i])
                            vector_data.extend(numeric_fields)
                    else:
                        numeric_fields = get_all_numeric_fields(field_value)
                        if preprocess_enabled:
                            for i in xrange(len(numeric_fields)):
                                operations = self.operations.get(field, [])
                                for op in operations:
                                    numeric_fields[i] = self.preprocess[op](
                                        numeric_fields[i])
                        vector_data.extend(numeric_fields)
            except Exception:
                print topic_name
                print '------------'
                print msg
                print '============'
                traceback.print_exc()
        return vector_data

    def stateCallback(self, *args):
        # transform the state messages into the state_data vector
        self.experience_data.state_data = self.topicsToVector(
            self.state_subscribers, True, *args)

    def commandCallback(self, *args):
        # transform the command messages into the command_data vector
        self.experience_data.command_data = self.topicsToVector(
            self.command_subscribers, False, *args)

    def publishExperience(self, event):
        self.experience_data.header = Header()
        self.experience_data.header.stamp = rospy.Time.now()
        if len(self.experience_data.state_data) > 0:
            self.experience_pub.publish(self.experience_data)
        self.experience_data.state_data = []

    def count_fields(self, subscribers):
        n = 0
        for topic in subscribers:
            if topic in self.last_msg:
                msg = self.last_msg[topic]
            else:
                msg = self.topic_types[topic]()
            for field in self.filtered_fields[topic]:
                field_value = recursive_getattr(msg, field)
                if not (hasattr(field_value, '_type')
                        and field_value._type == 'std_msgs/Header'):
                    numeric_fields = get_all_numeric_fields(field_value)
                    n += len(numeric_fields)
        return n

    def command_dims(self, req):
        return self.count_fields(self.command_subscribers)

    def state_dims(self, req):
        return self.count_fields(self.state_subscribers)


class PublisherManager:
    def __init__(self):
        self.command_publishers = OrderedDict()
        self.topic_types = OrderedDict()
        self.filtered_fields = OrderedDict()
        self.default_values = OrderedDict()

        # setup the command data callback
        self.command_sub = rospy.Subscriber(
            "/rl/command_data", ExperienceData, self.commandDataCallback)

        # setup the preprocessing operations
        self.operations = rospy.get_param("~preprocessing_operations", {})
        self.preprocess = {'square': lambda x: x**2,
                           'abs':  math.fabs,
                           'absolute':  math.fabs}
        for fname in math.__dict__:
            f = math.__dict__[fname]
            if callable(f):
                self.preprocess[fname] = f

    def setupPublisher(self, topic_data):
        # load message type
        topic_name = topic_data['topic_name']
        message_type = topic_data['type']
        m = load_message(message_type['package'], message_type['name'])

        # init publishers
        self.command_publishers[topic_name] = rospy.Publisher(
            topic_name, m, queue_size=1)

        # store the topic_type
        self.topic_types[topic_name] = m
        # setup the topic field filters
        self.filtered_fields[topic_name] = topic_data.get(
            'filter', m.__slots__)
        # setup the preprocessing operations
        self.default_values[topic_name] = topic_data.get('default_values', {})

    def vectorToTopic(self, vector_data, topic):
        # create a new message object
        msg = self.topic_types[topic]()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        if isinstance(vector_data, tuple):
            vector_data = [x for x in vector_data]

        # use the filtered fields to populate the correct fields in msg
        idx = 0
        for field in self.filtered_fields[topic]:
            operations = self.operations.get(field, [])
            msg_field = recursive_getattr(msg, field)
            if isinstance(msg_field, list):
                values = vector_data[idx:idx+len(msg_field)]
                for i in xrange(len(msg_field)):
                    for operation in operations:
                        values[i] = self.preprocess[operation](values[i])
                if not recursive_setattr(msg, field, values):
                    rospy.logerror(
                        "Failed to set field %d for topic %d" % (field, topic))
                idx += len(msg_field)
            else:
                value = vector_data[idx]
                for operation in operations:
                    value = self.preprocess[operation](value)
                if not recursive_setattr(msg, field, value):
                    rospy.logerror(
                        "Failed to set field %d for topic %d" % (field, topic))
                idx += 1
        # populate the default values
        default_values = self.default_values[topic]
        for field in default_values:
            if not recursive_setattr(msg, field, default_values[field]):
                rospy.logerror(
                    "Failed to set field %d for topic %d" % (field, topic))

        # publish the message
        self.command_publishers[topic].publish(msg)

    def commandDataCallback(self, msg):
        # the elements from the vector will populate the fields in the messages
        # for the publishers in the order specified by the keys of the
        # command_publishers dictionary and the elements of the filtered_fields
        # dictionary
        # E.g. consider the follwing setup:
        #  1. The incoming vector has 6 elements
        #  2. There are 2 command publishers.
        #  3. The filtered fields for the first topic are [field_1,field_2]
        #  4. The filtered fields for the first topic are
        #     [field_3, field_4, field_5, field_6]
        # Then, the I-th element of the incoming vector will be copied to
        # field_I
        element_index = 0
        for topic in self.command_publishers.keys():
            vector_data = msg.command_data[
                element_index:element_index+len(self.filtered_fields[topic])]
            self.vectorToTopic(vector_data, topic)
            element_index += len(self.filtered_fields[topic])


if __name__ == '__main__':
    rospy.init_node('topics_to_vector')
    # get the list of topics
    state_topics_list = rospy.get_param("~experience_state_topics", [])
    command_topics_list = rospy.get_param("~experience_command_topics", [])
    # get the delay (in seconds) within which messages can be synchronized
    slop = rospy.get_param("~slop", 0.1)
    rospy.loginfo('slop: %s' % (slop))
    sm = SubscriptionManager(slop)
    pm = PublisherManager()

    # setup subscribers
    for topic in state_topics_list:
        sm.setupSubscriber(topic, SubscriptionManager.STATE_SUB)
    for topic in command_topics_list:
        sm.setupSubscriber(topic, SubscriptionManager.COMMAND_SUB)
        pm.setupPublisher(topic)
    sm.command_dims(None)

    # start listening for messages
    sm.startListening()

    rospy.spin()
