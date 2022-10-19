#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-

# TILDE(MTT) aggregator node proto


from atexit import register
from copy import deepcopy
from email.message import Message
from re import I
import sys
import oyaml as yaml
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from rclpy.clock import Clock, ClockType
from rclpy.time import Time

from std_msgs.msg import String
#from tilde_msg.msg.MessageTrackingTag.msg import *
from tilde_msg.msg import *
from tilde_aggregator_interfaces.msg import *

from .tilde_agg_func import *
from .tilde_agg_com import *

TARGET_TOPIC = 'message_tracking_tag'
ORIGINAL_TOPIC = 'tilde_aggregator_command'
TICK_TIME = 0.1
RECONFIG_TIME = 10.0
g_msg_dump_enable = True

class TildeAggregateNode(Node):
    sub_list = []
    static_path_list = []
    target_list = []
    active_path_list = ActivePathlist()
    lock = threading.RLock()

    def __init__(self):
        super().__init__('TildeAggregateNode')

        # read path list file
        self.declare_parameter("mode", "")
        mode = str(self.get_parameter("mode").value)
        try:
            self.declare_parameter("path_list", "")
            fn = self.get_parameter("path_list").value
            DP(f"{location()}: path_list={fn}")
            if fn == None or fn == '':
                print(f"## not spicified yaml file", file=sys.stderr)
                sys.exit(-1)
            if os.path.exists(fn) == False:
                print(f"## not found yaml file: {fn}", file=sys.stderr)
                sys.exit(-1)
        except Exception as e:
            print(f"## {e}: {fn}", file=sys.stderr)
            sys.exit(-1)
        
        # make path_list
        make_path_list_from_yaml(fn, self.static_path_list)
        if len(self.static_path_list) == 0:
            print(f"## {e}: {fn}", file=sys.stderr)
            sys.exit(-1)

        DP(f"*********** START ***************\n!!! {location()}: {fn=}")

        # tilde aggregator topic
        self.original_topic_subscribe()
        # intial topic detect and subscription
        self.mtt_topic_subscribe()
        # timer callback 
        self.create_timer(RECONFIG_TIME, self.reconfig_callback)
        self.create_timer(TICK_TIME, self.tick_callback)

    # callbacks
    def reconfig_callback(self):
        self.get_logger().info('reconfig current MTT topics')
        with self.lock:
            try:
                self.mtt_topic_subscribe()
            except Exception as e:
                print(f"{location()}: [Exception] {e}", flush=True)

    def tick_callback(self):
        if len(self.active_path_list.active_path) == 0:
            return
        cur_time = nano_to_sec(Clock(clock_type=ClockType.ROS_TIME).now())
        with self.lock:
            try:
                deadline_check_and_proc(self.active_path_list, cur_time, TICK_TIME)
            except Exception as e:
                print(f"{location()}: [Exception] {e}", flush=True)

    def top_topic_callback_wrapper(self, topic_name, pinfo, tinfo):
        #DP(f"{location()}: [SET] {topic_name=}: {tinfo=}")
        def top_topic_callback(msg):
            sub_time = nano_to_sec(Clock(clock_type=ClockType.ROS_TIME).now())
            msg_dump(pinfo.path_name, topic_name, msg, sub_time)
            calc_statis_hz(pinfo.path_name, topic_name, msg)
            #DP(f"{location()}: [top_callback] {topic_name=}")
            self.get_logger().debug('Read topic: ' + topic_name)
            with self.lock:
                try:
                    top_topic_receive_main(msg, self.active_path_list, topic_name, pinfo, tinfo, sub_time)
                    # pending message checking
                    cppend = copy.deepcopy(self.active_path_list.pending_msg)
                    #for w in self.active_path_list.pending_msg:
                    for w in cppend:
                        for tpname, tp in w.items():
                            self.active_path_list.pending_msg.remove(w)
                            #DP(f"{location()}: #### {tpname=}: {tp}")
                            topic_receive_main(tp.msg, self.active_path_list, tpname, pinfo, tp, tp.recv_time)
                except Exception as e:
                    print(f"{location()}: [Exception] {e}", flush=True)
        return top_topic_callback

    def mtt_topic_callback_wrapper(self, topic_name, pinfo, tinfo):
        #DP(f"{location()}: [SET] {topic_name=}: {tinfo=}")
        def topic_callback(msg):
            sub_time = nano_to_sec(Clock(clock_type=ClockType.ROS_TIME).now())
            msg_dump(pinfo.path_name, topic_name, msg, sub_time)
            calc_statis_hz(pinfo.path_name, topic_name, msg)
            #DP(f"{location()}: [callback] {topic_name=}")
            self.get_logger().debug('Read topic: ' + topic_name)
            with self.lock:
                try:
                    topic_receive_main(msg, self.active_path_list, topic_name, pinfo, tinfo, sub_time)
                    # pending message checking
                    cppend = copy.deepcopy(self.active_path_list.pending_msg)
                    #for w in self.active_path_list.pending_msg:
                    for w in cppend:
                        for tpname, tp in w.items():
                            self.active_path_list.pending_msg.remove(w)
                            #DP(f"{location()}: #### {tpname=}: {tp}")
                            topic_receive_main(tp.msg, self.active_path_list, tpname, pinfo, tp, tp.recv_time)
                except Exception as e:
                    print(f"{location()}: [Exception] {e}", flush=True)
        return topic_callback

    def original_command_callback(self, msg):
        global g_msg_dump_enable, DEBUG
        with self.lock:
            try:
                if msg.command == COMMAND_SHOW_INNER:
                    com_show_inner_state(self.static_path_list, self.target_list, self.active_path_list)
                elif msg.command == COMMAND_SHOW_STATIS:
                    com_show_statis(msg.command)
                elif msg.command == COMMAND_SHOW_CLR_STATIS:
                    com_show_statis(msg.command)
                elif msg.command == COMMAND_MSG_DUMP_ON:
                    g_msg_dump_enable = True
                elif msg.command == COMMAND_MSG_DUMP_OFF:
                    g_msg_dump_enable = False
                elif msg.command == COMMAND_DEBUG_ON:
                    DEBUG = True
                elif msg.command == COMMAND_DEBUG_OFF:
                    DEBUG = False
            except Exception as e:
                print(f"{location()}: [Exception] {e}", flush=True)

###
    def original_topic_subscribe(self):
        profile = QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=5)
        sub = self.create_subscription(
                                        TildeAggregatorCommand, 
                                        ORIGINAL_TOPIC, 
                                        self.original_command_callback, 
                                        profile
                                        )


    ##########
    def default_callback(self, msg):
            self.get_logger().info('RECV msg=%s' % msg)
    ##########

    def mtt_topic_subscribe(self):
        profile = QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=5)
        cur_topics = self.get_topic_names_and_types()
        """
        ##########
        for w in cur_topics:
            DP(f" [{w[0]}]: {w[1]} get_topic")
            if TARGET_TOPIC in w[0]:
                DP(f" [{w[0]}]: {w[1]} subscribed")
                sub = self.create_subscription(
                                                w[1], 
                                                w[0], 
                                                self.default_callback,
                                                profile
                                               )
        return
        ##########
        """
        if len(self.static_path_list) == 0:
            return
        target_list = active_set_path_list(self.static_path_list, [w[0] for w in cur_topics])
        if set(self.target_list) == set(target_list):
            DP(f"{location()}: #### unchange wait topics: {self.sub_list=}")
            return
        else:
            self.target_list = target_list
        DP(f"{location()}: !!! {target_list=}")
        for mtt_topic in target_list:
            if self.is_registered_subscriber(mtt_topic) == True:
                continue
            pinfo, tinfo = get_path_list(self.static_path_list, mtt_topic)
            #DP(f"{location()}: [subsctiption] {mtt_topic} {tinfo=}")
            if pinfo == None or tinfo == None:
                DP(f"{location()}: ## Nothing path_list or topic_info {mtt_topic}")
                return
            if mtt_topic == pinfo.top_topic.name:
                # top topic
                sub = self.create_subscription(
                                                MessageTrackingTag, 
                                                mtt_topic, 
                                                self.top_topic_callback_wrapper(mtt_topic, pinfo, tinfo), 
                                                profile
                                                )
            else:
                # other topics
                sub = self.create_subscription(
                                                MessageTrackingTag, 
                                                mtt_topic, 
                                                self.mtt_topic_callback_wrapper(mtt_topic, pinfo, tinfo), 
                                                profile
                                                )
            self.register_subscriber(sub, mtt_topic)

    def is_registered_subscriber(self, topic):
        #DP(f"{location()}")
        if topic in [w.keys() for w in self.sub_list]:
            return True
        return False

    def register_subscriber(self, sub, topic):
        #DP(f"{location()}")
        if topic not in [w.keys() for w in self.sub_list]:
            self.sub_list.append({topic: sub})

def main(args=None):
    rclpy.init(args=args)

    node = TildeAggregateNode()
    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
