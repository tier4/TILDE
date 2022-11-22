#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-

# TILDE(MTT) aggregator node proto


from asyncio import current_task
from atexit import register
from cgi import test
from copy import deepcopy
from email.message import Message
from re import I
import re
import sys
import oyaml as yaml
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from rclpy.clock import Clock, ClockType
from rclpy.time import Time

from std_msgs.msg import String
from tilde_msg.msg import *
from tilde_aggregator_interfaces.msg import *

from .tilde_agg_func import *
from .tilde_agg_com import *
from .tilde_agg_test import *

TARGET_TOPIC = 'message_tracking_tag'
ORIGINAL_TOPIC = 'tilde_aggregator_command'
TICK_TIME = 0.1
RECONFIG_TIME = 10.0

class TildeAggregateNode(Node):
    yaml_fn = ''
    sub_list = []
    static_path_list = []
    target_topics = []
    target_list = []
    active_path_list = []
    tick_prev = 0.0
    lock = threading.RLock()
    mode = None
    deadline_miss_pub = None
    tilde_aggregator_statis_pub = None
    cb_statis = CbStatisList()

    def __init__(self):
        super().__init__('tilde_aggregator_node')

        # read path list file
        self.declare_parameter("mode", "")
        self.mode = str(self.get_parameter("mode").value)
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
            print(f"{location()}: [Exception] ## {e}: {fn}", file=sys.stderr)
            sys.exit(-1)
        
        # make path_list
        make_static_path_list_from_yaml(fn, self.static_path_list)
        if len(self.static_path_list) == 0:
            print(f"## invalid file: {fn}", file=sys.stderr)
            sys.exit(-1)
        make_active_path_list(self.static_path_list, self.active_path_list)

        if 'TEST' in self.mode:
            test_init(self, self.mode)

        cb_statis_init(self)

        print(f"\n\n*********** START *************** ({fn})", flush=True)

    # callbacks
    def reconfig_callback(self):
        #frame = inspect.currentframe()
        #func = frame.f_code.co_name
        func = 'reconfig_callback'
        with self.lock:
            cb_statis_enter(self, func)
            cur_time = nano_to_sec(Clock(clock_type=ClockType.ROS_TIME).now())
            for active_path in self.active_path_list:
                if len(active_path.pending_msg) > 0:
                    for k, tinfo in active_path.pending_msg[-1].items():
                        past_time = stamp_to_sec(tinfo.msg.output_info.pub_time_steady)
                        if (cur_time - past_time) > RECONFIG_TIME * 10:
                            active_path.pending_msg.clear()
                        break
                if len(self.target_list) != 0 and len(g_not_ready_topics) == 0:
                    return
                #self.get_logger().info('reconfig current MTT topics')
                try:
                    self.mtt_topic_subscribe()
                except Exception as e:
                    print(f"{location()}: [Exception] {e}", flush=True)
            cb_statis_exit(self, func)

    def tick_callback(self):
        #frame = inspect.currentframe()
        #func = frame.f_code.co_name
        func = 'tick_callback'
        with self.lock:
            cb_statis_enter(self, func)
            for active_path in self.active_path_list:
                if len(active_path.active_path) == 0:
                    break
                try:
                    cur_time = nano_to_sec(Clock(clock_type=ClockType.ROS_TIME).now())
                    ctick = TICK_TIME
                    if self.tick_prev != 0.0:
                        ctick = cur_time - self.tick_prev
                    ctick = max(ctick, TICK_TIME)
                    self.tick_prev = cur_time
                    deadline_check_and_proc(self, active_path, cur_time, ctick)
                except Exception as e:
                    print(f"{location()}: [Exception] {e}", flush=True)
            cb_statis_exit(self, func)

    def retrieve_pending(self, active_path):
        re_proc_topics = []
        c1 = c2 = cp = 0
        pre_len = len(active_path.pending_msg)
        post_len = 0
        while(True):
            hit = False
            c1 += 1
            for w in active_path.pending_msg:
                for tpname, tp in w.items():
                    c2 += 1
                    pi, ti = get_path_list(self.static_path_list, tpname)
                    if pi == None:
                        PP(f"{location()}: ## get_path_list() error {tpname}")
                        break
                    active_path.pending_msg.remove(w)
                    re_proc_topics.append(w)
                    if topic_receive_main(self, tp.msg, active_path, tpname, pi, ti, tp.recv_time) == True:
                        cp += 1
                        post_len = len(active_path.pending_msg)
                        DP(f"{location()}: pending {tpname} mlen({pre_len}->{post_len}) {cp=} {c1=} {c2=}")
                        hit = True
                        re_proc_topics.remove(w)
                    break
            if hit != True:
                break
            if len(re_proc_topics) != 0:
                active_path.pending_msg += re_proc_topics
            #PP(f"{location()}: pending exit {pre_len=} {post_len=} {cp=} {c1=} {c2=}")

    def top_topic_callback_wrapper(self, topic_name, active_path, pinfo, tinfo):
        #DP(f"{location()}: [SET] {topic_name=}: {tinfo=}")
        def top_topic_callback(msg):
            global g_not_ready_topics
            #frame = inspect.currentframe()
            #func = frame.f_code.co_name
            func = 'top_topic_callback'
            if 'TEST' in self.mode:
                test_top_topic_cb(self, topic_name, msg)
            with self.lock:
                cb_statis_enter(self, func)
                try:
                    if topic_name in g_not_ready_topics:
                        g_not_ready_topics.remove(topic_name)
                    if is_dup_top_topic(active_path, topic_name, msg) == True:
                        return
                    sub_time = nano_to_sec(Clock(clock_type=ClockType.ROS_TIME).now())
                    msg_dump(pinfo.path_name, topic_name, msg, sub_time)
                    calc_statis_hz(active_path, topic_name, msg)
                    #DP(f"{location()}: [top_callback] {topic_name=}")
                    #self.get_logger().debug('Read topic: ' + topic_name)
                except Exception as e:
                    print(f"{location()}: [Exception] {e}", flush=True)
                    return

                try:
                    top_topic_receive_main(msg, active_path, topic_name, pinfo, tinfo, sub_time)
                    # pending message checking
                    #pend = copy.deepcopy(active_path.pending_msg)
                    self.retrieve_pending(active_path)
                except Exception as e:
                    print(f"{location()}: [Exception] {e}", flush=True)
                cb_statis_exit(self, func)
        return top_topic_callback

    def mtt_topic_callback_wrapper(self, topic_name, active_path, pinfo, tinfo):
        #DP(f"{location()}: [SET] {topic_name=}: {tinfo=}")
        def topic_callback(msg):
            global g_not_ready_topics
            #frame = inspect.currentframe()
            #func = frame.f_code.co_name
            func = 'topic_callback'
            if 'TEST' in self.mode:
                if test_topic_cb(self, topic_name, msg, tinfo) != True:
                    return
            with self.lock:
                cb_statis_enter(self, func)
                try:
                    if topic_name in g_not_ready_topics:
                        g_not_ready_topics.remove(topic_name)
                    if is_dup_topic(active_path, topic_name, msg) == True:
                        return
                    sub_time = nano_to_sec(Clock(clock_type=ClockType.ROS_TIME).now())
                    msg_dump(pinfo.path_name, topic_name, msg, sub_time)
                    calc_statis_hz(active_path, topic_name, msg)
                    #DP(f"{location()}: [callback] {topic_name=}")
                    #self.get_logger().debug('Read topic: ' + topic_name)
                except Exception as e:
                    print(f"{location()}: [Exception] {e}", flush=True)
                    return

                try:
                    if topic_receive_main(self, msg, active_path, topic_name, pinfo, tinfo, sub_time) != True:
                        set_pending(active_path, topic_name, msg, tinfo, sub_time, False)
                        return
                    # pending message checking
                    #pend = copy.deepcopy(active_path.pending_msg)
                    self.retrieve_pending(active_path)
                except Exception as e:
                    print(f"{location()}: [Exception] {e}", flush=True)
                cb_statis_exit(self, func)
        return topic_callback

    def original_command_callback(self, msg):
        #frame = inspect.currentframe()
        #func = frame.f_code.co_name
        func = 'original_command_callback'
        with self.lock:
            cb_statis_enter(self, func)
            try:
                if msg.command == COMMAND_SHOW_INNER:
                    for active_path in self.active_path_list:
                        show_inner_state(msg.command, self.static_path_list, self.target_list, active_path)
                elif msg.command == COMMAND_SHOW_PENDING:
                    for active_path in self.active_path_list:
                        show_inner_state(msg.command, self.static_path_list, self.target_list, active_path)
                elif msg.command == COMMAND_SHOW_STATIS:
                    show_statis(msg.command, self)
                elif msg.command == COMMAND_SHOW_CB_STATIS:
                    show_cb_statis(msg.command, self)
                elif msg.command == COMMAND_REQ_STATIS_TOPIC:
                    show_statis(msg.command, self)
                elif msg.command == COMMAND_CLR_STATIS:
                    print(f"\n### clear statistics ###", flush=True)
                    clr_statis(msg.command, self)
                elif msg.command == COMMAND_CLR_PEND:
                    print(f"\n### clear pending messages ###", flush=True)
                    for active_path in self.active_path_list:
                        active_path.pending_msg.clear()
                elif msg.command == COMMAND_MSG_DUMP_ON:
                    print(f"\n### MSG DUMP ON ###", flush=True)
                    msg_ctrl(True)
                elif msg.command == COMMAND_MSG_DUMP_OFF:
                    print(f"\n### MSG DUMP OFF ###", flush=True)
                    msg_ctrl(False)
                elif msg.command == COMMAND_HIDDEN:
                    print(f"\n### Hidden disp ###", flush=True)
                    pub_disp_ctrl(False)
                elif msg.command == COMMAND_DISP:
                    print(f"\n### Disp info ###", flush=True)
                    pub_disp_ctrl(True)
                elif msg.command == COMMAND_DEBUG_ON:
                    print(f"\n### DEBUG ON ###", flush=True)
                    debug_ctrl(True)
                elif msg.command == COMMAND_DEBUG_OFF:
                    print(f"\n### DEBUG OFF ###", flush=True)
                    debug_ctrl(False)
                elif msg.command == COMMAND_RESET:
                    print(f"\n\n### RESET ###\n", flush=True)
                    self.reset_sub()
                else:
                    print(f"\n\n### Invalid command: {msg.command} ###\n", flush=True)
            except Exception as e:
                print(f"{location()}: [Exception] {e}", flush=True)
            cb_statis_exit(self, func)

###
    def original_topic_subscribe(self):
        profile = QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=5)
        sub = self.create_subscription(
                                        TildeAggregatorCommand, 
                                        ORIGINAL_TOPIC, 
                                        self.original_command_callback, 
                                        profile
                                        )

    def mtt_topic_subscribe(self):
        global g_not_ready_topics
        profile = QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=5)
        
        if self.mode == 'check1':
            check1_recv_topic(self, profile)
            return
        if self.mode == 'check2':
            check2_recv_topic(self, profile)
            return

        # normal process
        if self.mode == 'normal' or 'TEST' in self.mode:
            cur_topics = [w[0] for w in self.get_topic_names_and_types() if TARGET_TOPIC in w[0]]
        elif self.mode == 'simple' or 'TEST' in self.mode:
            cur_topics = self.target_topics

        target_list = active_set_path_list(self.static_path_list, cur_topics)
        if set(self.target_list) == set(target_list) and len(g_not_ready_topics) != 0:
            g_not_ready_topics = []
            for pinfo in self.static_path_list:
                if pinfo.top_topic.name not in target_list:
                    g_not_ready_topics.append(pinfo.top_topic.name)
                for w in pinfo.topic_dict.keys():
                    if w not in target_list:
                        g_not_ready_topics.append(w)
            if len(g_not_ready_topics) == 0:
                print(f"\n------ All MTT topics ready -------\n")
            else:
                print(f"## not ready to MTT topics: {g_not_ready_topics}\n")
            return
        if len(target_list) == 0:
            return
        diff_list = list(set(target_list) - set(self.target_list))
        self.target_list += diff_list
        DP(f"{location()}: !!! {self.target_list=}")
        #for mtt_topic in target_list:
        for mtt_topic in diff_list:
            if self.is_registered_subscriber(mtt_topic) == True:
                continue
            pinfo, tinfo = get_path_list(self.static_path_list, mtt_topic)
            if pinfo == None:
                PP(f"{location()}: ## get_path_list() error {mtt_topic}")
                break
            #DP(f"{location()}: [subsctiption] {mtt_topic} {tinfo=}")
            if pinfo == None or tinfo == None:
                DP(f"{location()}: ## Nothing path_list or topic_info {mtt_topic}")
                return
            l = [w for w in self.active_path_list if w.path_name == pinfo.path_name]
            if len(l) != 1:
                print(f"#### {location()}: inner error {pinfo.path_name} {self.active_path_list}", flush=True)
                sys.exit(-1)
            active_path = l[0]
            if mtt_topic == pinfo.top_topic.name:
                # top topic
                #print(f"{location()}: TOP create_sub {mtt_topic=}")
                sub = self.create_subscription(
                                                MessageTrackingTag, 
                                                mtt_topic, 
                                                self.top_topic_callback_wrapper(mtt_topic, active_path, pinfo, tinfo), 
                                                profile
                                                )
            else:
                # other topics
                #print(f"{location()}: OTH MTT create_sub {mtt_topic=}")
                sub = self.create_subscription(
                                                MessageTrackingTag, 
                                                mtt_topic, 
                                                self.mtt_topic_callback_wrapper(mtt_topic, active_path, pinfo, tinfo), 
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
        if len(self.sub_list) == len(self.target_topics):
            print(f"\n\n--- All topics subscription ready ---\n\n", flush=True)

    def reset_sub(self):
        global g_complete_count, g_deadline_miss_count, g_not_ready_topics
        try:
            self.sub_list = []
            self.static_path_list = []
            self.target_topics = []
            self.target_list = []
            self.active_path_list = []
            self.tick_prev = 0.0

            # make path_list
            fn = self.get_parameter("path_list").value
            make_static_path_list_from_yaml(fn, self.static_path_list)
            if len(self.static_path_list) == 0:
                print(f"## invalid file: {fn}", file=sys.stderr)
                sys.exit(-1)
            make_active_path_list(self.static_path_list, self.active_path_list)

            #for sub in [w.values() for w in self.sub_list]:
            #    self.destroy_subscription(sub)
            #self.sub_list.clear()
            #self.target_list.clear()
            #clr_statis(COMMAND_CLR_STATIS, self)
            #for active_path in self.active_path_list:
            #    active_path.active_path.clear()
            #    active_path.pending_msg.clear()
        except Exception as e:
            print(f"{location()}: [Exception] {e}", flush=True)

            g_complete_count = g_deadline_miss_count = 0
            g_not_ready_topics = self.target_topics
            time.sleep(1)
            # tilde aggregator topic
            self.original_topic_subscribe()
            # intial topic detect and subscription
            self.mtt_topic_subscribe()
            time.sleep(1)
            self.mtt_topic_subscribe()

def main(args=None):
    rclpy.init(args=args)

    node = TildeAggregateNode()
    try:
        node.deadline_miss_pub = node.create_publisher(TildeAggregatorDeadlineMiss, DEADLINE_MISS_TOPIC, 10)
        node.tilde_aggregator_statis_pub = node.create_publisher(TildeAggregatorStatistics, TILDE_AGGREGATOR_STATIS, 5)
        time.sleep(1)
        # tilde aggregator topic
        node.original_topic_subscribe()
        # intial topic detect and subscription
        node.mtt_topic_subscribe()
        time.sleep(1)
        node.mtt_topic_subscribe()
        # timer callback 
        if 'check' not in node.mode:
            node.create_timer(RECONFIG_TIME, node.reconfig_callback)
        node.create_timer(TICK_TIME, node.tick_callback)

        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException as e:
        print(f"{location()}: [Exception] {e}", flush=True)
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
