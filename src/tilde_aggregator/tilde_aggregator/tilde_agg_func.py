#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
from re import I
import sys, os, time
import tarfile
import oyaml as yaml
import copy
import json
from pprint import pprint

from rclpy.clock import Clock, ClockType
from rclpy.time import Time

from std_msgs.msg import String
from tilde_aggregator_interfaces.msg import *

from .tilde_agg_com import *
from .path_info import *
from .statis import *

YAML_PARAMS = 'params'
MTT_TOPIC_POSTFIX = 'message_tracking_tag'
MAX_MSG_QUE = 256
DEADLINE_MISS_TOPIC = 'tilde_aggregator_deadline_miss'

g_complete_count = 0
g_deadline_miss_count = 0
g_pub_disp = False

def make_static_path_list_from_yaml(fn, static_list):
    try:
        with open(fn, 'r') as f:
            yaml_data = yaml.safe_load(f)
    except Exception as e:
        print(f"{location()}: [Exception] ##{e}: No such file or yaml load error {fn}", file=sys.stderr)
        sys.exit(-1)
        
    for k1 in [key for key in yaml_data.keys()]:
        if k1 == YAML_PARAMS:
            # parameters
            param = yaml_data[k1]
            DP(f"{location()}: {k1=} {param=}")
            pass # not yet undefined
        else:
            # path = topic flow list
            path_info = yaml_data[k1]
            
            DP(f"{location()}: {k1=} {path_info=}")
            pinfo = TildePathManageTable()
            pinfo.path_name = k1
            pinfo.status = ST_INIT
            pinfo.deadline_timer = path_info.get('deadline_timer', 1.0)

            for i, k2 in enumerate([key for key in path_info['topic_list']]):
                if MTT_TOPIC_POSTFIX not in k2:
                    mtt_topic = k2 + '/' + MTT_TOPIC_POSTFIX
                else:
                    mtt_topic = k2
                tp = path_info['topic_list'][k2]
                tinfo = TopicInfo()
                tinfo.name = mtt_topic
                if tp != None and len(tp) != 0:
                    tinfo.order = tp.get('order', 0)
                    tinfo.timer = tp.get('timer', 0.0)
                tinfo.status = ST_INIT
                if i == 0:
                    pinfo.top_topic = tinfo
                else:
                    pinfo.topic_dict.update({mtt_topic: tinfo})
                #DP(f"{location()}: {mtt_topic=} {tinfo=}")
            # path list
            static_list.append(pinfo)

def make_active_path_list(static_list, active_path_list):
    for pinfo in static_list:
        pstat = make_statis_path(pinfo.path_name)
        make_statis_topic(pstat, pinfo.top_topic.name)
        for tn, tinfo in pinfo.topic_dict.items():
            make_statis_topic(pstat, tn)
        active_path = ActivePathlist()
        active_path.path_name = pinfo.path_name
        active_path.path_statis = pstat
        active_path_list.append(active_path)

def active_set_path_list(path_list, topics):
    #DP(f"{location()}: {path_list=}")
    target_list = []
    for pinfo in path_list:
        complete = True
        if pinfo.top_topic.name in topics:
            target_list.append(pinfo.top_topic.name)
        else:
            complete = False
        for name, tinfo in pinfo.topic_dict.items():
            if name in topics:
                target_list.append(name)
            else:
                complete = False
        if complete == True and pinfo.status == ST_INIT:
            pinfo.status = ST_WAIT
    return target_list

def get_path_list(path_list, mtt_topic):
    if len(path_list) == 0:
        PP(f"{location()}: ## empty path list {id(path_list)=}")
        return None, None
    for pinfo in path_list:
        if mtt_topic in pinfo.top_topic.name or pinfo.top_topic.name in mtt_topic:
            return pinfo, pinfo.top_topic
        for name, tinfo in pinfo.topic_dict.items():
            #DP(f"{location()}: #### {name}: {tinfo}")
            if mtt_topic in name or name in mtt_topic:
                return pinfo, tinfo
    # not found
    topics = []
    for pinfo in path_list:
        topics.append(pinfo.top_topic.name)
        for name, tinfo in pinfo.topic_dict.items():
            topics.append(name)
            topics.append(tinfo.name)
    PP(f"{location()}: ## not registered mtt-topic {mtt_topic=}")
    pprint(f"{topics=}")
    sys.stdout.flush()
    return None, None

#
def is_dup_top_topic(active_path, topic_name, msg):
    pub_time = msg.output_info.pub_time_steady
    for pinfo in active_path.active_path:
        tinfo = pinfo.top_topic
        if tinfo.status == ST_RECVD and tinfo.msg.output_info.pub_time_steady == pub_time:
            PP(f"## Dup top topic detected ({topic_name})")
            set_statis_dup_topic(active_path, topic_name)
            return True
    for pend in [w for w in active_path.pending_msg if list(w.keys())[0] == topic_name]:
        ##l = [v for k, v in pend.items() if k == topic_name and v.status == ST_RECVD]
        ##if l == [] or l == None or len(l) == 0:
        ##    continue
        ##tinfo = l[0]
        tinfo = pend[topic_name]
        if tinfo.status == ST_RECVD and tinfo.msg.output_info.pub_time_steady == pub_time:
            PP(f"## Dup topic detected in pending queue ({topic_name})")
            set_statis_dup_topic(active_path, topic_name)
            return True
    return False

def is_dup_topic(active_path, topic_name, msg):
    pub_time = msg.output_info.pub_time_steady
    for pinfo in active_path.active_path:
        ##l = [v for k, v in pinfo.topic_dict.items() if k == topic_name and v.status == ST_RECVD]
        ##if l == [] or l == None or len(l) == 0:
        ##    continue
        ##tinfo = l[0]
        #print(f"!!! {len(l)=} {type(tinfo)=} {tinfo=}", flush=True)
        tinfo = pinfo.topic_dict[topic_name]
        if tinfo.status == ST_RECVD and tinfo.msg.output_info.pub_time_steady == pub_time:
            PP(f"## Dup topic detected ({topic_name})")
            set_statis_dup_topic(active_path, topic_name)
            return True
    for pend in [w for w in active_path.pending_msg if list(w.keys())[0] == topic_name]:
        ##l = [v for k, v in pend.items() if k == topic_name and v.status == ST_RECVD]
        ##if l == [] or l == None or len(l) == 0:
        ##    continue
        ##tinfo = l[0]
        tinfo = pend[topic_name]
        if tinfo.status == ST_RECVD and tinfo.msg.output_info.pub_time_steady == pub_time:
            PP(f"## Dup topic detected in pending queue ({topic_name})")
            set_statis_dup_topic(active_path, topic_name)
            return True
    return False

def new_path_list_add(msg, topic_name, pinfo, tinfo, sub_time):
    np = copy.deepcopy(pinfo)
    np.status = ST_START
    np.start = sub_time
    np.top_topic.status = ST_RECVD
    np.top_topic.msg = msg
    np.top_topic.recv_time = sub_time
    for k, v in np.topic_dict.items():
        #DP(f"{location()}: #### {k}: {v}")
        v.status = ST_WAIT
    return np

def register_topic_info(msg, topic_name, pinfo, sub_time):
    DP(f"{location()}: --- General topic registered {topic_name} in {pinfo.path_name} ---")
    for name, tinfo in pinfo.topic_dict.items():
        if name in topic_name and tinfo.status <= ST_WAIT:
            tinfo.msg = msg
            tinfo.recv_time = sub_time
            tinfo.status = ST_RECVD
            DP(f"{location()}: --- General topic registered {topic_name} in {pinfo.path_name} ---")
            return True
    else:
        PP(f"{location()}: #### ERROR {name}: {tinfo}")
        return False

def top_topic_receive_main(msg, active_path, topic_name, pinfo, tinfo, sub_time):
    deadline_timer_start(pinfo)
    new_pinfo = new_path_list_add(msg, topic_name, pinfo, tinfo, sub_time)
    active_path.active_path.append(new_pinfo)
    DP(f"{location()}: --- TOP topic registered {topic_name} in {pinfo.path_name} ---")

def match_topic_in_path(node, msg, active_path, topic_name, pinfo, recv_time):
    try:
        if register_topic_info(msg, topic_name, pinfo, recv_time) == False:
            return False
        concurrent = len([w for w in active_path.active_path if w.path_name == pinfo.path_name])
        if path_complete_check_and_proc(node, active_path, pinfo, recv_time, concurrent) == True:
            active_path.active_path.remove(pinfo)
            return True
        return True
    except Exception as e:
        print(f"{location()}: [Exception] {e}", flush=True)
        return False
        
def set_pending(active_path, topic_name, msg, init_tinfo, recv_time, pend):
    new_tp = copy.deepcopy(init_tinfo)
    new_tp.msg = msg
    new_tp.recv_time = recv_time
    mlen = len(active_path.pending_msg)
    if mlen >= MAX_MSG_QUE:
        if mlen > MAX_MSG_QUE / 10:
            del active_path.pending_msg[0:int(MAX_MSG_QUE / 10)]
        else:
            active_path.pending_msg.remove(active_path.pending_msg[0])
    if pend == True:
        active_path.pending_msg.insert(0, {topic_name: new_tp})
    else:
        active_path.pending_msg.append({topic_name: new_tp})

def topic_receive_main(node, msg, active_path, topic_name, init_pinfo, init_tinfo, recv_time):
    """
    - 受信トピックがどのパス(複数インスタンスあり)に含まれているか探索
      どのインスタンスかは分からないので、一つづつ確認することになる
      (前提：異なるパスに同じトピックは含まれないこと)
      - 該当トピックが受信済なら該当パスインスタンスはスキップ      
    以下は該当パス(複数インスタンスあり)内での処理
    - 受信トピックのinputトピック(複数あり) & header_stamp が、topトピック名 & header_stamp と一致
    - 受信トピックのoutputトピック & header_stamp が、他のトピックのinputトピック & header_stamp と一致
    - 受信トピックのinputトピック(複数あり) & header_stamp が、他のoutputトピック & header_stamp と一致
    """
    if len(active_path.active_path) == 0:
        return False
    ## for pinfo in [pinfo for pinfo in active_path.active_path if topic_name in pinfo.topic_dict.keys()]:
    for pinfo in active_path.active_path:
        # 該当パスの全アクティブパスを検査
        DP(f"{location()}: {topic_name=}")
        tinfo = pinfo.topic_dict[topic_name]
        if tinfo.status == ST_RECVD:
            # 既に受信済なら次のアクティブパスへ
            continue
        top_topic_nt = pinfo.top_topic.msg.output_info.topic_name
        top_has_stamp = pinfo.top_topic.msg.output_info.has_header_stamp
        top_stamp = pinfo.top_topic.msg.output_info.header_stamp
        top_pub_time = pinfo.top_topic.msg.output_info.pub_time_steady
        #for w in msg.input_infos:
        w = [mi for mi in msg.input_infos if mi.topic_name == top_topic_nt]
        if w != None and w != []:
            w = w[0]
            # 受信トピックのinputトピック(複数あり)＋sub時間がtopトピック名＋pub時間と一致
            try:
                self_sub_topic_nt = w.topic_name
                self_sub_has_stamp = w.has_header_stamp
                self_sub_stamp = w.header_stamp
                self_sub_time = w.sub_time_steady
            except Exception as e:
                print(f"{location()}: [Exception] {e}", flush=True)

            #DP(f"{location()}: pre {self_sub_topic=} {top_topic_nt=} {self_sub_stamp=} {top_stamp=}")
            #if self_sub_topic_nt == top_topic_nt:
            try:
                #DP(f"{location()}: pre {self_sub_topic=} {top_topic_nt=} {self_sub_stamp=} {top_stamp=}")
                if self_sub_has_stamp == True and top_has_stamp == True:
                    if self_sub_stamp == top_stamp:
                        # match
                        DP(f"{location()}: MATCH {self_sub_stamp=}")
                        pinfo.top_topic.resp_time = stamp_to_sec(self_sub_time) - stamp_to_sec(top_pub_time)
                        #PP(f"{location()}: {self_sub_time=} {top_pub_time} {pinfo.top_topic.resp_time=} {pinfo.top_topic.name}")
                        calc_statis_resp(active_path, pinfo.top_topic)
                        if match_topic_in_path(node, msg, active_path, topic_name, pinfo, recv_time) != True:
                            PP(f"{location()}: ## match_topic_in_path() error discard {topic_name}")
                            return False
                        return True
                    #else:
                    #    continue
                else:
                    pinfo.top_topic.resp_time = stamp_to_sec(self_sub_time) - stamp_to_sec(top_pub_time)
                    DP(f"{location()}: {pinfo.top_topic.resp_time=} {pinfo.top_topic.name}")
                    calc_statis_resp(active_path, pinfo.top_topic)
                    if match_topic_in_path(node, msg, active_path, topic_name, pinfo, recv_time) != True:
                        PP(f"{location()}: ## match_topic_in_path() error discard {topic_name}")
                        return False
                    return True
            except Exception as e:
                print(f"{location()}: [Exception] {e}", flush=True)

        for name, tinfo in pinfo.topic_dict.items():
            # パス内他outputをsubscribeしていないか検査
            if topic_name in name:
                # 自トピックは飛ばす
                continue
            if tinfo.status == ST_RECVD:
                # 他topicのpub時間が自トピック内inputトピックのsub時間と一致
                oth_pub_topic_nt = tinfo.msg.output_info.topic_name
                oth_pub_has_stamp = tinfo.msg.output_info.has_header_stamp
                oth_pub_stamp = tinfo.msg.output_info.header_stamp
                oth_pub_time = tinfo.msg.output_info.pub_time_steady
                #for w in msg.input_infos:
                w = [mi for mi in msg.input_infos if mi.topic_name == oth_pub_topic_nt]
                if w != None and w != []:
                    w = w[0]
                    # 受信トピックのinputトピック(複数あり)＋sub時間が他トピック名＋pub時間と一致
                    try:
                        self_sub_topic_nt = w.topic_name
                        self_sub_has_stamp = w.has_header_stamp
                        self_sub_stamp = w.header_stamp
                        self_sub_time = w.sub_time_steady
                    except Exception as e:
                        print(f"{location()}: [Exception] {e}", flush=True)

                    #if self_sub_topic_nt == oth_pub_topic_nt:
                    try:
                        #DP(f"{location()}: pre {self_sub_topic_nt=} {oth_pub_topic_nt=} {self_sub_stamp=} {top_stamp=}")
                        if self_sub_has_stamp == True and oth_pub_has_stamp == True:
                            try:
                                if self_sub_stamp == oth_pub_stamp:
                                    # match
                                    #DP(f"{location()}: MATCH {self_sub_stamp=}")
                                    pi, ti = get_path_list(active_path.active_path, oth_pub_topic_nt)
                                    if ti == None:
                                        PP(f"{location()}: ## get_path_list() error {oth_pub_topic_nt}")
                                    else:
                                        ti.resp_time = stamp_to_sec(self_sub_time) - stamp_to_sec(oth_pub_time)
                                        calc_statis_resp(active_path, ti)
                                    if match_topic_in_path(node, msg, active_path, topic_name, pinfo, recv_time) != True:
                                        PP(f"{location()}: ## match_topic_in_path() error discard {topic_name}")
                                        return False
                                    return True
                                #else:
                                #    continue
                            except Exception as e:
                                print(f"{location()}: [Exception] {e}", flush=True)
                        else:
                            try:
                                pi, ti = get_path_list(active_path.active_path, oth_pub_topic_nt)
                                if ti == None:
                                    PP(f"{location()}: ## get_path_list() error {oth_pub_topic_nt}")
                                else:
                                    ti.resp_time = stamp_to_sec(self_sub_time) - stamp_to_sec(oth_pub_time)
                                    calc_statis_resp(active_path, ti)
                                if match_topic_in_path(node, msg, active_path, topic_name, pinfo, recv_time) != True:
                                    PP(f"{location()}: ## match_topic_in_path() error discard {topic_name}")
                                    return False
                                return True
                            except Exception as e:
                                print(f"{location()}: [Exception] {e}", flush=True)
                    except Exception as e:
                        print(f"{location()}: [Exception] {e}", flush=True)
                else:
                    # 受信トピックのoutputトピック＋pub時間が、他のトピックのinputトピック＋sub時間と一致
                    self_pub_topic_nt = msg.output_info.topic_name
                    self_pub_has_header_stamp = msg.output_info.has_header_stamp
                    self_pub_stamp = msg.output_info.header_stamp
                    self_pub_time = msg.output_info.pub_time_steady
                    #for w in tinfo.msg.input_infos:
                    w = [mi for mi in tinfo.msg.input_infos if mi.topic_name == self_pub_topic_nt]
                    if w == None or w == []:
                        continue
                    # self-pub_time search other topic's sub_time
                    w = w[0]
                    try:
                        oth_sub_topic_nt = w.topic_name
                        oth_sub_has_stamp = w.has_header_stamp
                        oth_sub_stamp = w.header_stamp
                        oth_sub_time = w.sub_time_steady
                    except Exception as e:
                        print(f"{location()}: [Exception] {e}", flush=True)
                    try:
                        #DP(f"{location()}: pre {oth_sub_topic_nt=} : {self_pub_topic_nt=} {oth_sub_stamp=} {self_pub_stamp=}")
                        #if oth_sub_topic_nt == self_pub_topic_nt:
                        #DP(f"{location()}: pre {oth_sub_topic_nt=} : {self_pub_topic_nt=} {oth_sub_stamp=} {self_pub_stamp=}")
                        if self_pub_has_header_stamp == True and oth_sub_has_stamp == True:
                            if oth_sub_stamp == self_pub_stamp:
                                # match topic
                                DP(f"{location()}: MATCH {oth_sub_topic_nt=}")
                                calc_statis_resp(active_path, tinfo)
                                pi, ti = get_path_list(active_path.active_path, self_pub_topic_nt)
                                if ti == None:
                                    PP(f"{location()}: ## get_path_list() error {self_pub_topic_nt}")
                                else:
                                    ti.resp_time = stamp_to_sec(oth_sub_time) - stamp_to_sec(self_pub_time)
                                    calc_statis_resp(active_path, ti)
                                if match_topic_in_path(node, msg, active_path, topic_name, pinfo, recv_time) != True:
                                    PP(f"{location()}: ## match_topic_in_path() error discard {topic_name}")
                                    return False
                                return True
                            #else:
                            #    continue
                        else:
                            pi, ti = get_path_list(active_path.active_path, self_pub_topic_nt)
                            if ti == None:
                                PP(f"{location()}: ## get_path_list() error {self_pub_topic_nt}")
                            else:
                                ti.resp_time = stamp_to_sec(oth_sub_time) - stamp_to_sec(self_pub_time)
                                #PP(f"{location()}: {ti.resp_time=} {ti.name}")
                                calc_statis_resp(active_path, ti)
                            if match_topic_in_path(node, msg, active_path, topic_name, pinfo, recv_time) != True:
                                PP(f"{location()}: ## match_topic_in_path() error discard {topic_name}")
                                return False
                            return True
                    except Exception as e:
                        print(f"{location()}: [Exception] {e}", flush=True)
            else:
                # not received
                continue
    return False

def path_complete_check_and_proc(node, active_path, pinfo, recv_time, concurrent):
    #print(f"{location()}: +++  {[tinfo for tinfo in pinfo.topic_dict.values() if tinfo.status == ST_RECVD]=}")
    if len(pinfo.topic_dict) == len([tinfo for tinfo in pinfo.topic_dict.values() if tinfo.status == ST_RECVD]):
        try:
            deadline_timer_cancel(pinfo)
            pinfo.status = ST_COMPLETED
            if recv_time == 0.0:
                pinfo.end = pinfo.start
            else:
                pinfo.end = recv_time
            set_statis_path(PATH_COMPLETED, active_path, pinfo)
            pub_complete_topic(node, pinfo, concurrent)
            return True
        except Exception as e:
            print(f"{location()}: [Exception] {e}", flush=True)
    return False


def deadline_timer_start(pinfo):
    pinfo.timer_counter = pinfo.deadline_timer
    #DP(f"{location()}: #### {pinfo=}")
    return

def deadline_timer_cancel(pinfo):
    pinfo.timer_counter = 0.0
    #DP(f"{location()}: #### {pinfo=}")
    return

def pub_complete_topic(node, pinfo, concurrent):
    global g_complete_count
    global g_pub_disp
    if g_pub_disp == True:
        PP(f">>> PATH COMPLETED({g_complete_count}): [{pinfo.path_name} ({concurrent})] {pinfo.end - pinfo.start:.4f}")
        sys.stdout.flush()
    g_complete_count += 1
        
def pub_deadline_miss_topic(node, active_path, pinfo, concurrent):
    global g_deadline_miss_count
    global g_pub_disp
    ok_topics = []
    ng_topics = []
    ok_topics.append({pinfo.top_topic.name: [pinfo.top_topic.recv_time - pinfo.start, pinfo.top_topic.msg.output_info.header_stamp]})
    for tn, tinfo in pinfo.topic_dict.items():
        if tinfo.status == ST_RECVD:
            ok_topics.append({tn: [tinfo.recv_time - pinfo.start, tinfo.msg.output_info.header_stamp]})
        else:
            set_statis_missing_topic(active_path, tn)
            ng_topics.append(tn)
    if g_pub_disp == True:
        print(f">>> DEADLINE MISS({g_deadline_miss_count}): [{pinfo.path_name} ({concurrent})] {pinfo.end - pinfo.start:.4f}")
        pprint(f"receive topics: {ok_topics}")
        pprint(f"mising topics: {ng_topics}")
        sys.stdout.flush()
    # publish
    m = TildeAggregatorDeadlineMiss()
    m.path_name = pinfo.path_name
    m.deadline_timer = pinfo.deadline_timer
    m.exceed = pinfo.deadline_timer - pinfo.timer_counter
    m.cur_instance_count = concurrent
    ## m.pending_msg_count = len(node.active_path_list.pending_msg)
    m.pending_msg_count = len(active_path.pending_msg)
    for w in ok_topics:
        for tn, ti in w.items():
            mok = TildeAggregatorTopic()
            mok.topic_name = tn
            mok.pub_stamp = ti[1]
            mok.response_time = ti[0]
            m.completed_topics.append(mok)
    m.missing_topics += ng_topics
    m.header.stamp = node.get_clock().now().to_msg()
    node.deadline_miss_pub.publish(m)
    g_deadline_miss_count += 1

def deadline_check_and_proc(node, active_path, cur_time, ctick):
    for pinfo in active_path.active_path: 
        if pinfo.status != ST_START or pinfo.timer_counter == 0.0:
            continue
        pinfo.timer_counter -= ctick
        if pinfo.timer_counter <= 0.0:
            try:
                # deadline timeout occured
                pinfo.end = cur_time
                pinfo.status = ST_DEADLINE_MISS
                set_statis_path(PATH_DEADLINE_MISS, active_path, pinfo)
                concurrent = len([w for w in active_path.active_path if w.path_name == pinfo.path_name])
                pub_deadline_miss_topic(node, active_path, pinfo, concurrent)
                active_path.active_path.remove(pinfo)
            except Exception as e:
                print(f"{location()}: [Exception] {e}", flush=True)

def pub_disp_ctrl(ope):
    global g_pub_disp
    g_pub_disp = ope

