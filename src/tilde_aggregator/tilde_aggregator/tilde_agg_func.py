#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
from re import I
import sys, os, time
import tarfile
import oyaml as yaml
import copy
import json

from rclpy.clock import Clock, ClockType
from rclpy.time import Time

from .tilde_agg_com import *
from .path_info import *
from .statis import *

YAML_PARAMS = 'params'
MTT_TOPIC_POSTFIX = 'message_tracking_tag'


def make_path_list_from_yaml(fn, plist):
    try:
        with open(fn, 'r') as f:
            yaml_data = yaml.safe_load(f)
    except Exception as e:
        print(location())
        print(f"##{e}: No such file or yaml load error {fn}", file=sys.stderr)
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
            pstat = make_statis_path(k1)

            for i, k2 in enumerate([key for key in path_info['topic_list']]):
                mtt_topic = k2 + '/' + MTT_TOPIC_POSTFIX
                make_statis_topic(pstat, mtt_topic)
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
            plist.append(pinfo)

def active_set_path_list(path_list, topics):
    #DP(f"{location()}: {path_list=}")
    target_list = []
    for pinfo in path_list:
        #DP(f"{location()}: {pinfo=}")
        complete = True
        if pinfo.top_topic.name in topics:
            #DP(f"{location()}: {pinfo.top_topic.name=}")
            #if pinfo.top_topic.status == ST_INIT:
            #    pinfo.top_topic.status = ST_WAIT
            #    target_list.append(pinfo.top_topic.name)
            target_list.append(pinfo.top_topic.name)
        else:
            complete = False
        for name, tinfo in pinfo.topic_dict.items():
            if name in topics:
                #DP(f"{location()}: #### {name}")
                #if tinfo.status == ST_INIT:
                #    tinfo.status = ST_WAIT
                #    target_list.append(name)
                target_list.append(name)
            else:
                complete = False
        if complete == True and pinfo.status == ST_INIT:
            pinfo.status = ST_WAIT
    #DP(f"{location()}: {target_list=}")
    return target_list

def get_path_list(path_list, mtt_topic):
    for pinfo in path_list:
        if mtt_topic in pinfo.top_topic.name:
            return pinfo, pinfo.top_topic
        for name, tinfo in pinfo.topic_dict.items():
            #DP(f"{location()}: #### {name}: {tinfo}")
            if mtt_topic in name:
                return pinfo, tinfo
    print(f"{location()}: ## not registered mtt-topic {mtt_topic}")
    return None, None


#
def new_path_list_add(msg, topic_name, pinfo, tinfo, sub_time):
    np = copy.deepcopy(pinfo)
    np.status = ST_START
    np.start = sub_time
    np.top_topic.status = ST_RECVD
    np.top_topic.msg = msg
    np.top_topic.recv_time = sub_time
    for topic, v in np.topic_dict.items():
        #DP(f"{location()}: #### {topic}: {v}")
        v.status = ST_WAIT
    #DP(f"{location()}: #### {np=}")
    return np

def register_topic_info(msg, topic_name, pinfo, sub_time):
    for name, tinfo in pinfo.topic_dict.items():
        if name in topic_name and tinfo.status <= ST_WAIT:
            tinfo.msg = msg
            tinfo.recv_time = sub_time
            tinfo.status = ST_RECVD
            DP(f"--- General topic registered {topic_name} in {pinfo.path_name} ---")
            return True
    else:
        DP(f"{location()}: #### ERROR {name}: {tinfo}")
        return False

def top_topic_receive_main(msg, active_path_list, topic_name, pinfo, tinfo, sub_time):
    deadline_timer_start(pinfo)
    new_pinfo = new_path_list_add(msg, topic_name, pinfo, tinfo, sub_time)
    active_path_list.active_path.append(new_pinfo)
    DP(f"--- TOP topic registered {topic_name} in {pinfo.path_name} ---")

def match_topic_in_path(msg, active_path_list, topic_name, pinfo, time):
    if register_topic_info(msg, topic_name, pinfo, time) == False:
        return False
    concurrent = len([w for w in active_path_list.active_path if w.path_name == pinfo.path_name])
    if path_comlete_check_and_proc(pinfo, time, concurrent) == True:
        active_path_list.active_path.remove(pinfo)
    return True

def topic_receive_main(msg, active_path_list, topic_name, init_pinfo, init_tinfo, time):
    """
    - 受信トピックがどのパス(複数インスタンスあり)に含まれているか探索
      どのインスタンスかは分からないので、一つづつ確認することになる
      (前提：異なるパスに同じトピックは含まれないこと)
      - 該当トピックが受信済なら該当パスインスタンスはスキップ
      - 基本的に、該当トピックのpub時間がtopトピックのpub時間以降ならskip

    以下は該当パス(複数インスタンスあり)内での処理
    - 受信トピックのinputトピック(複数あり)＋sub時間が、topトピック名＋pub時間以上
    - 受信トピックのoutputトピック＋pub時間が、他のトピックのinputトピック＋sub時間以下
    - 受信トピックのinputトピック(複数あり)＋sub時間が、他のoutputトピック＋pub時間以上
    """
    if len(active_path_list.active_path) == 0:
        new_tp = copy.deepcopy(init_tinfo)
        new_tp.msg = msg
        new_tp.recv_time = time
        active_path_list.pending_msg.append({topic_name: new_tp})
        return False

    for pinfo in [pinfo for pinfo in active_path_list.active_path if topic_name in pinfo.topic_dict.keys()]:
        #DP(f"{location()}: {topic_name=} {pinfo=}")
        #DP(f"====================================")
        #if pinfo.top_topic.msg.output_info.pub_time > msg.output_info.pub_time:
        #    # pub time over
        #    continue
        self_pub_topic = msg.output_info.topic_name
        self_has_header_stamp = msg.output_info.has_header_stamp
        self_pub_stamp = msg.output_info.header_stamp
        top_topic = pinfo.top_topic.msg.output_info.topic_name
        top_has_stamp = pinfo.top_topic.msg.output_info.has_header_stamp
        top_stamp = pinfo.top_topic.msg.output_info.header_stamp
        for w in msg.input_infos:
            # 受信トピックのinputトピック(複数あり)＋sub時間が、topトピック名＋pub時間以下、かつhas_header_stampが一致
            try:
                self_sub_topic = w.topic_name
                self_sub_has_stamp = w.has_header_stamp
                self_sub_stamp = w.header_stamp
            except Exception as e:
                print(f"{location()}: [Except] {e}", flush=True)

            DP(f"{location()}: pre {self_sub_topic=} {top_topic=} {self_sub_stamp=} {top_stamp=}")
            if self_sub_topic in top_topic:
                #DP(f"{location()}: pre {self_sub_topic=} {top_topic=} {self_sub_stamp=} {top_stamp=}")
                if self_sub_has_stamp == True and top_has_stamp == True:
                    if self_sub_stamp == top_stamp:
                        # match
                        DP(f"{location()}: MATCH {self_sub_stamp=}")
                        return match_topic_in_path(msg, active_path_list, topic_name, pinfo, time)
                    else:
                        continue
                else:
                    return match_topic_in_path(msg, active_path_list, topic_name, pinfo, time)

        for name, tinfo in pinfo.topic_dict.items():
            if topic_name in name and tinfo.status == ST_RECVD:
                # name match but already received -> next path list element
                break
            if tinfo.status == ST_RECVD:
                #DP(f"{location()}: #### {name}")
                # 受信トピックのoutputトピック＋pub時間が、他のトピックのinputトピック＋sub時間と一致
                for w in tinfo.msg.input_infos:
                    # self-pub_time search other topic's sub_time
                    try:
                        oth_sub_topic = w.topic_name
                        oth_sub_has_stamp = w.has_header_stamp
                        oth_sub_stamp = w.header_stamp
                    except Exception as e:
                        print(f"{location()}: [Except] {e}", flush=True)

                    try:
                        DP(f"{location()}: pre {oth_sub_topic=} : {self_pub_topic=} {oth_sub_stamp=} {self_pub_stamp=}")
                        if oth_sub_topic in self_pub_topic:
                            DP(f"{location()}: pre {oth_sub_topic=} : {self_pub_topic=} {oth_sub_stamp=} {self_pub_stamp=}")
                            if self_has_header_stamp == True and oth_sub_has_stamp == True:
                                if oth_sub_stamp == self_pub_stamp:
                                    # match topic
                                    DP(f"{location()}: MATCH {oth_sub_topic=}")
                                    return match_topic_in_path(msg, active_path_list, topic_name, pinfo, time)
                                else:
                                    continue
                            else:
                                return match_topic_in_path(msg, active_path_list, topic_name, pinfo, time)
                    except Exception as e:
                        print(f"{location()}: [Except] {e}", flush=True)

                    # self-some-topic sub_time search other topic's pub_time
                    # pinfo内のtinfoのpub時間と自身のmsgのsubしトピック名とsub時間が同じか検査する
                    for w in msg.input_infos:
                        #DP(f"{location()}: {name=}")
                        try:
                            self_sub_topic = w.topic_name
                            self_sub_has_stamp = w.has_header_stamp
                            self_sub_stamp = w.header_stamp
                            #DP(f"{location()}: #### {self_prev_topic}: {self_sub_time}")
                        except Exception as e:
                            print(f"{location()}: [Excepttion] {e}", flush=True)

                        try:
                            oth_pub_topic = tinfo.msg.output_info.topic_name
                            oth_pub_has_stamp = w.has_header_stamp
                            oth_pub_stamp = w.header_stamp
                        except Exception as e:
                            print(f"{location()}: [Excepttion] {e}", flush=True)

                        try:
                            DP(f"{location()}: pre {oth_pub_topic=} {self_sub_topic=} {oth_pub_stamp=} {self_sub_stamp=}")
                            if oth_pub_topic in self_sub_topic:
                                DP(f"{location()}: pre {oth_pub_topic=} {self_sub_topic=} {oth_pub_stamp=} {self_sub_stamp=}")
                                if oth_pub_has_stamp == True and self_sub_has_stamp == True:
                                    if oth_pub_stamp == self_sub_stamp:
                                        # match topic
                                        DP(f"{location()}: MATCH {oth_pub_topic=}")
                                        return match_topic_in_path(msg, active_path_list, topic_name, pinfo, time)
                                    else:
                                        continue
                                else:
                                    return match_topic_in_path(msg, active_path_list, topic_name, pinfo, time)
                        except Exception as e:
                            print(f"{location()}: [Excepttion] {e}", flush=True)

            else:
                # not received
                continue
    new_tp = copy.deepcopy(init_tinfo)
    new_tp.msg = msg
    new_tp.recv_time = time
    active_path_list.pending_msg.append({topic_name: new_tp})
    #DP(f"{location()}: #### {active_path_list=}")
    return False

def path_comlete_check_and_proc(pinfo, time, concurrent):
    if len(pinfo.topic_dict.items()) == len([tinfo for tinfo in pinfo.topic_dict.values() if tinfo.status == ST_RECVD]):
        deadline_timer_cancel(pinfo)
        pinfo.status = ST_COMPLETED
        pinfo.end = time
        set_statis_path(PATH_COMPLETED, pinfo)
        pub_complete_topic(pinfo, concurrent)
        return True
    return False


def deadline_timer_start(pinfo):
    pinfo.timer_counter = pinfo.deadline_timer
    #DP(f"{location()}: #### {pinfo=}")
    return

def deadline_timer_cancel(pinfo):
    pinfo.timer_counter = 0.0
    #DP(f"{location()}: #### {pinfo=}")
    return

def pub_complete_topic(pinfo, concurrent):
    DP(f"{location()}: !!! [{pinfo.path_name} ({concurrent})] COMPLETED {pinfo.end - pinfo.start:.4f}")
    return

def pub_deadline_miss_topic(pinfo, concurrent):
    DP(f"{location()}: !!! [{pinfo.path_name} ({concurrent})] DEADLINE_MISS {pinfo.end - pinfo.start:.4f}")
    return

def deadline_check_and_proc(active_path_list, cur_time, tick):
    for pinfo in active_path_list.active_path:
        if pinfo.status != ST_START or pinfo.timer_counter == 0.0:
            continue
        pinfo.timer_counter -= tick
        if pinfo.timer_counter <= 0.0:
            try:
                # deadline timeout occured
                pinfo.end = cur_time
                pinfo.status = ST_DEADLINE_MISS
                set_statis_path(PATH_DEADLINE_MISS, pinfo)
            except Exception as e:
                print(f"{location()}: [Exception] {e}", flush=True)
            try:
                concurrent = len([w for w in active_path_list.active_path if w.path_name == pinfo.path_name])
                pub_deadline_miss_topic(pinfo, concurrent)
            except Exception as e:
                print(f"{location()}: [Exception] {e}", flush=True)
            try:
                active_path_list.active_path.remove(pinfo)
            except Exception as e:
                print(f"{location()}: [Exception] {e}", flush=True)
