#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import sys, os, time
import tarfile
import copy

from dataclasses import dataclass, field, asdict
from typing import List, Literal, Tuple, Dict, ClassVar

from std_msgs.msg import String
from tilde_aggregator_interfaces.msg import *

from .tilde_agg_com import *

PATH_COMPLETED = 'path_completed'
PATH_DEADLINE_MISS = 'path_deadline_miss'
MAX_INTERVAL = 3.0

TILDE_AGGREGATOR_STATIS = 'tilde_aggregator_statis'
@dataclass(frozen=False)
class StatTopic:
    topic_name: str = ''
    recv_count: int = 0
    hz_min: float = 0.0
    hz_max: float = 0.0
    hz_ave: float = 0.0
    hz_accum: float = 0.0
    accum_cnt: int = 0
    prev_time: float = 0.0
    outorder_cnt: int = 0
    resp_min: float = 0.0
    resp_max: float = 0.0
    resp_ave: float = 0.0
    resp_accum: float = 0.0
    resp_accum_cnt: int = 0
    dup_count: int = 0
    missing_count: int = 0

@dataclass(frozen=False)
class StatPath:
    path_name: str = ''
    completed_cnt: int = 0
    completed_min: float = 0.0
    completed_max: float = 0.0
    completed_ave: float = 0.0
    completed_accum: float = 0.0
    completed_accum_cnt: int = 0
    deadline_miss_cnt: int = 0
    deadline_miss_min: float = 0.0
    deadline_miss_max: float = 0.0
    deadline_miss_ave: float = 0.0
    deadline_miss_accum: float = 0.0
    deadline_miss_accum_cnt: int = 0
    statis_topic: List[StatTopic] = field(default_factory=list)

@dataclass(frozen=False)
class CbStatis:
    cb_name: str = ''
    cb_cnt: int = 0
    cb_prev: float = 0.0
    cb_min: float = 0.0
    cb_max: float = 0.0
    cb_ave: float = 0.0
    cb_accum: float = 0.0
    cb_accum_cnt: int = 0

@dataclass(frozen=False)
class CbStatisList:
    cb_statis_list: List[CbStatis] = field(default_factory=list)

##########
def make_statis_path(path_name):
    pstat = StatPath()
    pstat.path_name = path_name
    return pstat

def make_statis_topic(pstat, topic_name):
    tstat = StatTopic()
    tstat.topic_name = topic_name
    pstat.statis_topic.append(tstat)

def calc_statis_hz(active_path, topic_name, msg):
    ps = active_path.path_statis
    DP(f"{[ti for ti in ps.statis_topic if ti.topic_name == topic_name]=}")
    ts = [ti for ti in ps.statis_topic if ti.topic_name == topic_name][0]
    ts.recv_count += 1
    cur_time = stamp_to_sec(msg.output_info.pub_time_steady)
    if cur_time < ts.prev_time:
        ts.outorder_cnt += 1
        hz = ts.hz_min
    elif (cur_time - max(1, ts.prev_time)) >= MAX_INTERVAL:
        ts.accum_cnt = 0
        ts.hz_accum = 0.0
    else:
        hz = cur_time - ts.prev_time
    if ts.accum_cnt == 0:
        pass 
    elif ts.accum_cnt == 1:
        ts.hz_min = hz
        ts.hz_max = hz
        ts.hz_accum += hz
    else:
        ts.hz_min = min(hz, ts.hz_min)
        ts.hz_max = max(hz, ts.hz_max)
        ts.hz_accum += hz
    ts.accum_cnt += 1
    ts.prev_time = cur_time

def calc_statis_resp(active_path, tinfo):
    ps = active_path.path_statis
    ts = [ti for ti in ps.statis_topic if ti.topic_name == tinfo.name][0]
    res = tinfo.resp_time
    if res > 0.0:
        if ts.resp_max == 0.0:
            ts.resp_min = res
            ts.resp_max = res
        else:
            ts.resp_min = min(res, ts.resp_min)
            ts.resp_max = max(res, ts.resp_max)
        ts.resp_accum += res
        ts.resp_accum_cnt += 1

def set_statis_dup_topic(active_path, topic_name):
    ps = active_path.path_statis
    ts = [ti for ti in ps.statis_topic if topic_name in ti.topic_name][0]
    ts.dup_count += 1

def set_statis_missing_topic(active_path, topic_name):
    ps = active_path.path_statis
    ts = [ti for ti in ps.statis_topic if topic_name in ti.topic_name][0]
    ts.missing_count += 1
                        
def set_statis_path(id, active_path, pinfo):
    sl = active_path.path_statis
    if id == PATH_COMPLETED:
        # complete time
        diff_time = max(0.0, pinfo.end - pinfo.start)
        if diff_time >= MAX_INTERVAL:
            diff_time = max(0.0, sl.completed_max)
        if sl.completed_accum_cnt == 0:
            sl.completed_min = diff_time
            sl.completed_max = 0.0
        else:
            sl.completed_min = min(diff_time, sl.completed_min)
            sl.completed_max = max(diff_time, sl.completed_max)
        sl.completed_accum += diff_time
        sl.completed_accum_cnt += 1
        sl.completed_cnt += 1
    else:
        # deadline miss time
        timeout = pinfo.deadline_timer - pinfo.timer_counter
        if sl.deadline_miss_accum_cnt == 0:
            sl.deadline_miss_min = timeout
            sl.deadline_miss_max = timeout
        else:
            sl.deadline_miss_min = min(timeout, sl.deadline_miss_min)
        if (timeout - sl.deadline_miss_max) >= MAX_INTERVAL:
            pass
        else:
            sl.deadline_miss_max = max(timeout, sl.deadline_miss_max)
        sl.deadline_miss_accum_cnt += 1
        sl.deadline_miss_accum += timeout
        sl.deadline_miss_cnt += 1

###
def show_statis(command, node):
    print_topic = []
    if command == COMMAND_SHOW_STATIS:
        print(f"\n\n--- TILDE aggregator statistics ---\n")
    else:
        # publish
        m = TildeAggregatorStatistics()
    for active_path in node.active_path_list:
        ps = active_path.path_statis
        if command != COMMAND_SHOW_STATIS:
            p = TildeAggregatorPathStatistics()
        pn = ps.path_name
        pending = len(active_path.pending_msg)
        completed_count = ps.completed_cnt
        deadline_miss_count = ps.deadline_miss_cnt
        completed_min = ps.completed_min
        completed_max = ps.completed_max
        completed_ave = ps.completed_ave = ps.completed_accum / max(1, ps.completed_accum_cnt)
        deadline_miss_min = ps.deadline_miss_min
        deadline_miss_max = ps.deadline_miss_max
        deadline_miss_ave = ps.deadline_miss_ave = ps.deadline_miss_accum / max(1, ps.deadline_miss_accum_cnt)
        cur_instance_count = len(active_path.active_path)
        if command == COMMAND_SHOW_STATIS:
            print(f"[{pn}] ({cur_instance_count}) {pending=} : {completed_count=} {deadline_miss_count=}\n \
                {completed_min=:.4f}/{completed_ave=:.4f}/{completed_max=:.4f} (sec)\n \
                {deadline_miss_min=:.4f}/{deadline_miss_ave=:.4f}/{deadline_miss_max=:.4f} (sec)\n \
                 ")
        else:
            p.path_name = pn
            p.completed_count = completed_count
            p.completed_time_min = completed_min
            p.completed_time_max = completed_max
            p.completed_time_ave = completed_ave
            p.deadline_miss_count = deadline_miss_count
            p.cur_instance_count = cur_instance_count
            p.pending_msg_count = pending
            m.path_statis.append(p)
        if pn not in print_topic:
            for ts in ps.statis_topic:
                tn = ts.topic_name
                recv_count = ts.recv_count
                ts.hz_ave = ts.hz_accum / max(1, ts.accum_cnt)
                if command == COMMAND_SHOW_STATIS:
                    print(f" -[{tn}]\n        - : HZ min={ts.hz_min:.4f}/ave={ts.hz_ave:.4f}/max={ts.hz_max:.4f} (sec) {recv_count=}")
                ts.resp_ave = ts.resp_accum / max(1, ts.resp_accum_cnt)
                if command == COMMAND_SHOW_STATIS:
                    print(f"        - : Response min={ts.resp_min:.4f}/ave={ts.resp_ave:.4f}/max={ts.resp_max:.4f} (sec)")
                dup_count = ts.dup_count
                out_of_order = ts.outorder_cnt
                missing_count = ts.missing_count
                if command == COMMAND_SHOW_STATIS:
                    print(f"        - : Notice {dup_count=} {out_of_order=} {missing_count=}")
                else:
                    t = TildeAggregatorTopicStatistics()
                    t.topic_name = tn
                    t.recv_count = recv_count
                    t.hz_min = ts.hz_min
                    t.hz_max = ts.hz_max
                    t.hz_ave = ts.hz_ave
                    t.resp_min = ts.resp_min
                    t.resp_max = ts.resp_max
                    t.resp_ave = ts.resp_ave
                    t.dup_count = dup_count
                    t.out_of_order_count = out_of_order
                    t.missing_count_in_active_path = missing_count
                    p.topic_statis.append(t)
                print_topic.append(pn)
        if command == COMMAND_SHOW_STATIS:
            print(f"-----------------------------------------")
    if command == COMMAND_SHOW_STATIS:
        print(f"(END)--------------------------------------\n", flush=True)
    else:
        m.header.stamp = node.get_clock().now().to_msg()
        node.tilde_aggregator_statis_pub.publish(m)

def clr_statis(command, node):
    for active_path in node.active_path_list:
        ps = active_path.path_statis
        ps.completed_cnt = ps.completed_accum_cnt = ps.deadline_miss_cnt = ps.deadline_miss_accum_cnt = 0
        ps.completed_min = ps.completed_max = ps.completed_ave = ps.completed_accum = 0.0
        ps.deadline_miss_min = ps.deadline_miss_max = ps.deadline_miss_ave = ps.deadline_miss_accum = 0.0
        for ts in ps.statis_topic:
            ts.hz_min = ts.hz_max = ts.hz_ave = ts.hz_accum =0.0
            ts.resp_min = ts.resp_max = ts.resp_ave = ts.resp_accum =0.0
            ts.recv_count = ts.accum_cnt = ts.outorder_cnt = ts.resp_accum_cnt = ts.dup_count = ts.missing_count = 0
            ts.prev_time = 0.0

    for cs in node.cb_statis.cb_statis_list:
        cs.cb_min = 0.0
        cs.cb_max = 0.0
        cs.cb_ave = 0.0
        cs.cb_prev = 0.0
        cs.cb_accum = 0.0
        cs.cb_accum_cnt = 0
        cs.cb_cnt = 0
    #node.cb_statis.cb_statis_list.clear()

###
def cb_statis_init(node):
    cb_funcs = []
    cb_funcs.append('reconfig_callback')
    cb_funcs.append('tick_callback')
    cb_funcs.append('top_topic_callback')
    cb_funcs.append('topic_callback')
    cb_funcs.append('original_command_callback')
    #cb_funcs.append('org_topic_callback')
    #cb_funcs.append('default_callback')
    #cb_funcs.append('default_timer_callback')
    for cb_name in cb_funcs:
        cb = CbStatis()
        cb.cb_name = cb_name
        node.cb_statis.cb_statis_list.append(cb)

def cb_statis_enter(node, func):
    cs = [w for w in node.cb_statis.cb_statis_list if w.cb_name == func][0]
    cs.cb_prev = time.perf_counter()

def cb_statis_exit(node, func):
    cs = [w for w in node.cb_statis.cb_statis_list if w.cb_name == func][0]
    elapse = time.perf_counter() - cs.cb_prev
    if elapse < 0:
        return
    if cs.cb_min == 0.0:
        cs.cb_min = elapse
    else:
        cs.cb_min = min(cs.cb_min, elapse)
    if cs.cb_max == 0.0:
        cs.cb_max = elapse
    else:
        cs.cb_max = max(cs.cb_max, elapse)
    cs.cb_accum += elapse
    cs.cb_accum_cnt += 1
    cs.cb_ave = cs.cb_accum / max(1, cs.cb_accum_cnt)
    cs.cb_cnt += 1

def show_cb_statis(command, node):
    print(f"\n\n--- Callback latency statistics ---\n")
    for cs in node.cb_statis.cb_statis_list:
        cbmin = cs.cb_min
        cbmax = cs.cb_max
        cbave = cs.cb_accum / max(1, cs.cb_accum_cnt)
        cnt = cs.cb_cnt
        print(f"{cs.cb_name} ({cnt}) --- min={to_usec(cbmin):6.4f} ave={to_usec(cbave):6.4f} max={to_usec(cbmax):6.4f} (usec)")
    print(f"(END)--------------------------------------\n", flush=True)

def to_usec(ns):
    return ns * 1000
