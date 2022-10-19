#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import sys, os, time
import tarfile
import copy

from dataclasses import dataclass, field, asdict
from typing import List, Literal, Tuple, Dict, ClassVar

from .tilde_agg_com import *

PATH_COMPLETED = 'path_completed'
PATH_DEADLINE_MISS = 'path_deadline_miss'

@dataclass(frozen=False)
class StatTopic:
    topic_name: str = ''
    hz_min: float = 0.0
    hz_max: float = 0.0
    hz_ave: float = 0.0
    hz_acum: float = 0.0
    acum_cnt: int = 0
    prev_time: float = 0.0

@dataclass(frozen=False)
class StatPath:
    path_name: str = ''
    completed_cnt: int = 0
    completed_min: float = 0.0
    completed_max: float = 0.0
    completed_ave: float = 0.0
    completed_acum: float = 0.0
    acum_cnt: int = 0
    deadline_miss_cnt: int = 0
    deadline_miss_min: float = 0.0
    deadline_miss_max: float = 0.0
    deadline_miss_ave: float = 0.0
    deadline_miss_acum: float = 0.0
    deadline_miss_acum_cnt: int = 0
    stat_topic: List[StatTopic] = field(default_factory=list)

@dataclass(frozen=False)
class StatList:
    statis_list: List[StatPath] = field(default_factory=list)

g_statis_list = StatList()

def make_statis_path(path_name):
    pstat = StatPath()
    pstat.path_name = path_name
    g_statis_list.statis_list.append(pstat)
    return pstat

def make_statis_topic(pstat, topic_name):
    tstat = StatTopic()
    tstat.topic_name = topic_name
    pstat.stat_topic.append(tstat)

def calc_statis_hz(path_name, topic_name, msg):
    for ps in g_statis_list.statis_list:
        if ps.path_name == path_name:
            for ts in ps.stat_topic:
                if ts.topic_name == topic_name:
                    cur_time = stamp_to_sec(msg.output_info.pub_time_steady)
                    if cur_time < ts.prev_time:
                        DP(f"{location()}: ## timing reverse {cur_time=} {ts.prev_time=}")
                    hz = cur_time - ts.prev_time
                    if ts.prev_time != 0.0:
                        ts.hz_min = min(hz, ts.hz_min)
                        ts.hz_max = max(hz, ts.hz_max)
                        ts.acum_cnt += 1
                        ts.hz_acum += hz
                    else:
                        ts.hz_min = hz
                        ts.hz_max = 0.0
                    ts.prev_time = cur_time
                    ts.hz_ave = ts.hz_acum / max(1, ts.acum_cnt)
                        
def set_statis_path(id, pinfo):
    diff_time = pinfo.end - pinfo.start
    for sl in  g_statis_list.statis_list:
        if sl.path_name == pinfo.path_name:
            if id == PATH_COMPLETED:
                # complete time
                if sl.acum_cnt == 0:
                    sl.completed_min = diff_time
                    sl.completed_max = 0.0
                else:
                    sl.completed_min = min(diff_time, sl.completed_min)
                    sl.completed_max = max(diff_time, sl.completed_max)
                sl.completed_acum += diff_time
                sl.acum_cnt += 1
                sl.completed_cnt += 1
                sl.completed_ave = sl.completed_acum / max(1, sl.acum_cnt)
            else:
                # complete time
                if sl.deadline_miss_acum_cnt == 0:
                    sl.deadline_miss_min = diff_time
                    sl.deadline_miss_max = diff_time
                else:
                    sl.deadline_miss_min = min(diff_time, sl.deadline_miss_min)
                    sl.deadline_miss_max = max(diff_time, sl.deadline_miss_max)
                sl.deadline_miss_acum += diff_time
                sl.deadline_miss_acum_cnt += 1
                sl.deadline_miss_cnt += 1
                sl.deadline_miss_ave = sl.deadline_miss_acum / max(1, sl.deadline_miss_acum_cnt)

###
def com_show_statis(command):
    print_topic = []
    print(f"\n\n--- TILDE aggregator statistics ---\n")
    for ps in g_statis_list.statis_list:
        pn = ps.path_name
        completed_count = ps.completed_cnt
        deadline_miss_count = ps.deadline_miss_cnt
        completed_min = ps.completed_min
        completed_max = ps.completed_max
        completed_ave = ps.completed_ave
        dead_min = ps.deadline_miss_min
        dead_max = ps.deadline_miss_max
        dead_ave = ps.deadline_miss_ave
        print(f"[{pn}] : {completed_count=} {deadline_miss_count=}\n \
              {completed_min=:.4f}/{completed_ave=:.4f}/{completed_max=:.4f} (sec)\n \
              {dead_min=:.4f}/{dead_ave=:.4f}/{dead_max=:.4f} (sec)\n \
             ")
        if pn not in print_topic:
            for ts in ps.stat_topic:
                tn = ts.topic_name            
                recv_count = ts.acum_cnt
                min = ts.hz_min
                max = ts.hz_max
                ave = ts.hz_ave
                print(f" -[{tn}] : {min=:.4f}/{ave=:.4f}/{max=:.4f} (sec) {recv_count=}")
                print_topic.append(pn)
        print(f"-----------------------------------------")
    if command == COMMAND_SHOW_CLR_STATIS:
        print(f"### statistics clear not yet implemented")
    print(f"(END)--------------------------------------\n", flush=True)
