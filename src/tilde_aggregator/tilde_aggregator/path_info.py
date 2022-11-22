#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-



from collections import OrderedDict

from dataclasses import dataclass, field, asdict
from socket import TIPC_ZONE_SCOPE
from ssl import SSL_ERROR_WANT_CONNECT
from typing import List, Literal, Tuple, Dict, ClassVar

from .tilde_agg_com import *
from .statis import *
from tilde_msg.msg import *

ST_NONE = 'ST_NONE'   # not set
ST_INIT = 'ST_INIT'   # initialized
# topic state
ST_WAIT = 'ST_WAIT'   # wait mtt topic
ST_RECVD = 'ST_RECVD' # mtt topic received
# active path state
ST_START = 'ST_START' # path start
ST_COMPLETED = 'ST_COMPLETED' # path completed
ST_DEADLINE_MISS = 'ST_DEADLINE_MISS' # path deadline miss occured

g_not_ready_topics = []

@dataclass(frozen=False)
class TopicInfo:
    name: str = ''
    order: int = 0
    timer: float = 0.0
    status: str = ST_NONE
    msg: MessageTrackingTag = () # save msg
    recv_time: float = 0.0
    resp_time: float = 0.0

@dataclass(frozen=False)
class TildePathManageTable:
    path_name: str = ''
    deadline_timer: float = 0.0
    timer_counter: float = 0.0
    status: str = ST_NONE
    start: float = 0.0
    end: float = 0.0
    # path=topic order
    top_topic: TopicInfo = ()
    topic_dict: Dict[str, TopicInfo] = field(default_factory=dict) # {'topic_name': TopicInfo}

@dataclass(frozen=False)
class ActivePathlist:
    path_name: str = ''
    active_path: List[TildePathManageTable] = field(default_factory=list)
    pending_msg: List[Dict[str, TopicInfo]] = field(default_factory=list)
    path_statis: StatPath = ()

#
def print_path_list(pl, concurrent):
    pn = pl.path_name
    ptm = pl.timer_counter
    pst = pl.status
    start = pl.start
    end = pl.end
    ttn = pl.top_topic.name
    ttst = pl.top_topic.status
    recv = pl.top_topic.recv_time
    resp = pl.top_topic.resp_time
    print(f"[{pn}] ({concurrent}) : remain={ptm:.4f}(sec) {pst} {start=:.4f}-{end=:.4f}")
    print(f" - TOP: {ttn} {ttst} {resp=:.4f} {recv=}")
    for k, v in pl.topic_dict.items():
        if v.status == ST_RECVD:
            print(f"   --- {k}: {v.status} resp={v.resp_time:.4f} recv={v.recv_time:.4f}")
        else:
            print(f"   --- {k}: {v.status} resp=(---) recv=(---)")
    print(f"-----------------------------------------")

###
def show_inner_state(command, static_plist, target_topics, active_path):
    print(f"\n\n--- TILDE aggregator inner state ---\n")
    for pl in static_plist:
        pn = pl.path_name
        pdead = pl.deadline_timer
        ttn = pl.top_topic.name
        print(f"[{pn}] : {pdead:.4f}(sec)\n - TOP: {ttn}")
        for k, v in pl.topic_dict.items():
            print(f" --- {k}")
    print(f"-----------------------------------------")
    print(f"current available topics ({len(target_topics)}) pending message({len(active_path.pending_msg)})")
    pprint(f"{target_topics}")
    pprint(f"current not ready topics ({len(g_not_ready_topics)})  {g_not_ready_topics}")
    print(f"--- TILDE aggregator active path instance ({len(active_path.active_path)}) ---\n")
    for pinfo in active_path.active_path:
        concurrent = len(active_path)
        print_path_list(pinfo, concurrent)
    if command == COMMAND_SHOW_PENDING:
        mlen = len(active_path.pending_msg)
        print(f"--- TILDE aggregator bufferd messages ({mlen}) ---\n")
        for i, w in enumerate(active_path.pending_msg):
            for topic, msg in w.items():
                print(f"({i}/{mlen}) {topic}: {msg}")
                print(f"-----------------------------------------")
    print(f"(END)--------------------------------------\n", flush=True)

