import collections

def time2str(t):
    """
    t: builtin_interfaces.msg.Time
    """
    return f"{t.sec}.{t.nanosec:09d}"

class TopicInfo(object):
    def __init__(self, topic, pubsub_stamp, pubsub_stamp_steady,
                 has_stamp, stamp):
        """
        topic: target topic [string]
        pubsub_stamp:
          when publish or subscription callback is called
          [builtin_interfaces.msg.Time]
        pubsub_stamp_steady:
          same with above but in steady time
          [builtin_interfaces.mg.Time]
        has_stamp: whether main topic has header.stamp or not [bool]
        stamp: header.stamp [builtin_interfaces.msg.Time]
        """
        self.topic = topic
        self.pubsub_stamp = pubsub_stamp
        self.pubsub_stamp_steady = pubsub_stamp_steady
        self.has_stamp = has_stamp
        self.stamp = stamp

    def __str__(self):
        stamp_s = time2str(self.stamp) if self.has_stamp else "NA"
        return f"TopicInfo(topic={self.topic}, stamp={stamp_s})"

class PubInfo(object):
    """
    PubInfo
      - out_info = TopicInfo
      - in_infos = {topic_name => list of TopicInfo}
    """
    def __init__(self, out_topic, pub_time, pub_time_steady, out_stamp):
        """
        out_topic: topic name
        pub_time: when publish main topic [builtin_interfaces.msg.Time]
        out_stamp: main topic header.stamp [builtin_interfaces.msg.Time]
        """
        self.out_info = TopicInfo(out_topic, pub_time, pub_time_steady,
                                  True, out_stamp)
        # topic name vs TopicInfo
        self.in_infos = {}

    def add_input_info(self, in_topic, sub_stamp, sub_stamp_steady,
                       has_stamp, stamp):
        """
        in_topic: topic string
        has_stamp: bool
        stamp: header stamp [builtin_interfaces.msg.Time]
        """
        info = TopicInfo(in_topic, sub_stamp, sub_stamp_steady,
                         has_stamp, stamp)
        self.in_infos.setdefault(in_topic, []).append(info)

    def __str__(self):
        s = "PubInfo: \n"
        s += f"  out_info={self.out_info}\n"
        for _, ti in self.in_infos.items():
            s += f"  in_infos={ti}\n"
        return s

    @property
    def out_topic(self):
        return self.out_info.topic

class PubInfos(object):
    '''
    topic vs PubInfo

    We have double-key dictionary internally, i.e.
    we can get PubInfo by topic_vs_pubinfo[topic_name][stamp].
    '''
    def __init__(self):
        # {topic => {stamp_str => PubInfo}}
        self.topic_vs_pubinfos = {}

    def add(self, pubinfo):
        out_topic = pubinfo.out_info.topic
        out_stamp = time2str(pubinfo.out_info.stamp)
        if out_topic not in self.topic_vs_pubinfos.keys():
            self.topic_vs_pubinfos[out_topic] = {}
        infos = self.topic_vs_pubinfos[out_topic]

        if out_stamp not in infos.keys():
            infos[out_stamp] = {}

        infos[out_stamp] = pubinfo

    def erase_until(self, stamp):
        """
        erase added pubinfo where out_stamp < stamp
        stamp: rclpy.Time
        """
        thres_stamp = time2str(stamp)
        erases = {}
        for (topic, infos) in self.topic_vs_pubinfos.items():
            for stamp in infos.keys():
                if thres_stamp <= stamp:
                    continue
                erases.setdefault(topic, []).append(stamp)

        for (topic, stamps) in erases.items():
            for stamp in stamps:
                del self.topic_vs_pubinfos[topic][stamp]

    def topics(self):
        return list(self.topic_vs_pubinfos.keys())

    def stamps(self, topic):
        """
        return List[stamps]
        """
        if topic not in self.topic_vs_pubinfos.keys():
            return []

        return list(self.topic_vs_pubinfos[topic].keys())

    def get(self, topic, stamp=None, idx=None):
        """
        topic: str
        stamp: str such as 1618559286.884563157
        """
        ret = None
        if topic not in self.topic_vs_pubinfos.keys():
            return None

        if stamp is None and idx is None:
            return list(self.topic_vs_pubinfos[topic].values())[0]

        infos = self.topic_vs_pubinfos[topic]

        if stamp in infos.keys():
            ret = infos[stamp]

        if idx is not None:
            if len(infos.keys()) <= idx:
                print("args.idx too large, should be < {len(info.kens())}")
            else:
                key = sorted(infos.keys())[idx]
                ret = infos[key]

        return ret

    def in_topics(self, topic):
        """
        gather input topics by ignoring stamps
        return set of topic name
        """
        ret = set()

        if topic not in self.topic_vs_pubinfos.keys():
            return ret

        for pubinfo in self.topic_vs_pubinfos[topic].values():
            for t in pubinfo.in_infos.keys():
                ret.add(t)
        return ret

    def all_topics(self):
        out = set()

        lhs = self.topics()
        for t in lhs:
            out.add(t)

            rhs = self.in_topics(t)
            for t in rhs:
                out.add(t)
        return out
