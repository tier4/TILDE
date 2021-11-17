def time2str(t):
    return f"{t.sec}.{t.nanosec:09d}"

class TopicInfo(object):
    def __init__(self, topic, has_stamp, stamp):
        self.topic = topic
        self.has_stamp = has_stamp
        self.stamp = stamp

    def __str__(self):
        stamp_s = time2str(self.stamp) if self.has_stamp else "NA"
        return f"TopicInfo(topic={self.topic}, stamp={stamp_s})"

class PubInfo(object):
    """
    PubInfo
      - out_info = TopicInfo
      - in_infos = {topic_name => TopicInfo}
    """
    def __init__(self, out_topic, out_stamp):
        self.out_info = TopicInfo(out_topic, True, out_stamp)
        # topic name vs TopicInfo
        self.in_infos = {}

    def add_input_info(self, in_topic, has_stamp, stamp):
        self.in_infos[in_topic] = TopicInfo(in_topic, has_stamp, stamp)

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
        self.topic_vs_pubinfos = {}

    def add(self, pubinfo):
        out_topic = pubinfo.out_info.topic
        out_stamp = time2str(pubinfo.out_info.stamp)
        if not out_topic in self.topic_vs_pubinfos.keys():
            self.topic_vs_pubinfos[out_topic] = {}
        infos = self.topic_vs_pubinfos[out_topic]

        if out_stamp not in infos.keys():
            infos[out_stamp] = {}

        infos[out_stamp] = pubinfo

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
        stamp: str
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
