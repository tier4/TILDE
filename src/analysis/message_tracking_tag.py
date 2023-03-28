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


class Message_Tracking_Tag(object):
    """
    Message_Tracking_Tag.

      - out_info = TopicInfo
      - in_infos = {topic_name => list of TopicInfo}
    """

    def __init__(self, out_topic, out_stamp):
        self.out_info = TopicInfo(out_topic, True, out_stamp)
        # topic name vs TopicInfo
        self.in_infos = {}

    def add_input_info(self, in_topic, has_stamp, stamp):
        self.in_infos.setdefault(in_topic, []).append(TopicInfo(in_topic, has_stamp, stamp))

    def __str__(self):
        s = "Message_Tracking_Tag: \n"
        s += f"  out_info={self.out_info}\n"
        for _, ti in self.in_infos.items():
            s += f"  in_infos={ti}\n"
        return s

    @property
    def out_topic(self):
        return self.out_info.topic


class Message_Tracking_Tags(object):
    """
    Topic vs Message_Tracking_Tag.

    We have double-key dictionary internally, i.e.
    we can get Message_Tracking_Tag by topic_vs_message_tracking_tag[topic_name][stamp].
    """

    def __init__(self):
        self.topic_vs_message_tracking_tags = {}

    def add(self, message_tracking_tag):
        out_topic = message_tracking_tag.out_info.topic
        out_stamp = time2str(message_tracking_tag.out_info.stamp)
        if out_topic not in self.topic_vs_message_tracking_tags.keys():
            self.topic_vs_message_tracking_tags[out_topic] = {}
        infos = self.topic_vs_message_tracking_tags[out_topic]

        if out_stamp not in infos.keys():
            infos[out_stamp] = {}

        infos[out_stamp] = message_tracking_tag

    def topics(self):
        return list(self.topic_vs_message_tracking_tags.keys())

    def stamps(self, topic):
        """Return List[stamps]."""
        if topic not in self.topic_vs_message_tracking_tags.keys():
            return []

        return list(self.topic_vs_message_tracking_tags[topic].keys())

    def get(self, topic, stamp=None, idx=None):
        """
        Get corresponding Message_Tracking_Tag.

        topic: str
        stamp: str
        """
        ret = None
        if topic not in self.topic_vs_message_tracking_tags.keys():
            return None

        if stamp is None and idx is None:
            return list(self.topic_vs_message_tracking_tags[topic].values())[0]

        infos = self.topic_vs_message_tracking_tags[topic]

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
        Gather input topics by ignoring stamps.

        return set of topic name
        """
        ret = set()

        if topic not in self.topic_vs_message_tracking_tags.keys():
            return ret

        for message_tracking_tag in self.topic_vs_message_tracking_tags[topic].values():
            for t in message_tracking_tag.in_infos.keys():
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
