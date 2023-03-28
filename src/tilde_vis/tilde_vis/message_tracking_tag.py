# Copyright 2021 Research Institute of Systems Planning, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Internal data structures for MessageTrackingTag."""

from rclpy.time import Time


def time2str(t):
    """Convert builtin_interfaces.msg.Time to string."""
    return f'{t.sec}.{t.nanosec:09d}'


class TopicInfo(object):
    """Represent output_info and input_infos of MessageTrackingTag message."""

    def __init__(self, topic, pubsub_stamp, pubsub_stamp_steady,
                 has_stamp, stamp):
        """
        Hold information similar to MessageTrackingTag in/out info.

        :param topic: target topic [string]
        :param pubsub_stamp:
            when publish or subscription callback is called
            [builtin_interfaces.msg.Time]
        :param pubsub_stamp_steady:
            same with above but in steady time
            [builtin_interfaces.mg.Time]
        :param has_stamp: whether main topic has header.stamp or not [bool]
        :param stamp: header.stamp [builtin_interfaces.msg.Time]

        """
        self.topic = topic
        self.pubsub_stamp = pubsub_stamp
        self.pubsub_stamp_steady = pubsub_stamp_steady
        self.has_stamp = has_stamp
        self.stamp = stamp

    def __str__(self):
        """Get string."""
        stamp_s = time2str(self.stamp) if self.has_stamp else 'NA'
        return f'TopicInfo(topic={self.topic}, stamp={stamp_s})'


class MessageTrackingTag(object):
    """
    Hold information similar to MessageTrackingTagMsg.

    TODO(y-okumura-isp): Can we use MessageTrackingTagMsg directly?

    """

    def __init__(self, out_topic, pub_time, pub_time_steady,
                 has_stamp, out_stamp):
        """
        Initialize data.

        :param out_topic: topic name
        :param pub_time: when publish main topic [builtin_interfaces.msg.Time]
        :param has_stamp: whether main topic has header.stamp [bool]
        :param out_stamp: main topic header.stamp [builtin_interfaces.msg.Time]

        """
        self.out_info = TopicInfo(out_topic, pub_time, pub_time_steady,
                                  has_stamp, out_stamp)
        # topic name vs TopicInfo
        self.in_infos = {}

    def add_input_info(self, in_topic, sub_stamp, sub_stamp_steady,
                       has_stamp, stamp):
        """
        Add input info.

        :param in_topic: topic string
        :param has_stamp: bool
        :param stamp: header stamp [builtin_interfaces.msg.Time]

        """
        info = TopicInfo(in_topic, sub_stamp, sub_stamp_steady,
                         has_stamp, stamp)
        self.in_infos.setdefault(in_topic, []).append(info)

    def __str__(self):
        """Get string."""
        s = 'MessageTrackingTag: \n'
        s += f'  out_info={self.out_info}\n'
        for _, infos in self.in_infos.items():
            for info in infos:
                s += f'  in_infos={info}\n'
        return s

    @property
    def out_topic(self):
        """Get output topic."""
        return self.out_info.topic

    @staticmethod
    def fromMsg(message_tracking_tag_msg):
        """
        Convert MessageTrackingTagMsg to MessageTrackingTag.

        :param message_tracking_tag_msg: MessageTrackingTagMsg
        :return MessageTrackingTag

        """
        output_info = message_tracking_tag_msg.output_info
        message_tracking_tag = MessageTrackingTag(
            output_info.topic_name,
            output_info.pub_time,
            output_info.pub_time_steady,
            output_info.has_header_stamp,
            output_info.header_stamp)
        for input_info in message_tracking_tag_msg.input_infos:
            message_tracking_tag.add_input_info(
                input_info.topic_name,
                input_info.sub_time,
                input_info.sub_time_steady,
                input_info.has_header_stamp,
                input_info.header_stamp)
        return message_tracking_tag


class MessageTrackingTags(object):
    """
    Hold topic vs MessageTrackingTag.

    We have double-key dictionary internally, i.e.
    we can get MessageTrackingTag by
    topic_vs_message_tracking_tag[topic_name][stamp].

    """

    def __init__(self):
        """Initialize data."""
        # {topic => {stamp_str => MessageTrackingTag}}
        self.topic_vs_message_tracking_tags = {}

    def add(self, message_tracking_tag):
        """Add a MessageTrackingTag."""
        out_topic = message_tracking_tag.out_info.topic
        out_stamp = time2str(message_tracking_tag.out_info.stamp)
        if out_topic not in self.topic_vs_message_tracking_tags.keys():
            self.topic_vs_message_tracking_tags[out_topic] = {}
        infos = self.topic_vs_message_tracking_tags[out_topic]

        if out_stamp not in infos.keys():
            infos[out_stamp] = {}

        infos[out_stamp] = message_tracking_tag

    def erase_until(self, stamp):
        """
        Erase added message_tracking_tag where out_stamp < stamp.

        :param stamp: builtin_interfaces.msg.Time

        """
        def time_ge(lhs, rhs):
            """
            Compare time.

            Helper function to compare stamps i.e.
            self.topic_vs_message_tracking_tags[*].keys().

            As stamps are string, it is not appropriate to compare as string.
            Builtin_msg.msg.Time does not implement `<=>`, we use rclpy.Time,
            although clock_type has no meaning.

            """
            lhs_sec, lhs_nano_sec = (int(x) for x in lhs.split('.'))
            rhs_sec, rhs_nano_sec = (int(x) for x in rhs.split('.'))
            lhs_time = Time(seconds=lhs_sec, nanoseconds=lhs_nano_sec)
            rhs_time = Time(seconds=rhs_sec, nanoseconds=rhs_nano_sec)
            return lhs_time <= rhs_time

        thres_stamp = time2str(stamp)
        erases = {}
        for (topic, infos) in self.topic_vs_message_tracking_tags.items():
            for stamp in infos.keys():
                if time_ge(thres_stamp, stamp):
                    continue
                erases.setdefault(topic, []).append(stamp)

        for (topic, stamps) in erases.items():
            for stamp in stamps:
                del self.topic_vs_message_tracking_tags[topic][stamp]

    def topics(self):
        """Get topic names which has registered MessageTrackingTag."""
        return list(self.topic_vs_message_tracking_tags.keys())

    def stamps(self, topic):
        """Get List[stamps]."""
        if topic not in self.topic_vs_message_tracking_tags.keys():
            return []

        return list(self.topic_vs_message_tracking_tags[topic].keys())

    def get(self, topic, stamp=None, idx=None):
        """
        Get a MessageTrackingTag.

        :param topic: str
        :param stamp: str such as 1618559286.884563157
        :return MessageTrackingTag or None

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
                print('args.idx too large, should be < {len(info.kens())}')
            else:
                key = sorted(infos.keys())[idx]
                ret = infos[key]

        return ret

    def in_topics(self, topic):
        """
        Gather input topics by ignoring stamps.

        :param topic: topic name
        :return set of topic name

        """
        ret = set()

        if topic not in self.topic_vs_message_tracking_tags.keys():
            return ret

        for message_tracking_tag in self.topic_vs_message_tracking_tags[topic].values():
            for t in message_tracking_tag.in_infos.keys():
                ret.add(t)
        return ret

    def all_topics(self):
        """Get all topics which are used as input or output."""
        out = set()

        lhs = self.topics()
        for t in lhs:
            out.add(t)

            rhs = self.in_topics(t)
            for t in rhs:
                out.add(t)
        return out
