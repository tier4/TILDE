// Copyright 2021 Research Institute of Systems Planning, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Provide fake header guard for cpplint
#undef TILDE__TP_H_
#ifndef TILDE__TP_H_
#define TILDE__TP_H_

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER ros2_caret

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "tilde/tp.h"

#if !defined(_TP_H) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _TP_H

#include <lttng/tracepoint.h>

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, tilde_subscription_init,
  TP_ARGS(
    const void *, subscription_arg, const char *, node_name_arg, const char *, topic_name_arg),
  TP_FIELDS(ctf_integer_hex(const void *, subscription, subscription_arg)
              ctf_string(node_name, node_name_arg) ctf_string(topic_name, topic_name_arg)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, tilde_subscribe,
  TP_ARGS(const void *, subscription_arg, const uint64_t, tilde_message_id_arg),
  TP_FIELDS(ctf_integer_hex(const void *, subscription, subscription_arg)
              ctf_integer(const uint64_t, tilde_message_id, tilde_message_id_arg)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, tilde_publisher_init,
  TP_ARGS(const void *, publisher_arg, const char *, node_name_arg, const char *, topic_name_arg),
  TP_FIELDS(ctf_integer_hex(const void *, publisher, publisher_arg)
              ctf_string(node_name, node_name_arg) ctf_string(topic_name, topic_name_arg)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, tilde_subscribe_added,
  TP_ARGS(
    const void *, subscription_id_arg, const char *, node_name_arg, const char *, topic_name_arg),
  TP_FIELDS(ctf_integer_hex(const void *, subscription_id, subscription_id_arg)
              ctf_string(node_name, node_name_arg) ctf_string(topic_name, topic_name_arg)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, tilde_publish,
  TP_ARGS(
    const void *, publisher_arg, const uint64_t, tilde_publish_timestamp_arg, const void *,
    subscription_id_arg, const uint64_t, tilde_message_id_arg),
  TP_FIELDS(ctf_integer_hex(const void *, publisher, publisher_arg)
              ctf_integer(const uint64_t, tilde_publish_timestamp, tilde_publish_timestamp_arg)
                ctf_integer_hex(const void *, subscription_id, subscription_id_arg)
                  ctf_integer(const uint64_t, tilde_message_id, tilde_message_id_arg)))
#endif /* _TP_H */

#include <lttng/tracepoint-event.h>

#endif  // TILDE__TP_H_
