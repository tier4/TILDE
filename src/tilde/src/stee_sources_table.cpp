#include "tilde/stee_sources_table.hpp"

using tilde::SteeSourcesTable;

SteeSourcesTable::SteeSourcesTable(
    size_t default_max_stamps_per_topic,
    std::map<SteeSourcesTable::TopicName, size_t> max_stamps_per_topic)
    : default_max_stamps_per_topic_(default_max_stamps_per_topic),
      max_stamps_per_topic_(max_stamps_per_topic)
{
}

void SteeSourcesTable::set(
    const SteeSourcesTable::TopicName & topic,
    const SteeSourcesTable::Stamp & stamp,
    const SteeSourcesTable::SourcesMsg & sources_msg)
{
  sources_[topic][stamp] = sources_msg;

  auto max_stamps = default_max_stamps_per_topic_;
  auto max_it = max_stamps_per_topic_.find(topic);
  if (max_it != max_stamps_per_topic_.end()) {
    max_stamps = max_it->second;
  }

  auto stamp_vs_sources = sources_[topic];
  if (stamp_vs_sources.size() == max_stamps) {
    stamp_vs_sources.erase(stamp_vs_sources.begin());
  }
}

SteeSourcesTable::TopicSources
SteeSourcesTable::get_latest_sources() const
{
  SteeSourcesTable::TopicSources ret;

  for(const auto & topic_it : sources_) {
    const auto & topic = topic_it.first;
    const auto & sources = topic_it.second.crbegin()->second;
    ret[topic] = sources;
  }

  return ret;
}

SteeSourcesTable::SourcesMsg SteeSourcesTable::get_sources(
    const SteeSourcesTable::TopicName & topic,
    const SteeSourcesTable::Stamp & stamp) const
{
  SteeSourcesTable::SourcesMsg empty;

  auto it = sources_.find(topic);
  if (it == sources_.end()) {
    return empty;
  }

  auto it_stamp_vs_sources = it->second.find(stamp);
  if (it_stamp_vs_sources == it->second.end()) {
    return empty;
  }

  return it_stamp_vs_sources->second;
}

