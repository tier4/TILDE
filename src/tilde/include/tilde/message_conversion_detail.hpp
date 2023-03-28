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

#ifndef TILDE__MESSAGE_CONVERSION_DETAIL_HPP_
#define TILDE__MESSAGE_CONVERSION_DETAIL_HPP_

#include <tuple>

namespace tilde
{

template <class K, class V>
struct Pair
{
  using Key = K;
  using Value = V;
};

template <typename Key, typename T>
struct Get;

template <typename Key, typename Head>
struct Get<Key, std::tuple<Head>>
{
  using type = typename std::conditional<
    std::is_same<Key, typename Head::Key>::value, typename Head::Value, std::nullptr_t>::type;
};

template <typename Key, typename Head, typename... Tail>
struct Get<Key, std::tuple<Head, Tail...>>
{
  using type = typename std::conditional<
    std::is_same<Key, typename Head::Key>::value, typename Head::Value,
    typename Get<Key, std::tuple<Tail...>>::type>::type;
};

}  // namespace tilde

#endif  // TILDE__MESSAGE_CONVERSION_DETAIL_HPP_
