# TILDE - Tilde Is Latency Data Embedding

TILDE は複数のノードにまたがるレイテンシ測定やデッドライン検出の為のフレームワークです。
TILDE はノードから出力されたトピックを遡り、元となった入力トピック(センサー)を特定することができます。
入力から出力までを辿れる為、レイテンシ計測とデッドライン検出が可能です。

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->

## Table of Contents

- [TILDE - Tilde Is Latency Data Embedding](#tilde---tilde-is-latency-data-embedding)
  - [Table of Contents](#table-of-contents)
  - [TILDE の目標](#tilde-の目標)
  - [想定ユースケース](#想定ユースケース)
  - [動作原理](#動作原理)
  - [API](#api)
    - [tilde::TildeNode](#tildetildenode)
  - [インストール, 組み込み](#インストール-組み込み)
  - [デッドライン検出](#デッドライン検出)
  - [周辺ツール](#周辺ツール)
  - [リポジトリ一覧](#リポジトリ一覧)

<!-- markdown-toc end -->

## TILDE の目標

ROS2 のトピック通信は、一般に以下の様な有向グラフ(DAG)を構成します。

![tilde_dag](./images/tilde_dag.svg)

「センサーの情報がアクチュエーターにいつ反映されるか」つまり「あるトピックはどのタイミングのセンサーの情報を参照したものか」は難しい問いです。

- DAG は必ずしも一本道では無いため、経路ごとに参照されているセンサーの時刻が異なる可能性があります。
  - 例えば PlanningNode は SensorNodeB からの情報を FusionNode 経由で受信する場合と直接受信する場合があります。
  - また PlanningNode は FeedbackNode を通じて SensorNodeC の情報を参照しています。
- ループが含まれる場合も考える必要があります。
  - FeedbackNode を通じて PlanningNode は潜在的には過去全ての SensorNodeA, SensorNodeB の情報を参照しているとも言えます。
- DAG からは分からないこともあります。
  - 例えば、ノードによってはメッセージをバッファリングし選択的に入力データを利用している可能性があります。

TILDE ではこの様な複雑な問題にも対応できる様、 DAG を解析し、ランタイム時に各トピックがいつのセンサー情報を元に作成されたかを求め、その情報を元にレイテンシ解析やデッドライン検出をすることを目標としています。

## 想定ユースケース

TILDE では以下の様なユースケースを想定ユースケースしています。

- オンラインレイテンシ測定
- デッドライン検出
  - パスの途中終了
  - 入力情報の「賞味期限」検出

詳しくは [usecase](./usecase.md) をご覧下さい。

## 動作原理

TILDE ではメイントピックの publish 時に MessageTrackingTag というメタ情報を `<topic名>/message_tracking_tag` に publish します。  
MessageTrackingTag は数百バイト程度のメッセージで、メイントピックを構成する入力トピックの情報が記載されます。  
MessageTrackingTag の詳細やオーバーヘッドについては [mechanism](./mechanism.md) をご覧下さい。

## API

TILDE では ROS2 rclcpp と同じ引数で名前少し異なる API 群を用意しています。  
ご自分のアプリケーションに取り込む際は rclcpp::Node を tilde::TildeNode に置換するなど、機械的な置換で利用可能です。

- Node
  - [tilde::TildeNode](../src/tilde/include/tilde/tilde_node.hpp). 下記も参照。
- Publisher
  - [tilde::Node::create_tilde_publisher()](../src/tilde/include/tilde/tilde_node.hpp)
  - [tilde::TildePublisher](../src/tilde/include/tilde/tilde_publisher.hpp)
  - [tilde::TildePublisher::publish()](../src/tilde/include/tilde/tilde_publisher.hpp)
- Subscription
  - [tilde::Node::create_tilde_subscription()](../src/tilde/include/tilde/tilde_node.hpp)
- MessageTrackingTag explicit API
  - [tilde::TildePublisherBase::set_explicit_input_info()](../src/tilde/include/tilde/tilde_publisher.hpp)
  - [tilde::TildePublisherBase::set_max_sub_callback_infos_sec()](../src/tilde/include/tilde/tilde_publisher.hpp)
- Deadline detection
  - T.B.D.

### tilde::TildeNode

rclcpp::Node の子クラスです。
以下の Parameter を持ちます。

- `enable_tilde`
  - boolean.
  - false の場合 TILDE の機能がオフになる。つまり Subscription callback でのフックや publish 時の MessageTrackingTag 送信が抑制される。
  - 2022/03/02 現在初期化時のみ指定可能。

## インストール, 組み込み

[build](./build.md)

既存のアプリケーションに TILDE を組み込む際は以下の流れになります。
※ TODO: TILDE ライブラリ（\*.so）の取込み手順

- rclcpp::Node, rclcpp::Node::create_publisher, rclcpp::Node::create_subscription に代わり tilde の各 API の利用
  - rclcpp::Node とコンストラクタを tilde::TildeNode に置換
  - create_subscription() を create_tilde_subscription() に置換
  - create_publisher() を create_tilde_publisher() に置換
  - rclcpp::Publisher を tilde::TildePublisher に置換
- 対象ファイルに include 文の追加
  - `#include "tilde/tilde_node.hpp"`
  - `#include "tilde/tilde_publisher.hpp"`
- package.xml に tilde を追加
  - `<depend>tilde</depend>`
- CMakeLists.txt に tilde を追加
- メッセージをバッファしているノードがあれば MessageTrackingTag explicit API の呼び出しを追加

サンプルプロジェクトは [tilde_sample](../src/tilde_sample) をご覧下さい。

- sample_publisher.cpp
  - ros2/demos の talker.cpp 相当のプログラムです
  - メイントピックの header.stamp の有無が異なる 2 サンプルがあります。
  - `TildeNode`, `create_tilde_publisher` の例です。
- relay_timer.cpp
  - sample_publisher.cpp のトピックを受信・転送するプログラムです。
  - `create_tilde_subscription` の例です。
- relay_timer_with_buffer.cpp
  - relay_timer.cpp で buffer がある例です。
  - relay_timer.cpp に加え explicit API を用いています。

## デッドライン検出

> **_NOTE:_** 2022/01/21 現在本機能は未実装

## 周辺ツール

- [latency viewer](./latency_viewer.md): オンラインレイテンシ表示ツール

## リポジトリ一覧

- tilde 本体
- tilde tools ← latency viewer を含む各種 TILDE ツールを 1 レポジトリにまとめる予定
