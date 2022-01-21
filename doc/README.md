TILDE - Tilde Is Latency Data Embedding 
===

TILDE はノード間のレイテンシ測定やデッドライン検出の為のフレームワークです。  
TILDE はノードから出力されたトピックを遡り、元となった入力トピック(センサー)を特定することができます。  
入力から出力までを辿れる為、レイテンシ計測やはデッドライン検出が可能です。

## TILDE の目標

ROS2 のトピック通信は、一般に以下の様な有向グラフ(DAG)を構成します。

![tilde_dag](./images/tilde_dag.svg)


「センサーの情報がアクチュエーターにいつ反映されるか」つまり「あるトピックはどのタイミングのセンサーの情報を参照したものか」は難しい問いです。
- DAG は必ずしも一本道では無いため、経路ごとに参照されているセンサーの時刻が異なる可能性があります。  
  - 例えば PlanningNode は BSernsorNode からの情報を FusionNode 経由で受信する場合と直接受信する場合があります。  
  - また PlanningNode は FeedbackNode を通じて CSernsorNode の情報を参照しています。  
- ループが含まれる場合も考える必要があります。 
  - FeedbackNode を通じて PlanningNode は潜在的には過去全ての ASersorNode, BSensorNode の情報を参照しているとも言えます。
- DAG からは分からないこともあります。
  - 例えば、ノードによってはメッセージをバッファリングし選択的に入力データを利用している可能性があります。  

TILDE ではこの様な複雑な問題にも対応できる様、 DAG を解析し、ランタイム時に各トピックがいつのセンサー情報を元に作成されたかを求め、その情報を元にレイテンシ解析やデッドライン検出をすることを目標としています。

## 想定ユースケース

TILDE では以下の様なユースケースを想定ユースケースしています。

- オンライン測定
- デッドライン検出
  - パスの途中終了
  - 入力情報の「賞味期限」検出

詳しくは [usecase](./usecase.md) をご覧下さい。

※ TODO: [topic info v2 検討](https://tier4.atlassian.net/wiki/spaces/ISPCOLLABO/pages/2161607374/topic+info+v2) あたりの情報をまとめる

## 動作原理

TILDE ではメイントピックの publish 時に PubInfo というメタ情報を `<topic名>/info/pub` に publish します。  
PubInfo は数百バイト程度のメッセージで、メイントピックを構成する入力トピックの情報が記載されます。  
PubInfo の詳細やオーバーヘッドについては [mechanism](./mechanism.md) をご覧下さい。

※ TODO: [TopicInfo v2 (=PubInfo) の PoC 速報値](https://tier4.atlassian.net/wiki/spaces/ISPCOLLABO/pages/2173829123/TopicInfo+v2+PubInfo+PoC) あたりの情報をまとめる

## API

TILDE では ROS2 rclcpp と同じ引数で名前少し異なる API 群を用意しています。  
ご自分のアプリケーションに取り込む際は rclcpp::Node を tilde::TildeNode に置換するなど、機械的な置換で利用可能です。

- Node
  - tilde::TildeNode
- Publisher
  - tilde::Node::create_tilde_publisher()
  - tilde::TildePublisherBase
  - tilde::TildePublisher
- Subscription
  - tilde::Node::create_tilde_subscription()
- PubInfo explicit API
  - tilde::TildePublisherBase::set_explicit_input_info()
- Deadline detection
  - T.B.D.

※ TODO: ↑のリストを各 API のコメントドキュメントへのリンクにする

※ TODO: [CARET_demos](https://github.com/tier4/CARET_demos) の様なサンプルプロジェクトを作り、その中のコードを解説する

## インストール・組み込み

既存のアプリケーションに TILDE を組み込む際は以下の流れになります。

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
  - `<depend>tilde_msg</depend>`
- メッセージをバッファしているノードがあれば PubInfo explicit API の呼び出しを追加

サンプルプロジェクトは以下を参照下さい。

※ TODO: [CARET_demos](https://github.com/tier4/CARET_demos) の様なサンプルプロジェクトを作りリンクする

## 周辺ツール

- [latency viewer](./latency_viewer.md)

## リポジトリ一覧

- tilde 本体
- tilde tools ← latency viewer を含む各種 TILDE ツールを 1 レポジトリにまとめる予定
