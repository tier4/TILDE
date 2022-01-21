TILDE の想定ユースケース
===

TILDE では以下の様なユースケースを想定ユースケースしています。

- オンライン測定
- デッドライン検出
  - パスの途中終了
  - 入力情報の「賞味期限」検出

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [TILDE の想定ユースケース](#tilde-の想定ユースケース)
    - [TILDE で分かること](#tilde-で分かること)
    - [オンラインレイテンシ計測](#オンラインレイテンシ計測)
    - [デッドライン検出](#デッドライン検出)

<!-- markdown-toc end -->

## TILDE で分かること

ROS2 のトピック通信は、一般に以下の様な有向グラフ(DAG)を構成します。

![tilde_dag](./images/tilde_dag.svg)

TILDE では、各ノードでメインのトピックを publish する際に PubInfo という「メイントピックを構成する入力トピックの情報(トピック)」を同時に publish します。  
メッセージの特定の為、ROS2 の [std_msgs/msg/Header](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Header.msg) を用います。

例えば FusionNode は以下の様な PubInfo を送信します。
stamp はメイントピックの Header stamp で、説明の為単位は秒とします。

- **PubInfo(FusionNode)**: stamp=8 の `/sensor/fusion` は以下のトピックを参照した
  - t=5 に受信した stamp=4 の `/sensor/topic/A`
  - t=7 に受信した stamp=6 の `/sensor/topic/B`
  - 他、トピックの送信時刻などの付属情報

PlanningNode も同様の PubInfo を送信します。

- **PubInfo(PlanningNode)**: stamp=10 の `/planning/base` は以下のトピックを参照した
  - t=8 に受信した stamp=8 の `/sensor/fusion`
  - t=4 に受信した stamp=3 の `/sensor/topic/B`
  - `/planning/fback` は未受信

これらの情報から、stamp=10 の `/planning/base` は以下のセンサー情報を元に算出されたと分かります。

- **推論**: stamp=10 の `/planning/base` は以下のトピックを参照している
  - `/sensor/topic/A`: stamp=4 のもの(`/sensor/fusion` 経由) 
  - `/sensor/topic/B`: stamp=6 のもの(`/sensor/fusion` 経由) と stamp=3 のもの(直接受信)

同様の推論を繰り返すことで「stamp=X の `/conrol/cmd` はいつのセンサー情報を用いたか?」という問いに答えることができます。
PubInfo を元に、DAG を逆向きに遡ってデータの紐付けを行うことからこの様な推論を **PubInfo の探索** と呼称します。


## オンラインレイテンシ計測

上記の推論から stamp=10 の `/planning/base` は各センサーの最も古い値として以下を参照していることが分かります。

- `/sensor/topic/A`: stamp=4 のもの(`/sensor/fusion` 経由) 
- `/sensor/topic/B`: stamp=3 のもの(直接受信)

「センサーの stamp = センサーの計測時刻」と仮定するとこのデータを作るまで **センサー A 取得から 6 秒, センサー B からは 7 秒かかった** と言えます。

他のノードでも同様の推論が可能です。

また、 PubInfo の探索を途中で止めることでノード間にかかった時間も分かります。  
例えば 「/sernsor/fusion が送信されてから /control/cmd が送信されるまで X 秒かかった」などの推論ができます。

この様にして TILDE を用いてレンテンシを計測することが可能です。

**オンライン性**

PubInfo は topic で送信される為、計測対象のシステムを動かしながら PubInfo を解析することが可能です。  
latency viewer では top や vmstat の様にシステムを動かしながらレンテンシを見ることができます。  
TILDE ではこの様にシステムを動かしながらメトリククを計算できる性質を **オンライン** と呼称しています。  

**オンライン vs オフライン、TILDE と CARET**

[CARET](https://tier4.github.io/CARET_doc/) は LTTng のトレースポイントを用いて計算する為、システムを動かした後に性能を評価します。この様に事後に解析することを、**オフライン** と呼称しています。

レンテンシ計測の観点では TILDE と CARET は似ていますが、以下の様に特徴付けられます。

- TILDE はオンラインにレイテンシを計測できるが取得できるレイテンシは荒い
- CARET はオフライン計測だが、細かいレイテンシを取得できる


## デッドライン検出

**(2022/01/21 現在本機能は未実装)**

以下の様な DAG を考えます。この DAG は所定の時間内(例えば 100 ms 以内)に処理を追える必要があるとします。
NodeA、NodeB はタイマーで動作すると考えます。

![tilde_deadline](./images/tilde_deadline.svg)

上記の通り TILDE により TargetNode は topicA の stamp や送信時間を取得できる為、「TargetNode の処理開始時点で topicA 送信から 80 ms 経過している」等と推論できます。

その他 TILDE の情報を用いることで以下の様な問題を検出することができます。

- (1) 直接的な入力トピックの遅延 (case1)
  - NodeB やネットワークの遅延により TargetNode が topicB を想定した周期で受信できない場合があります。
  - TargetNode 実行時に「最後に topicB を受信した」を取得できる為、この様な問題を検出できます。
- (2) 間接的な入力トピックの遅延 (case2)
  - NodeB は正しい周波数で動作しているが NodeA が停止・あるいは topicA の遅延などで、NodeB が参照している情報が古い可能性があります。
  - TILDE では PubInfo を辿ることで、センサーや途中のトピックの諸元を調べることができるため、この様な問題を検出できます。





