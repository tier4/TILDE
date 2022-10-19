### TILDE aggregator 検討 (1)

tilde_aggregator は、指定されたパスにおけるトピックの流れが、指定デッドライン時間以内に完了するかどうかを **message_tracking_tag(mtt)**
トピックを監視することで実現し、完了しない場合、デッドラインミス検出トピックを発行する。

- パスのデッドラインミス検出
- デッドラインミス発生トピック送信

#### 1. 処理概要

##### 1-1 パス指定： 初期起動時に、外部パラメータで与える
  - 監視パスをトピックの流れで指定
  - デッドラインタイマ値を指定 
##### 1-2 アクティブトピック検出
  - 現在稼働しているシステム上でトピックを検索する
  - 定期的(デフォルト10秒)に実行し、1-3 と連動して、subscriber を更新する
##### 1-3  指定トピック受信
   -  パス指定トピックがアクティブトピックで検出されると、該当トピックを subscribe する
##### 1-4 受信トピック解析
最初のトピック受信時点から、パス内全トピックが規定時間以内に受信することを監視するので、トップダウンにパスを構築することになる。最終トピックから逆順に辿ることは意味がない。

  - パスの先頭トピック
    - パス管理情報を生成する。これをアクティブパスと呼ぶ
      - 先頭トピック受信毎にアクティブパスを生成するので、同時に複数存在する可能性がある
    - デッドラインタイマ起動
  - 先頭トピックでなければ、以下の処理を行う
    - アクティブパスがなければ、MSGをバッファリングする
    - アクティブパス内に、受信トピックが含まれていなければ、MSGをバッファリングする
    - アクティブパス内の条件に合致すれば、該当アクティブパスに受信トピックを登録
      -  既に受信しているトピックの output_info の header_stamp時間が、受信トピックのinput_infos内のトピック名＋header_stamp時間に一致
      -  既に受信しているトピックの input_infos内のトピック名＋header_stamp時間が、受信トピックの output_info の header_stamp時間に一致
 
※ 受信トピック毎に統計を取得(主にhz)

##### 1-5 パス完結チェック
- 1-4項で受信トピックを登録すると、パスが完結したかいかどうかをチェックする
  - 完結すれば、デッドラインタイマ停止
  - アクティブパス削除

※ パス統計(完結時間など)

##### 1-6 デッドラインタイマタイムアウト検出
- デッドラインタイマは 100(ms)の tick をカウントし、各アクティうパスのデッドラインタイマから tickを減算することで実現する
- デッドラインミス検出トピック送信 (初期実装では、単にプリントアウト)
- アクティブパス削除

※ パス統計(失敗回数など)

##### 1-7 ペンディングメッセージの再処理
- 1-4 で受信トピックがペンディングにならずに、いずれかのアクティブパスに登録された場合、ペンディング中のメッセージが処理できる可能性がある
- 処理できたアクティブパスのトピックリストにある未受信トピックについて、ペンディングメッセージを探し、1-4と同じ処理を行う
- ペンディングメッセージ自体のpub時間が、いずれのアクティブパスの先頭トピックのpub時間より小さくなった場合、該当ペンディングメッセージは処理する対象がなくなったと判断して、削除する

##### 1-8 トピックのスキップ (未)
途中ノードが非TILDEノードの場合など、途中トピックに対応するMTTが抜けてしまう。そのような場合は、該当トピックをスキップする。
スキップトピックを subscribe しているトピックは、スキップトピック以外のパス内トピックの subscribe に読み替える。但し。header_stamp は変わらないことを前提とする。


#### 2. 実装案

##### 2-1 パス指定
起動時、yamlファイルで与える
想定形式
```
params:
  # nothing
PATH_gyro_odometer: # 任意のパス名
  deadline_timer: 3.0
  topic_list:
    /sensing/imu/imu_data: # sensor_msgs/msg/Imu
    /sensing/imu/tamagawa/imu_raw: # sensor_msgs/msg/Imu
    /sensing/vehicle_velocity_converter/twist_with_covariance: # geometry_msgs/msg/TwistWithCovarianceStamped
    /vehicle/status/velocity_status: # autoware_auto_vehicle_msgs/msg/VelocityReport

PATH_concat_data:
  deadline_timer: 8.0
  topic_list:
    /sensing/lidar/left/outlier_filtered/pointcloud: # sensor_msgs/msg/PointCloud2
    /sensing/lidar/right/outlier_filtered/pointcloud: # sensor_msgs/msg/PointCloud2
    /sensing/lidar/top/outlier_filtered/pointcloud: # sensor_msgs/msg/PointCloud2
    /vehicle/status/velocity_status: # autoware_auto_vehicle_msgs/msg/VelocityReport
```
##### 2-1 パス情報

|No|項目|内容|
|--|--|--|
|1|path_name|yamlで指定したパス名|
|2|deadline_timer|yamlで指定したデッドラインタイマ値|
|3|timer_counter|デッドラインタイマ用tickカウンタ|
|4|status|パス状態：初期状態・監視中・完了・タイムアウト|
|5|start|パス開始時間(rosタイムで秒float)|
|6|end|パス終了時間|
|7|トピックリスト|yamlで指定したトピック数分保持|

##### 2-2 トピック情報
2-1 のトピックリストは、以下の形式のリストとして保持する。
|No|項目|内容|
|--|--|--|
|1|name|yamlで指定したトピック名|
|2|status|トピック状態：初期状態・受信待ち・受信済|
|3|msg|受信message_tracking_tagメッセージを保持|
|4|recv_time|本トピックのコールバックが呼ばれた時間(rosタイム、秒float)|

##### 2-3 アクティブパスリスト
アクティブ(完結もタイムアウトもしていないが、少なくとも先頭トピックを受信している)パスと、現在ペンディングになっているメッセージを管理する。
|No|項目|内容|
|--|--|--|
|1|active_path|アクティブパス情報のリスト|
|2|pending_message|未処理メッセージリスト{トピック名:受信メッセージ}のリスト|

##### 2-4 デッドラインミス検出トピック (未)



#### 3. フロー (未) 

```plantuml
title fig.1 動作概要
hide footbox
box "tilde_aggregator operation" #Transparent
  participant "tilde_aggregator" as ta
  participant "target-node1(start)" as n1
  participant "target-node2" as n2
  participant "target-node3" as n3
  participant "target-node4" as n4
end box

  participant "autoware/diag" as aw

group パス完結 (path-A)
ta++
[->n1++: origin topic
n1->n2++:top_topic
n2->ta:MTT(top_topic)
ta->ta++--:make active-path instance
ta->ta++--:start deadline timer
n2->n3++:2nd_topic
n2->ta:MTT(2nd_topic)
ta->ta++--:register topic in this path
n3->n4++:3rd_topic
n4->ta:MTT(3rd-topic)
ta->ta++--:register topic in this path

note over ta: パス構築完了\n- タイマキャンセル\n- パス解放
end

group デッドラインミス検出 (path-A)
ta++
[->n1: origin topic
n1->n2:top_topic
n2->ta:MTT(top_topic)
ta->ta++--:make active-path instance
ta->ta++--:start deadline timer
n2->n3:2nd_topic
n2->ta:MTT(2nd_topic)
ta->ta++--:register topic in this path
n3->n4:3rd_topic
destroy n4
ta->ta:★TIMEOUT occured★
ta->]:deadline miss topic (path-A)
note over ta: デッドラインミス検出\n- deadline missトピック創出\n- パス解放
end

```




##### message_tracking_tag(mtt)
```
header:
  stamp:
    sec: 1585897259
    nanosec: 882025458
  frame_id: ''
output_info:
  topic_name: /localization/twist_estimator/gyro_twist
  node_fqn: ''
  seq: 129
  pub_time:                 # 本mttの本トピックのpub時間(rosタイム)
    sec: 1585897259
    nanosec: 882025458
  pub_time_steady:          # pub時間をシステム起動からの時間に換算(?)
    sec: 48468
    nanosec: 495195629
  has_header_stamp: true    # MSGが本来保持していたstamp (pub時間ではないケースもある) 
  header_stamp:
    sec: 1585897259
    nanosec: 901549544
input_infos:
- topic_name: /sensing/imu/imu_data # 本トピックの元になったトピック
  sub_time:                # ?? sub時間? pub時間ではないか?
    sec: 1585897259
    nanosec: 882025458
  sub_time_steady:
    sec: 48468
    nanosec: 495096713
  has_header_stamp: true   # MSGが本来保持していたstamp (pub時間ではないケースもある)
  header_stamp:
    sec: 1585897259
    nanosec: 901549544
- topic_name: /sensing/vehicle_velocity_converter/twist_with_covariance
  sub_time:
    sec: 1585897259
    nanosec: 841343690
  sub_time_steady:
    sec: 48468
    nanosec: 297550086
  has_header_stamp: true
  header_stamp:
    sec: 1585897259
    nanosec: 832886384
```

#### 統計等
topic毎 hz min/max/ave
パス毎 完了数・デッドラインミス数
 - 完了時間 min/max/ave

同時処理インスタンス数(統計ではなくリアルタイム)

#### コマンド
トピックで内部状態や統計をダンプ


