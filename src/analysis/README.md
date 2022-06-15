# old scripts for analysis

## Description

ros2 の bag や graph を操作するためのユーティリティ群。
オプションは `-h` でヘルプを参照のこと。

## topic_times.py

bag file 内のトピックの送信時刻を出力する。
時刻は header field の timestamp を参照する。無い場合はその旨出力して終了する。
シミュレータを動かし全トピックを計測の上、送信時刻を確認するなど。

```bash
$ ./topic_times.py <bagfile> <topic>

$ ./topic_times.py lsim_all_topics/lsim_all_topics_0.db3 /sensing/lidar/top/rectified/pointcloud
1618559266.752622000: <class 'sensor_msgs.msg._point_cloud2.PointCloud2'>
1618559266.850460000: <class 'sensor_msgs.msg._point_cloud2.PointCloud2'>
1618559266.950311000: <class 'sensor_msgs.msg._point_cloud2.PointCloud2'>
1618559267.506740000: <class 'sensor_msgs.msg._point_cloud2.PointCloud2'>
1618559267.150548000: <class 'sensor_msgs.msg._point_cloud2.PointCloud2'>
1618559267.250492000: <class 'sensor_msgs.msg._point_cloud2.PointCloud2'>
1618559267.350317000: <class 'sensor_msgs.msg._point_cloud2.PointCloud2'>
1618559267.450254000: <class 'sensor_msgs.msg._point_cloud2.PointCloud2'>
1618559267.550121000: <class 'sensor_msgs.msg._point_cloud2.PointCloud2'>
1618559267.650272000: <class 'sensor_msgs.msg._point_cloud2.PointCloud2'>
1618559267.750288000: <class 'sensor_msgs.msg._point_cloud2.PointCloud2'>
```

## topic_traversal.py

topic や node の from, to を指定して最短経路を取得する。
`rqt_graph` がバックエンドの為、実機やシミュレータなど対象システムが動作している状態で実行すること。

```bash
$ ./topic_traversal.py <from> <to>

$ ./topic_traversal.py /sensing/lidar/top/pointcloud_raw_ex /sensing/lidar/top/rectified/pointcloud
Graph refresh: done, updated[True]
ROS stats update took 1.032935380935669s

args:  /sensing/lidar/top/pointcloud_raw_ex -> /sensing/lidar/top/rectified/pointcloud
graph:  /sensing/lidar/top/pointcloud_raw_ex| ->  /sensing/lidar/top/rectified/pointcloud|

Found
  ' /sensing/lidar/top/pointcloud_raw_ex|'
  '/sensing/lidar/top/crop_box_filter_self|'
  ' /sensing/lidar/top/self_cropped/pointcloud_ex|'
  '/sensing/lidar/top/crop_box_filter_mirror|'
  ' /sensing/lidar/top/mirror_cropped/pointcloud_ex|'
  '/sensing/lidar/top/velodyne_interpolate_node|'
  ' /sensing/lidar/top/rectified/pointcloud|'
```

Found のうち、半角スペース始まりは topic、そうでないものは Node の様子(不確かか書き方なのは内部で使っている `rqt_graph` が加工したもののため)。

## graph_around

ある地点からグラフを BFS してノードやトピックを出力する。

```bash
$ ./graph_around.py /localization/pose_twist_fusion_filter/ekf_localizer --depth 2

depth: 0
now: '/localization/pose_twist_fusion_filter/ekf_localizer|'

depth: 1
now: ' /tf|'
now: ' /localization/pose_twist_fusion_filter/pose|'
now: ' /localization/pose_twist_fusion_filter/pose_with_covariance|'
now: ' /localization/pose_twist_fusion_filter/pose_with_covariance_without_yawbias|'
now: ' /localization/pose_twist_fusion_filter/twist|'

depth: 2
now: '/perception/object_recognition/prediction/transform_listener_impl_55931a9941c0|'
now: '/sensing/lidar/pointcloud_preprocessor/transform_listener_impl_5588e2e7d940|'
now: '/sensing/lidar/pointcloud_preprocessor/transform_listener_impl_5588e2f57090|'
(snip)
```

## parse_message_tracking_tag.py

PugInfo が保存された bag ファイルを message_tracking_tag.py のデータ書式に変換し
pkl 形式で保存する。
結果は `${CWD}/topic_infos.pkl` に保存される。

```bash
parse_message_tracking_tag.py <path/to/bagfile>
```

## message_tracking_tag_traverse.py

parse_message_tracking_tag.py で作成された pkl ファイルを元に E2E latency を算出する。
第三引数が終点となるトピック名。
第二引数は何番目に送信されたトピック(正確には Message_Tracking_Tag)を対象とするか。あまり若い数字だと初期化中の為辿り付けないセンサーが発生することがある。

```bash
./message_tracking_tag_traverse.py \
  <path/to>/topic_infos.pkl \
  1000 \
  /localization/pose_twist_fusion_filter/twist_with_covariance
```
