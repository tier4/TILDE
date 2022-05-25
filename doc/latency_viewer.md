# Latency Viewer

Latency Viewer はあるセンサー入力からのレンテンシの表示ツールです。
PubInfo を利用して DAG を遡ることで、注目しているトピックからセンサーまで全てのパスにおけるレイテンシを確認できます。
vmstat や top の様に定期的にレイテンシ情報が更新されます。

起動方法やオプションについては [ビシュアライズツールの README](../src/tilde_vis/README.md) をご覧下さい。

## 出力例

trajectory_follower を表示した結果は以下の通りです。

トピック・header stamp・レイテンシが 2 種類が表示されます。

- トピック名
  - 前のスペースの数でパスが表示されます
- header stamp
  - メイントピックの header stamp です
- レイテンシ
  - ROS time ベースと steady time の 2 種類が表示されています。
  - レイテンシはある行のトピックが publish されてから `/control/trajectory_follower/lateral/control_cmd` が publish されるまでの経過時間です

```text
/control/trajectory_follower/lateral/control_cmd                                 1618559274.549348133      0      0
 /localization/twist                                                             1618559274.544330488      5      3
 /planning/scenario_planning/trajectory                                          1618559274.479350019     15     12
  (snip)
    /sensing/lidar/concatenated/pointcloud                                       1618559273.951109000    289    288
     /sensing/lidar/left/outlier_filtered/pointcloud                             1618559274.168087000    305    302
      /sensing/lidar/left/mirror_cropped/pointcloud_ex                           1618559274.168087000    305    304
       /sensing/lidar/left/self_cropped/pointcloud_ex                            1618559274.168087000    305    304
        /sensing/lidar/left/pointcloud_raw_ex*                                                     NA     NA     NA
     /sensing/lidar/right/outlier_filtered/pointcloud                            1618559274.170810000    300    301
      /sensing/lidar/right/mirror_cropped/pointcloud_ex                          1618559274.170810000    305    302
       /sensing/lidar/right/self_cropped/pointcloud_ex                           1618559274.170810000    305    303
        /sensing/lidar/right/pointcloud_raw_ex*                                                    NA     NA     NA
     /sensing/lidar/top/outlier_filtered/pointcloud                              1618559273.951109000    334    333
      /sensing/lidar/top/mirror_cropped/pointcloud_ex                            1618559273.951109000    410    407
       /sensing/lidar/top/self_cropped/pointcloud_ex                             1618559273.951109000    435    433
        /sensing/lidar/top/pointcloud_raw_ex*                                                      NA     NA     NA
     /vehicle/status/twist*                                                                        NA     NA     NA
```

以下のことが読みとれます。

- `trajectory_follower` は `/localization/twist` と `/planning/scenario_planning/trajectory` を入力とする
  - それぞれのレイテンシは ROS time (今回はシミュレーション時間) で 5 ms, 15 ms、 steady time で 3 ms, 12 ms.
- `/planning/scenario_planning/trajectory` に至るまでに concat filter (`/sensing/lidar/concatenated/pointcloud`) がある
  - 最終的に `/sensing/lidar/left/self_cropped/pointcloud_ex` などに辿りつく.
  - `/sensing/lidar/left/pointcloud_raw_ex` は終端(センサー)だが、TILDE 非対応の為、レイテンシ等は NA になっている
