# droneschool

send_ned_distance.py

  コア関数：send_ned_distance

    X,Y,Z方向の距離を指定して、飛行させます。

      distance_x: X方向の距離（単位:m）
      distance_y: Y方向の距離（単位:m）
      distance_z: Z方向の距離（単位:m）
      duration:   飛行時間（単位:s）※省略時:1
      max_speed:  飛行速度（単位:m/s）※省略時:1

 特徴：
* set_position_target_local_ned_encode を距離指定で動く様にラッピングしています。
* max_speed を超える場合は、飛行時間を延ばして調整しています。
* 距離の理論値と、GPSで計測した実測値を表示し、比べられる様にしています。
* 停止（ホバリング）には、フレア操作を実装しています。
