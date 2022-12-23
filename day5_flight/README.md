## droneschool(Day5)

### SITL + Mission Planner の起動方法

1. 下記コマンドで Ubuntu から SITL を起動する。

   * コプター、メインポートをホームにする場合
      ```
      $ sim_vehicle.py -v Copter --console --map -D --custom-location=35.878275,140.338069,0,0 -i 1
      ```

   * ローバー1、隣接ポートをホームにする場合
      ```
      $ sim_vehicle.py -v Rover --console --map -D --custom-location=35.867003,140.305987,0,0 -i 2
      ```

   * ローバー2、対岸ポートをホームにする場合
      ```
      $ sim_vehicle.py -v Rover --console --map -D --custom-location=35.879768,140.348495,0,0 -i 3
      ```

   * ボート1、メインポートをホームにする場合
      ```
      $ sim_vehicle.py -v Rover --frame=motorboat --console --map -D --custom-location=35.8773498,140.3383255,0,0 -i 4
      ```

   * ボート2、対岸ポートをホームにする場合
      ```
      $ sim_vehicle.py -v Rover --frame=motorboat --console --map -D --custom-location=35.8802285,140.3476155,0,0 -i 5
      ```

   * プレーン、任意地点をホームにする場合
      ```
      $ sim_vehicle.py -v Plane --console --map -D --custom-location=35.878275,140.338069,0,0 -i 6
      ```

2. ポート番号を機体コンソールで確認する。
   * 「5xx3」はプログラムで使用する
   ```
   Smoothing reset at 0.001
   bind port 5xx2 for 2
   Serial port 2 on TCP port 5xx2
   bind port 5773 for 3
   Serial port 3 on TCP port 5xx3
   ```
3. Mission Planner から下記操作を機体数分繰り返し、複数機体に接続する。

   1. 「接続」で左クリック
   2. 「接続オプション」を選択
   3. 「TCP」,「11520」を選択
   4. 「Connect」を押下
   5. IP/ポート番号の入力し、「OK」を押下
      ```
      Enter host name/ip	127.0.0.1
      Enter remote port	5xx2
      ```

### インストール方法

    $ git clone https://github.com/csc-takeda/droneschool.git

### 実行方法

    $ cd day5_flight
    $ python3 day5_flight.py

