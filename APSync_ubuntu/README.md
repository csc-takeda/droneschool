# APSync T265 ubuntu20.04 環境構築

## イメージファイルで環境構築

① 既存の APSync イメージをインストール

* [firmware.ardupilot.org](https://firmware.ardupilot.org/Companion/apsync/) より、"apsync-rpi-ubuntu-t265-latest.img.xz" をダウンロードし、<br>
  最近購入した Raspberry Pi 4 Model B (4GB) を使用して起動を試みてみましたが、<br>
  **<font color="red">ハードに対してファームウェアが古いため起動できませんでした。</font>**

  <img alt="[起動画面1" src="image/起動1.JPG" width="90%">

## ubuntu20.04 をインストールして環境構築

### ① ubuntu20.04 インストール

[Rasberry Pi 4 Model BにUbuntu Server 22.04 LTSを導入](https://www.kabukigoya.com/2022/05/rasberry-pi-4-model-bubuntu-server-2204.html) を参考に、<br>
Ubuntu Server 20.04.5 LTS (64-bit) をインストールします。

<img alt="インストール画面1" src="image/インストール1.png" width="70%">


* インストール後に設定するよりも簡単に設定できるので、<br>
  インストール時に以下の設定をお勧めします。

  * SSHの有効化
  * ユーザ名とパスワードの設定
  * Wi-Fiの設定
  * ロケールの設定

  <img alt="インストール画面2" src="image/インストール2.png" width="70%">


### ② IPアドレスの確認

インストール後にログインし、以下を参考にIPアドレスを確認します。
```
$ ip a
```

### ③ パッケージの更新

以下を参考にパッケージを更新します。
```
$ sudo apt update && sudo apt upgrade -y
```

### ④ GUI 環境の構築 

以下を参考に GUI（Lubuntu）をインストールします。
```
$ mkdir ~/GitHub
$ cd ~/GitHub
$ git clone https://github.com/wimpysworld/desktopify.git
$ cd desktopify/
$ sudo ./desktopify --de lubuntu
$ sudo reboot
```
* ターミナルに xterm が選択され、ターミナルが開かなくなった場合は、<br>
Edit->Preferences->Advanced->Terminal emulator: qterminal を選択します。

### ⑤ UART 設定変更
Ubuntu 20.04の UART0(/dev/serial0) は、デフォルトでシリアルコンソールに割り当てられているので<br>
**<font color="red">そのままの設定で UART0 を使用すると、OS が起動しなくなります。</font>**
* GUIをインストールしていると、メッセージも出さないまま起動しません。
* 下記はGUIをインストールしていない環境での起動画面の例です。

  <img alt="[起動画面2" src="image/起動2.JPG" width="90%">

[Raspberry Pi 4 + Ubuntu 20.04で複数のUARTを有効にする](https://tshell.hatenablog.com/entry/2021/03/04/205346) と、
[The config.txt file](https://www.raspberrypi.com/documentation/computers/config_txt.html) を参考に、<br>
UART2（/dev/ttyAMA1）を有効にします。

[カメラの有効化 ── Ubuntu の場合](http://m-ac.jp/raspi/hardware/camera/enable_camera/ubuntu/index_j.phtml) を参考に、
camera（/dev/video0）を有効にします。
```
$ cd /boot/firmware/

$ sudo cp -pi config.txt config.bak

$ sudo vi config.txt
$ diff config.txt config.bak
65,67d64
< dtoverlay=uart2
< #dtoverlay=disable-bt
< #arm_boost=1
69,70d65
< #start_x=1
< #gpu_mem=128

$ sudo adduser $USER dialout
$ sudo reboot
```

* [Raspberry PIのGPIO上シリアルとArduinoの通信](https://qiita.com/ryugyoku/items/bf5fd10512c84a55d030) を参考に、<br>
  UART0（/dev/serial0）を無効にしても起動しません（効果なし）。<br>
  ⇒　**<font color="red">UART0(8, 10pin) は使用不可とします。</font>**
* [MAVProxy (Raspiコンパニオンコンピューター接続・インストール)](https://playing-engineer.com/wordpress/2022/12/05/ardupilot-raspi%e3%82%b3%e3%83%b3%e3%83%91%e3%83%8b%e3%82%aa%e3%83%b3%e3%82%b3%e3%83%b3%e3%83%94%e3%83%a5%e3%83%bc%e3%82%bf%e3%83%bc2/) を参考に、<br>
  Bluetooth をオフにしても起動しません（効果なし）。<br>
  ⇒　設定はコメントに変更しました。
* /dev/video0 が作成されない　⇒　カメラ（T265）動作に無関係。<br>
  ⇒　設定はコメントに変更しました。

### ⑥ UART 接続
* raspberry pi 4 Tx：27pin、Rx：28pin と FCU Telem2 Rx、Tx を接続します。

  <img alt="UART接続" src="image/UART接続.JPG" width="80%">

## カメラ（T265）環境構築

[Ubuntu 20.04 on Raspberry PI 4 model BでRealsenseカメラ(D415)を動かしてみる](https://qiita.com/nv-h/items/3bd206bebf9303dd6624) を参考に、<br>
Realsense SDK をインストールします。

* [Build Configuration - Intel RealSense](https://dev.intelrealsense.com/docs/build-configuration) を参考に、<br>
「DFORCE_RSUSB_BACKEND」を使用する手順例を選択しました。	

### ① 依存パッケージのインストール
```
$ sudo apt update && sudo apt upgrade -y

$ sudo apt install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev -y
$ sudo apt install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev -y
$ sudo apt install cmake -y
$ sudo apt install g++ -y
$ sudo apt install python3-dev -y
```

### ② Realsense SDKのダウンロード
```
$ cd ~/GitHub
$ git clone https://github.com/IntelRealSense/librealsense
$ cd librealsense
$ git checkout -b v2.53.1 refs/tags/v2.53.1
$ git branch
  master
* v2.53.1
```
※ 執筆時点の最新版が「v2.53.1」になります。

### ③ ビルド
```
$ mkdir build && cd build
$ cmake ../ \
-DFORCE_RSUSB_BACKEND=true \
-DCMAKE_BUILD_TYPE=release \
-DBUILD_EXAMPLES=true \
-DBUILD_GRAPHICAL_EXAMPLES=true \
-DBUILD_PYTHON_BINDINGS=true \
-DPYTHON_EXECUTABLE=$(which python3)

$ sudo make uninstall && make clean && make && sudo make install
```

### ④ realsense-viewer用設定
```
$ sudo cp ~/.99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules && sudo udevadm control --reload-rules && udevadm trigger
$ sudo reboot
```

### ⑤ Viwerの実行

Rasberry Pi の GUI 上のターミナルから、T265 の動作確認をします。
```
$ realsense-viewer
```
**<font color="red">※ 下記のエラーが発生しますが、</font>**
```
$ realsense-viewer
 10/03 15:11:05,957 INFO [281473265874000] (tm-boot.h:22) Found a T265 to boot
 10/03 15:11:06,971 WARNING [281473265874000] (messenger-libusb.cpp:65) bulk_transfer returned error, endpoint: 0x1, error: Resource temporarily unavailable, err. num: 11
 10/03 15:11:06,972 ERROR [281473265874000] (tm-boot.h:39) Error booting T265
 10/03 15:11:09,770 INFO [281473265874000] (synthetic-stream-gl.cpp:80) Initializing rendering, GLSL=0
 10/03 15:11:09,770 INFO [281473265874000] (synthetic-stream-gl.cpp:89)  0 GPU objects initialized
 10/03 15:11:09,904 INFO [281473265874000] (tm-boot.h:22) Found a T265 to boot
 10/03 15:11:10,918 WARNING [281473265874000] (messenger-libusb.cpp:65) bulk_transfer returned error, endpoint: 0x1, error: Resource temporarily unavailable, err. num: 11
 10/03 15:11:10,918 ERROR [281473265874000] (tm-boot.h:39) Error booting T265
```
**<font color="red">　USB を刺し直しすと解決します。</font>**

<img alt="[realsense-viewer_1" src="image/rv1.JPG" width="90%">

## pyrealsense2 環境構築

### ① pyrealsense2 環境設定
```
$ echo "PYTHONPATH=$PYTHONPATH:/usr/local/lib:/usr/lib/python3/dist-packages/pyrealsense2" >> ~/.bashrc
$ source ~/.bashrc
```

### ② 依存パッケージのインストール
```
$ sudo apt install python3-numpy -y
$ sudo apt install python3-opencv -y
$ sudo apt install python3-pip -y
```

### ③ サンプルプログラムの動作確認

Rasberry Pi の GUI 上のターミナルから、T265 の動作確認をします。
```
$ cd ~/GitHub/librealsense/wrappers/python/examples/
$ python3 t265_example.py

Frame #0
Position: x: 0, y: 0, z: 0
Velocity: x: 0, y: 0, z: 0
Acceleration: x: 0, y: 0, z: 0

　　　　　　　　：

Frame #49
Position: x: -2.32191e-05, y: -3.03128e-05, z: -0.000144963
Velocity: x: -0.000845491, y: 0.00475642, z: 0.00195427
Acceleration: x: -0.00256478, y: 0.0904832, z: 0.0510505
```

## RealSense ROS 環境構築

## ROS Noeticのインストール

[ubuntu20.04LTSにROS Noeticをインストールする](https://qiita.com/porizou1/items/7c2c4b33126cfad05944) を参考に、ROS Noetic をインストールします。

### ① source.list の設定
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### ② キーの設定 
```
$ sudo apt install curl
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

### ③ インストール
```
$ sudo apt update
$ sudo apt install ros-noetic-desktop -y

$ sudo apt install python3-catkin-tools -y
$ sudo apt-get install ros-noetic-imu-tools -y
```

### ④ 環境設定
```
$ echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc

$ sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

$ sudo rosdep init
$ rosdep update
```

### ⑤ ワークスペースを作成
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make

$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

## RealSense ROS のインストール

[ROS Wrapper for Intel RealSense Devices](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy) を参考に、RealSense ROS をインストールします。

### ① 依存パッケージのインストール
```
$ sudo apt update && sudo apt upgrade -y
$ sudo apt install ros-noetic-ddynamic-reconfigure
```

### ② LibrealSense-ros のインストール
```
$ cd ~/catkin_ws/src/

$ git clone https://github.com/IntelRealSense/realsense-ros.git
$ cd realsense-ros/
$ git branch
* ros2-development

$ git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
$ git branch
* (HEAD detached at 2.3.2)
  ros2-development

$ cd ../../
$ catkin_make clean
$ catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
$ catkin_make install

$ source ~/.bashrc
```

### ③ 動作確認

[RealSense D435(距離計測カメラ) & T265(自己位置認識カメラ) をROS上で動かした](https://ogimotokin.hatenablog.com/entry/2019/06/05/222039) を参考に、<br>
Rasberry Pi の GUI 上のターミナルから、T265 の動作確認をします。

１つのターミナルで起動し、
```
$ roslaunch realsense2_camera rs_t265.launch
```
別のターミナルで、自己位置（相対位置）を表示します。
```
$ rostopic echo /tf

transforms:
  -
    header:
      seq: 0
      stamp:
        secs: 1678684106
        nsecs: 349720001
      frame_id: "camera_odom_frame"
    child_frame_id: "camera_pose_frame"
    transform:
      translation:
        x: 2.5222741896868683e-05
        y: 8.745512786845211e-06
        z: -0.0002126105537172407
      rotation:
        x: -0.006261443253606558
        y: 0.011176524683833122
        z: -1.3685310477740131e-05
        w: 0.9999179840087891
---
transforms:
  -
    header:
      seq: 0
      stamp:
        secs: 1678684106
        nsecs: 354726791
      frame_id: "camera_odom_frame"
    child_frame_id: "camera_pose_frame"
    transform:
      translation:
        x: 2.263886017317418e-05
        y: 8.973940566647798e-06
        z: -0.0002068436297122389
      rotation:
        x: -0.006260068155825138
        y: 0.011174202896654606
        z: -1.6503043298143893e-05
        w: 0.9999179840087891
---
```

## ArduPilot 環境構築（non-ROS）

[Intel RealSense T265](https://ardupilot.org/rover/docs/common-vio-tracking-camera.html) を参考に、ArduPilot の環境を構築します。

### ① ArduPilot 設定

* SERIAL2_PROTOCOL = 2 (MAVLink2). Note this assumes the RPI4 is connected to AutoPilot “Telem2” port.
* SERIAL2_BAUD = 921 (921600 baud)
* SERIAL2_OPTIONS = 0 (the default)
* VISO_TYPE = 2 (IntelT265)

* EK3_SRC1_POSXY = 6 (ExternalNav)
* EK3_SRC1_VELXY = 6 (ExternalNav)
* EK3_SRC1_VELZ = 6 (ExternalNav)
* EK3_SRC1_YAW = 6 (ExternalNav)
* EK3_SRC1_POSZ = 1 (Baro which is safer because of the camera’s weakness to high vibrations)

* GPS_TYPE = 0 (disable the GPS)
* COMPASS_USE = 0, COMPASS_USE2 = 0, COMPASS_USE3 = 0 to disable all compasses

## APSync 関連ツールをビルドして環境構築

[Ubuntu Companion](https://github.com/ArduPilot/companion/tree/master/Common/Ubuntu?fbclid=IwAR1EwGJqlMZ6CaEvMsXXn0k0354d2JHQCN0C5cNrXctUxwXqDOKUeH67ZbU) を参考に、APSync 関連ツールをビルドします。

### ① パッケージ等のインストール

以下を参考にパッケージ等をインストールします。
```
$ cd ~/GitHub
$ git clone https://github.com/ArduPilot/companion.git

$ mkdir log
$ sudo ./companion/Common/Ubuntu/set-hostname |& tee log/set-hostname.log
$ sudo ./companion/Common/Ubuntu/install_packages.sh |& tee log/install_packages.log
$ sudo apt install libopencv-dev python3-opencv -y
$ sudo ./companion/Common/Ubuntu/install_niceties |& tee log/install_niceties.log
$ sudo ./companion/Common/Ubuntu/install_avahi |& tee log/install_avahi.log
```
* install_packages.sh 実行時に以下のエラーが発生しますが、別途インストールしているので問題ありません。
  ```
  E: Package 'python-pip' has no installation candidate
  E: Unable to locate package python-opencv
  ```

### ② mavlink-router のインストール

以下を参考に mavlink-router をインストールします。
```
$ sudo apt install git ninja-build pkg-config gcc g++ systemd -y
$ sudo apt install python3-pip -y
$ sudo pip3 install meson

$ cd ~/GitHub
$ git clone --recurse-submodules https://github.com/mavlink-router/mavlink-router.git
$ cd mavlink-router

$ meson setup build .
$ ninja -C build
$ sudo ninja -C build install

$ cp -p ~/GitHub/companion/Common/Ubuntu/mavlink-router/mavlink-router.conf main.conf
$ vi main.conf
$ diff main.conf ~/GitHub/companion/Common/Ubuntu/mavlink-router/mavlink-router.conf
98d97
< #TcpServerPort = 5760
100d98
< Log=/var/log/flight-stack
107c105
< Device = /dev/ttyAMA1
---
> Device = /dev/ttyUSB0

$ sudo mkdir -p /etc/mavlink-router
$ sudo cp -p main.conf /etc/mavlink-router

$ sudo systemctl enable mavlink-router.service
$ sudo systemctl start mavlink-router.service

$ sudo systemctl stop mavlink-router.service
$ sudo systemctl disable mavlink-router.service
$ sudo reboot
```

以下を参考に mavlink-router のLOGファイルが生成されていることを確認をします。
* LOGファイル名は、XXXXX-date-time.bin、XXXXX は通番です。
```
$ ls /var/log/flight-stack
00000-2023-03-14_09-33-03.bin
```

* roslaunch mavros で timeout が発生するため、<br>
  **<font color="red">mavlink-router.service は停止し、必要に応じて起動します。</font>**

  ```
  エラーメッセージ：
  [ WARN] [1679283746.074020083]: GF: timeout, retries left 2
  [ WARN] [1679283746.076230024]: RP: timeout, retries left 2
  [ WARN] [1679283746.078598645]: WP: timeout, retries left 2
  　　　　　　　　：
  [ WARN] [1679283758.190133043]: PR: request param #0 timeout, retries left 2, and 426 params still missing
  [ WARN] [1679283759.689046020]: PR: request param #6 timeout, retries left 2, and 421 params still missing
  　　　　　　　　：
  対策方法：
    mavlink-router.service を停止する。
  対象ファイル：
  　mavlink-router
  ```

### ③ apweb のインストール

以下を参考に apweb をインストールします。
```
$ cd ~/GitHub
$ sudo apt install python3-pip libtalloc-dev -y
$ sudo apt install make
$ sudo apt update
$ sudo apt install python2 -y
$ sudo apt install curl
$ curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py
$ sudo python2 get-pip.py
$ pip2 install future --user
$ sudo ln -s /usr/bin/python2 /usr/bin/python

$ git clone -b video_streaming https://github.com/shortstheory/APWeb.git
$ cd APWeb/modules
$ rmdir mavlink/
$ git clone --recurse-submodules https://github.com/ArduPilot/mavlink.git
$ cd ../
$ make CFLAGS=-Wno-error
$ sudo rm /usr/bin/python

$ APWEB_HOME=~/start_apweb
$ mkdir $APWEB_HOME
$ cp web_server $APWEB_HOME
$ cd $APWEB_HOME
$ vi APWeb.service
　※記述内容は下記を参照して下さい。
$ sudo cp -p APWeb.service /etc/systemd/system/APWeb.service
$ sudo systemctl enable APWeb.service
$ sudo systemctl start APWeb.service
$ sudo reboot
```
* APWeb.service
```
[Unit]
Description=ApWeb Service

[Service]
Type=simple
ExecStart=/home/pi/start_apweb/web_server -p 80 -f 14756
WorkingDirectory=/home/pi/start_apweb
Restart=on-failure
User=root

[Install]
WantedBy=multi-user.target
```
* APWeb/modules/mavlink が --recurse-submodules で clone されないため、<br>
  **<font color="red">mavlink を APWeb/modules の下にマニュアルで clone します。</font>**

  ```
  エラーメッセージ：
  　Submodule 'modules/mavlink' (git://github.com/ArduPilot/mavlink) registered for path 'modules/mavlink'
  　Cloning into '/home/pi/APWeb/modules/mavlink'...
  　fatal: unable to connect to github.com:
  　github.com[0: 20.27.177.113]: errno=Connection timed out
  
  　fatal: clone of 'git://github.com/ArduPilot/mavlink' into submodule path '/home/pi/APWeb/modules/mavlink' failed
  　Failed to clone 'modules/mavlink'. Retry scheduled
  対策方法：
    https://github.com/ArduPilot/mavlink.git を APWeb/modules の下にマニュアルで clone します。
  対象ファイル：
  　mavlink
  ```

Rasberry Pi の Webブラウザで ”http://127.0.0.1” にアクセスして動作確認をします。

<img alt="apweb画面1" src="image/apweb1.png" width="70%">

### ④ pymavlink のインストール

以下を参考に pymavlink をインストールします。
```
$ pip3 install pymavlink
```
### ⑤ wifi_access_point のインストール

以下を参考に NetworkManager をインストールします。
```
$ cd ~/
$ sudo rm /etc/network/interfaces
$ sudo touch /etc/network/interfaces
$ sudo apt-get install -y network-manager

$ sudo systemctl disable networking
$ sudo systemctl stop networking

$ sudo apt-get remove -y modemmanager
$ sudo mv /etc/wpa_supplicant/wpa_supplicant.conf{,-unused}

$ sudo systemctl stop wpa_supplicant
$ sudo systemctl disable wpa_supplicant
$ sudo killall /usr/sbin/wpa_supplicant || true

$ sudo systemctl start NetworkManager
$ sudo systemctl enable NetworkManager
$ sudo reboot
```
* 起動中の環境によって、コマンドが失敗することがあるため、個々に確認が必要です。

以下を参考に wifi_access_point をインストールします。
```
$ cd ~/GitHub
$ sudo ./companion/Common/Ubuntu/3_wifi_access_point.sh |& tee log/3_wifi_access_point.log
$ sudo reboot
```

Rasberry Pi のアクセスポイント "ardupilot" にアクセスして動作確認をします。

### ⑥ MAVProxy のインストール

以下を参考に MAVProxy をインストールします。
```
$ sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame -y
$ pip3 install PyYAML mavproxy --user

$ cat >>$HOME/.mavinit.scr <<EOF2
set moddebug 3
EOF2

$ sudo reboot
```

以下を参考に MAVProxy の起動を確認をします。
```
$ mavproxy.py --master=/dev/ttyAMA1 --baudrate 921600 --aircraft MyRo
ver
Connect /dev/ttyAMA1 source_system=255
Running script (/home/pi/.mavinit.scr)
Running script /home/pi/.mavinit.scr
-> set moddebug 3
no script MyRover/mavinit.scr
Log Directory: MyRover/logs/2023-03-20/flight3
Telemetry log: MyRover/logs/2023-03-20/flight3/flight.tlog
Waiting for heartbeat from /dev/ttyAMA1
MAV> Detected vehicle 1:1 on link 0
online system 1
HOLD> Mode HOLD
fence present
Detected vehicle 1:0 on link 0
AP: ArduRover V4.2.3 (2172cfb3)
AP: ChibiOS: 38022f4f
AP: CubeOrangePlu 0043002D 31395105 3539333
AP: RCOut: PWM:1-14
AP: IMU0: fast sampling enabled 2.0kHz
AP: IMU1: fast sampling enabled 9.0kHz/2.3kHz
AP: IMU2: fast sampling enabled 9.0kHz/2.3kHz
Received 855 parameters (ftp)
Saved 855 parameters to MyRover/logs/2023-03-20/flight3/mav.parm

HOLD> param show ARMING_CHECK
HOLD> ARMING_CHECK     1

HOLD> param set ARMING_CHECK 0
HOLD>
HOLD> arm throttle
HOLD> Got COMMAND_ACK: COMPONENT_ARM_DISARM: ACCEPTED
AP: Warning: Arming Checks Disabled
AP: Throttle armed
ARMED
Arming checks disabled

HOLD> mode GUIDED
HOLD> Got COMMAND_ACK: DO_SET_MODE: ACCEPTED
GUIDED> Mode GUIDED

GUIDED> AP: EKF3 IMU0 forced reset
AP: EKF3 IMU0 initialised

GUIDED> AP: EKF3 IMU0 tilt alignment complete

GUIDED>
```

### ⑦ Python packages インストール
```
# pip install may require sudo, so proceed accordingly
$ pip3 install transformations
$ pip3 install dronekit
$ pip3 install apscheduler

# Install serial packages for serial connection
$ sudo pip3 install pyserial
```

## ArduPilot 動作確認（non-ROS）

[Intel RealSense T265](https://ardupilot.org/rover/docs/common-vio-tracking-camera.html) を参考に、ArduPilot の動作を確認します。

### ① mavlink 動作確認

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/hoangthien94/vision_to_mavros.git

$ cd ~/catkin_ws/src/vision_to_mavros/scripts
$ chmod +x t265_to_mavlink.py
$ cp -p t265_to_mavlink.py t265_to_mavlink.py.bak
$ vi t265_to_mavlink.py
$ diff t265_to_mavlink.py t265_to_mavlink.py.bak
48c48
< connection_string_default = '/dev/ttyAMA1'
---
> connection_string_default = '/dev/ttyUSB0'
50c50
< connection_timeout_sec_default = 3600
---
> connection_timeout_sec_default = 5

$ echo 'PATH=$PATH:~/catkin_ws/src/vision_to_mavros/scripts' >> ~/.bashrc
$ source ~/.bashrc
$ t265_to_mavlink.py
```
結果
```
INFO: Using default connection_string /dev/ttyAMA1
INFO: Using default connection_baudrate 921600
INFO: Using default vision_position_estimate_msg_hz 30.0
INFO: Using default vision_position_delta_msg_hz 30.0
INFO: Using default vision_speed_estimate_msg_hz 30.0
INFO: Using camera position offset: Disabled
INFO: Using compass: Disabled
INFO: Using default scale factor 1.0
INFO: Using default camera orientation 0
INFO: pyrealsense2 version: 2.53.1
INFO: Starting Vehicle communications
INFO: Connecting to camera...
INFO: Camera connected.
INFO: Received first ATTITUDE message with heading yaw 135.64 degrees
INFO: Sending vision messages to FCU
INFO: Tracking confidence: Medium
```
* フレームの取得でタイムアウトが発生する場合があります。
  ```
  エラーメッセージ：
    Frame didn't arrive within 5000
    Closing the script...
    INFO: Realsense pipeline and vehicle object closed.
  対策方法：
    connection_timeout_sec_default = 5 ⇒ 3600
    ※この修正でも、最長 15 分程度で、カメラからの情報が切れてしまう。
  対象ファイル：
  　t265_to_mavlink.py
  ```

### ② stream 動作確認

```
$ echo 'PYTHONPATH=$PYTHONPATH:/usr/local/lib' >> ~/.bashrc
$ source ~/.bashrc
$ python3 t265_test_streams.py
```
結果
```
Found device: Intel RealSense T265 , with serial number:  944222110910
Available streams:
['__class__',
 '__delattr__',
 '__dir__',
 '__doc__',
 '__entries',
 '__eq__',
 '__format__',
 '__ge__',
 '__getattribute__',
 '__getstate__',
 '__gt__',
 '__hash__',
 '__index__',
 '__init__',
 '__init_subclass__',
 '__int__',
 '__le__',
 '__lt__',
 '__members__',
 '__module__',
 '__ne__',
 '__new__',
 '__reduce__',
 '__reduce_ex__',
 '__repr__',
 '__setattr__',
 '__setstate__',
 '__sizeof__',
 '__str__',
 '__subclasshook__',
 'accel',
 'any',
 'color',
 'confidence',
 'depth',
 'fisheye',
 'gpio',
 'gyro',
 'infrared',
 'name',
 'pose',
 'value']
Left frame (800, 848)
Right frame (800, 848)

Frame number:     5
Position xyz: -0.0000  0.0001 -0.0000
Velocity xyz: -0.0003  0.0049 -0.0010
Accelera xyz:  0.0062  0.0703 -0.0018
Quatern xyzw: -0.0110 -0.0002  0.0045  0.9999
Left frame (800, 848)
Right frame (800, 848)

Frame number:     5
Position xyz: -0.0000  0.0001 -0.0000
Velocity xyz: -0.0003  0.0049 -0.0010
Accelera xyz:  0.0062  0.0703 -0.0018
Quatern xyzw: -0.0110 -0.0002  0.0045  0.9999
Left frame (800, 848)
Right frame (800, 848)

Frame number:    11
Position xyz: -0.0000  0.0002 -0.0006
Velocity xyz: -0.0005  0.0070 -0.0092
Accelera xyz:  0.0037  0.1002 -0.0461
Quatern xyzw: -0.0119 -0.0002  0.0046  0.9999
Left frame (800, 848)
Right frame (800, 848)

Frame number:    12
Position xyz: -0.0000  0.0002 -0.0006
Velocity xyz: -0.0005  0.0076 -0.0094
Accelera xyz:  0.0036  0.1065 -0.0450
Quatern xyzw: -0.0119 -0.0002  0.0046  0.9999
Left frame (800, 848)
Right frame (800, 848)
```

### ③ ArduPilot 動作確認

#### (1) Mission Planner を起動

#### (2) t265_to_mavlink.py を起動

#### (3) Mission Planner から以下を確認

* 距離情報の受信確認
  1. Mission Planner で Ctrl+F を押します。
  1. “Mavlink Inspector” をクリックします。
  1. VISION_POSITION_DELTA の値を確認します。
  <img alt="VISION_POSITION_1" src="image/VPD1.png" width="90%">
  * **<font color="red">最長 15 分程度で、カメラからの情報が切れてしまう。</font>**<br>
* リアルタイムの距離確認
  1. Mission Planner で Ctrl+F を押します。
  1. “Proximity” をクリックします。
  1. proximity viewer を確認します。
  <img alt="proximity_viewer_1" src="image/pv1.png" width="60%">

  * <font color="red">距離が表示されない。</font>

## ROS インターフェース 環境構築

### ① MAVROS のインストール

[ROS講座131 ArdupilotとROS経由で接続する](https://qiita.com/srs/items/09d217c8b9f9e21d2f1d) を参考に、MAVROS をインストールします。

```
$ sudo apt install ros-noetic-mavros -y
$ sudo apt install ros-noetic-mavros-extras -y
```

### ② Vision_to_mavros のインストール

[ROS and VIO tracking camera for non-GPS Navigation](https://ardupilot.org/dev/docs/ros-vio-tracking-camera.html) を参考に、<br>
Vision_to_mavros をインストールします。

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/hoangthien94/vision_to_mavros.git
$ cd ../
$ catkin_make

$ source ~/.bashrc
```

## ArduPilot 環境構築（ROS）

[ROS and VIO tracking camera for non-GPS Navigation](https://ardupilot.org/dev/docs/ros-vio-tracking-camera.html) を参考に、ArduPilot の環境を構築します。

### ① ArduPilot 設定

* SERIAL2_PROTOCOL = 2 (MAVLink2). Note this assumes the RPI4 is connected to AutoPilot “Telem2” port.
* SERIAL2_BAUD = 921 (921600 baud)
* SERIAL2_OPTIONS = 0 (the default)
* BRD_SER2_RTS = 2 (the default)
* BRD_RTC_Types = 2 (GPS -> UTC Time)
* VISO_TYPE = 2 (IntelT265)
* AHRS_EKF_TYPE = 3 (the default) to use EKF3
* EK2_ENABLE = 0 (the default)
* EK3_ENABLE = 1 (the default)

* EK3_SRC1_POSXY = 6 (ExternalNav)
* EK3_SRC1_VELXY = 6 (ExternalNav)
* EK3_SRC1_VELZ = 6 (ExternalNav)
* EK3_SRC1_YAW = 6 (ExternalNav)
* EK3_SRC1_POSZ = 1 (Baro which is safer because of the camera’s weakness to high vibrations)

* EK3_GPS_CHECK = 0 to bypass the EKF’s check of the GPS
* EK3_POSNE_M_NSE = 0.1
* EK3_VELD_M_NSE = 0.1
* EK3_VELNE_M_NSE = 0.1
* GPS_TYPE = 0 to disable the GPS
* COMPASS_ENABLE = 0, COMPASS_USE = 0, COMPASS_USE2 = 0, COMPASS_USE3 = 0 to disable the EKF’

### ② ROS 起動

Rasberry Pi で３つのターミナルを開き、ROSを起動します。

ターミナル１
```
$ roslaunch realsense2_camera rs_t265.launch
```

ターミナル２
```
$ roslaunch vision_to_mavros t265_tf_to_mavros.launch
```

ターミナル３
```
$ roslaunch mavros apm.launch fcu_protocol:=v2.0 fcu_url:=/dev/ttyAMA1:921600
```

* [RTT timesync issue after reboot](https://github.com/mavlink/mavros/issues/1339) を参考に、下記ワーニングを対策します。
  ```
  エラーメッセージ：
  　[ WARN] [xxxx]: TM : RTT too high for timesync: 17.76 ms.
  対策方法：
    下記手順で、timesync_rate を 0 に変更します。
    $ cd /opt/ros/noetic/share/mavros/launch/
    $ sudo cp -p apm_config.yaml apm_config.yaml.bak
    $ sudo vi apm_config.yaml
    $ diff apm_config.yaml apm_config.yaml.bak
    2,13c12,13
    <   timeout: 50.0          # heartbeat timeout in seconds
    <   timesync_rate: 0.0    # TIMESYNC rate in Hertz (feature disabled if 0.0)
    ---
    >   timeout: 10.0          # heartbeat timeout in seconds
    >   timesync_rate: 10.0    # TIMESYNC rate in Hertz (feature disabled if 0.0)
  対象ファイル：
  　apm_config.yaml
  ```

ターミナル４
* LOG取得用
```
$ rosrun rqt_console rqt_console
```

### ③ ArduPilot 動作確認

#### (1) Mission Planner を起動

#### (2) ROS を起動

#### (3) Mission Planner から以下を確認

* 距離情報の受信確認
  1. Mission Planner で Ctrl+F を押します。
  1. “Mavlink Inspector” をクリックします。
  1. VISION_POSITION_DELTA の値を確認します。
  <img alt="VISION_POSITION_2" src="image/VPD2.png" width="90%">
  * **<font color="red">最長 1 分程度で、カメラからの情報が切れてしまう。</font>**<br>
    以下は、その際の LOG の例です。（CSVは、rqt_console の出力）
      * [mavros_1.log](./image/mavros_1.log) 
      * [mavros_1.csv](./image/mavros_1.csv) 
      * [mavros_2.log](./image/mavros_2.log) 
      * [mavros_2.csv](./image/mavros_2.csv) 
