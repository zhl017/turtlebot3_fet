# TurtleBot3 Customization : FET  

> 馬達 ： XM430-W210-T | linear : 0.40 | angular : 2.0

## 快速安裝手冊

### 1. 環境設定
為了測試客製化系列的模型（教學以FET稱呼），需要下列一些配置。

#### 1.1. PC安裝

請遵照TurtleBot3官方電子手冊「[快速入門指南](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)」進行安裝並留意下列步驟更改事項。

- [3.1. PC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)，目前僅於ROS Noetic版本運行，請於「[快速入門指南](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)」中上排工具列選擇「**Noetic分支**」進行安裝。

- 更改步驟 [3.1.4. Install TurtelBot3 Packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#install-turtlebot3-packages)

安裝完ROS Noetic後，請輸入下列指令安裝FET相關ROS packages。
```code
$ sudo apt remove ros-noetic-turltebot3-msgs
$ sudo apt remove ros-noetic-turtlebot3
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/zhl017/turtlebot3_fet
$ git clone https://github.com/zhl017/turtlebot3_msgs_idminer_custom
$ cd ~/catkin_ws && catkin_make
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

#### 1.2. SBC安裝（若出廠已安裝好系統可以不用再次安裝SBC)
> user id  : ubuntu   |   password : turtlebot

- [3.2. SBC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup)，FET使用Raspberry Pi 4B作為車體主機，目前僅於ROS Noetic版本運行，請於「[快速入門指南](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)」中上排工具列選擇「**Noetic分支**」進行安裝。

安裝完映像檔後，開啟電源進入系統後，請輸入下列指令安裝FET相關ROS packages。
```code
$ sudo apt remove ros-noetic-turltebot3-msgs
$ sudo apt remove ros-noetic-turtlebot3
$ sudo rm -r catkin_ws
$ mkdir -p ~/caktin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/zhl017/turtlebot3_fet
$ cd ~/catkin_ws/src/turtlebot3_fet
$ sudo rm -r fet_description/ fet_teleop/ fet_navigation/ fet_slam/ fet_example/
$ git clone https://github.com/zhl017/turtlebot3_msgs_idminer_custom
$ cd ~/catkin_ws && catkin_make -j1
$ source ~/.bashrc
```

#### 1.3. OpenCR安裝

- 待更新...

### 2. 網路設定

第一次連接可透過**wifi連接**或是**網路線連接**來遠端進入FET中進行網路設定。

#### 2.1. wifi連接
我們使用Raspberry Pi 作為WiFi熱點。請使用PC進行連接。
> ssid : **TurtleBot_FET**  
> password : **turtlebot**

1. PC連接後，檢查IP位址。
    ```
    $ ifconfig
    ```
    可以看到類似 ```10.42.0.XXX``` 的IP型態。

2. 遠端進入FET並輸入密碼turtlebot。
    ```
    $ ssh ubuntu@10.42.0.1
    ```

#### 2.2. 網路線連接
我們設定Raspberry Pi網路孔固定IP，請使用網路線與PC進行連接。
> IP : **192.168.123.1**

1. PC連接後進入網路設定修改IP選項ipv4修改為手動設定IP。
    ```
    address : 192.168.123.2
    netmask : 255.255.255.0
    gateway : 192.168.123.255
    ```
    
2. 檢查IP位址。
    ```
    $ ifconfig
    ```
    確認是否有看到 ```192.168.123.2``` 的IP型態。

3. 遠端進入FET並輸入密碼turtlebot。
    ```
    $ ssh ubuntu@192.168.123.1

#### 2.3. 設定FET連接wifi環境
1. 開啟wifi設定檔案。
    ```
    sudo nano /etc/netplan/50-cloud-init.yaml
    ````

2. 輸入想連接的wifi裝置與密碼，請參考[官方手冊](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#configure-the-wifi-network-setting-1)。

3. 確認修改完畢後使用快捷鍵 ```ctrl+s``` 儲存以及快捷鍵 ```ctrl+x``` 離開。

4. 輸入指令重新載入配置。
    ```
    $ sudo netplan apply
    ```
    
#### 2.4. 設定bashrc檔案
確認PC與FET連接到相同的wifi環境底下並確認各自的IP位址。

1. 修改「~/.bashrc」檔案。
    ```
    $ nano ~/.bashrc
    ```
    透過使用快捷鍵 ```alt+/``` 幫助您移動到文件最底部。並寫下下列訊息

    1.1 PC
      ```
      export ROS_MASTER_URI=http://PC的IP:11311
      export ROS_HOSTNAME=PC的IP
      ```

    1.2 SBC
      ```
      export ROS_MASTER_URI=http://PC的IP:11311
      export ROS_HOSTNAME=SBC的IP
      ```
   
    
3. 確認修改完畢後使用快捷鍵 ```ctrl+s``` 儲存以及快捷鍵 ```ctrl+x``` 離開。

4. 最後，輸入指令重新載入配置。
    ```
    $ source ~/.bashrc
    ```

## 如何運作
- **開機**
1. 於**PC端**，執行ROS Master。
```
roscore
```

2. 於**PC端**，使用指令遠端至FET。
> default password : **turtlebot**
```
$ ssh ubuntu@ip_address
$ roslaunch fet_bringup fet_robot.launch
```

- **基本遙控**
1. 於**PC端**，執行遙控範例。
```
$ roslaunch fet_teleop fet_teleop_key.launch
```

- **SLAM (mapping)**
1. 於**PC端**，執行SLAM建圖。
```
$ roslaunch fet_slam fet_slam.launch
```

- **Navigation**
1. 於**PC端**，執行Navigation。
```
$ roslaunch fet_navigation fet_navigation.launch
```
  
其他相關應用請參閱 [官方電子手冊](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)。
