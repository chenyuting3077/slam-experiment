# 資料集特性說明

本專案用兩份不同感測器、不同性質的資料集測試同一組四套 SLAM 系統(Cartographer 3D、RTAB-Map、LIO-SAM、FAST-LIO2),目的是看同一套系統在不同 LiDAR 掃描模式與不同軌跡結構下的表現差異。

## 1. Campus(LIO-SAM 官方資料集)

| 項目 | 內容 |
|---|---|
| LiDAR | Velodyne VLP-16(16 線,機械旋轉式,360° 全周) |
| IMU | MicroStrain 3DM-GX5-25 |
| 幀數 / 時長 | 9,865 幀點雲、16.58 分鐘(994.97s) |
| 路徑長度 | ~1420–1441m |
| 軌跡結構 | **閉環**——起點終點是同一個實體地點。獨立用原始 GPS(`/gps/fix`)核對過:起終點 GPS 座標只差 1.76~2.09m,遠小於整條路徑長度,證實迴圈確實閉合 |
| Ground truth | official 無 MoCap/RTK 等級 ground truth,只能用「終點誤差」(首尾姿態歐氏距離,論文 Table II 定義)作為量化指標。GPS 本身有但精度低(建築物旁誤差可達數公尺到十幾公尺),只能當粗略定性參照 |
| 原始格式 | ROS1 bag(`rosbags-convert` 轉成 ROS2 供 Cartographer/RTAB-Map/LIO-SAM(Humble)使用;FAST-LIO2 用官方 ROS1 版直接吃原始 bag,不用轉檔) |
| 環境 | 校園步道,地形近乎平面,兩側建築物、樹木 |

VLP-16 是傳統機械旋轉多線 LiDAR,每個 PointCloud2 訊息就是一次完整 360° 旋轉,點雲天生按「線束」(ring)組織,四套系統原生都認得這種資料格式,幾乎不用改動。

## 2. Mid360 outdoor_hard_01(Zenodo「Hard Point Cloud Localization Dataset」)

| 項目 | 內容 |
|---|---|
| LiDAR | Livox Mid360(固態,非重複掃描,垂直 FOV −7°~52°,水平 360°) |
| IMU | Mid360 內建 IMU(~200Hz),跟 LiDAR 同一個 `livox_frame`,無外部標定需求 |
| 幀數 / 時長 | 5,147 幀點雲(~7.5Hz)、683.66 秒(兩段 rosbag `outdoor_hard_01a`+`01b` 合併而成,原始切檔只是錄製檔案輪替,首尾時間差 <1 秒,不是真的中斷) |
| 路徑長度 | ~998.9m |
| 軌跡結構 | **非閉環**——ground truth 顯示起點終點距離 18.4m(相對於 998.9m 全長路徑,不算閉合),不能沿用 Campus 那種「終點誤差」評估法 |
| Ground truth | 資料集**直接附完整連續軌跡**(5,147 筆,TUM 格式,`gt/traj_lidar_outdoor_hard_01.txt`),可以做真正的 ATE(絕對軌跡誤差)評估,比 Campus 只有頭尾兩點嚴謹很多 |
| 原始格式 | ROS2 bag(rosbag2/db3),點雲用 `livox_ros_driver2/CustomMsg` 跟已轉好的 `sensor_msgs/PointCloud2` 兩種格式都有 |
| 環境 | 戶外,官方資料集說明特別標示為 "hard" 序列:快速感測器運動、點雲品質劣化,是專門設計來考驗定位穩健性的困難場景,不是隨手錄的日常資料 |

### PointCloud2 欄位

`/livox/points` 的欄位是 `x, y, z, t, intensity, tag, line`——`line` 是 Mid360 版本的「掃描線編號」(概念上類似 VLP-16 的 `ring`,但 Mid360 本質上沒有真正固定的掃描線,是演算法端另外算出來的近似值),`t` 是**該點相對於這個封包起始時間的偏移量,單位是「奈秒」的 uint32**(不是 Velodyne 慣例的「秒」float)——這個單位差異是這次讓 LIO-SAM 移植過程中出現「時間戳解析錯誤」的根本原因(詳見下方)。

### 四套系統移植上的差異(相對於 Campus)

| 系統 | 移植狀態 | 最終跑測結果 |
|---|---|---|
| Cartographer 3D | 設定調整後可啟動(調整 `tracking_frame`/`published_frame` 為 `livox_frame`,縮小 `min_range`/`max_range`) | ❌ **實測發現真的會發散**:即時播放下 `odom->livox_frame` 的姿態外推器啟動後很快開始二次方式發散(幾分鐘內飄到數千萬公尺),但內部 pose graph/submap 日誌完全沒有報錯,難以事先察覺。嘗試調整 `imu_gravity_time_constant`(10s→1s)沒解決,反而讓 submap 完全插入不了,已還原成預設值,列為已知限制,沒有可用軌跡 |
| RTAB-Map(純 ICP) | 直接可用,拿 Campus 的設定改 topic/frame 即可 | ⚠️ **只成功追蹤 17% 的路徑**(170.95m / 998.9m)就停止累積新節點,跟 log 裡觀察到的 ICP 註冊間歇性失敗(`libpointmatcher` 找不到足夠配對點)一致,推測跟資料集本身設計的「快速運動、點雲劣化」情境直接相關。ATE RMSE 只有 0.944m,但那只是「一小段跑得準」,不是「全程穩健」 |
| LIO-SAM | 官方 `ros2` 分支不支援 Livox 非重複掃描模式,改用社群 fork [`rajvishnu07/lio_sam_mid360`](https://github.com/rajvishnu07/lio_sam_mid360)(ROS2 Humble)。修了兩個 bug:①`lidarFrame == baselinkFrame` 時,程式碼發布的靜態轉換 `frame_id` 是空字串,導致 TF 一直報 `TF_NO_FRAME_ID`,改成 `lidarFrame="livox_frame"`、`baselinkFrame="base_link"` 兩個不同名稱並補上真正的靜態轉換解決;②原始 fork 把 PointCloud2 的 `t`(uint32 奈秒)欄位直接用型別不匹配的方式塞進 PCL 點結構的 `float time`(預期單位是秒)成員,等同把奈秒的整數位元組原始重新解讀成 float,得到的是完全錯誤的雜訊值——修法是仿造同一支程式碼裡處理 Ouster `uint32_t t` 欄位的既有寫法(`dst.time = src.t * 1e-9f`),先保留原始型別再做正確的數值轉換 | ⚠️ **演算法驗證正確,但沒能匯出完整軌跡檔**:修完上述兩個 bug 後,多次即時播放期間直接 `ros2 topic echo` 過 `/lio_sam/mapping/odometry`,確認姿態合理(例如 x=-63.98, y=0.94, z=1.16)。但這次 session 背景管理 process 疏失——曾經同時跑了兩個重疊的 `ros2 bag play`,互相汙染訊息流——導致完整錄製反覆失敗,最後決定不再花時間重跑,誠實記錄成「有效但沒有可匯出的完整軌跡檔」,而不是掩蓋這個 session 本身的操作問題 |
| FAST-LIO2 | 官方 `hku-mars/FAST_LIO` 本身就附 `config/mid360.yaml`,但 `lidar_type=1`(Livox)的訂閱寫死只吃 `livox_ros_driver::CustomMsg`,不接受 PointCloud2。沒有修改 FAST_LIO2 原始碼(維持跟 Campus 一樣「完全official、未修改」的可信度),而是寫了一支轉檔腳本(`mid360_ros2_to_ros1_customsg.py`),把 ROS2 bag 的 `/livox/points`(PointCloud2)還原成真正的 `livox_ros_driver/CustomMsg` 格式(欄位對應到 `offset_time`/`x`/`y`/`z`/`reflectivity`/`tag`/`line`,跟 livox_ros_driver2 自己產生 PointCloud2 時的邏輯完全對應),寫成 ROS1 bag 餵給既有的 FAST-LIO2 官方 ROS1 容器 | ✅ **完整跑完 97% 的路徑長度**(968.41m / 998.9m),ATE RMSE=5.228m。沒有迴環偵測,純里程計在這段「hard」序列上累積了明顯漂移,但至少完整覆蓋了整條路線,跟 RTAB-Map「準但只跑一小段」形成鮮明對比 |

**方法論教訓**:RTAB-Map 的 ATE(0.944m)比 FAST-LIO2(5.228m)漂亮,但只是因為它提早停止追蹤;只看單一數字會誤判「RTAB-Map 比較準」,必須同時檢查軌跡覆蓋的路徑長度佔比,才能判斷這個數字代表的是「全程穩健的精度」還是「一小段運氣好的精度」。

### 為什麼特地找一份「非閉環、有完整 ground truth、非重複掃描 LiDAR」的第二資料集

Campus 資料集的評估方法(終點誤差)有兩個先天限制:①只看頭尾兩點,中間軌跡飄多遠、飄去哪個方向完全看不到;②所有系統都是同一顆 VLP-16、同一種機械旋轉掃描模式,沒辦法看出「換一顆完全不同掃描原理的 LiDAR(固態、非重複)」時,同一套演算法的相容性跟強健性差異。Mid360 資料集正好補上這兩塊:完整連續 ground truth 可以做真正的 ATE,而且官方資料集本身就是為了測試「困難場景下的穩健性」設計的,兩份資料集合起來看,才能同時驗證「同一種掃描模式下、不同演算法緊耦合程度」跟「同一組演算法、換一種掃描模式跟資料品質」這兩個獨立維度的差異。
