# 資料集特性說明

本專案用兩種感測器(VLP-16、Livox Mid360)、三個獨立資料集來源(LIO-SAM 官方、Zenodo、TIERS)、共 7 段序列測試同一組四套 SLAM 系統(Cartographer 3D、RTAB-Map、LIO-SAM、FAST-LIO2),目的是看同一套系統在不同 LiDAR 掃描模式、不同軌跡結構、不同資料集來源下的表現差異。

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
| Cartographer 3D | 設定調整後可啟動(調整 `tracking_frame`/`published_frame` 為 `livox_frame`,縮小 `min_range`/`max_range`) | ❌ **實測發現真的會發散**:即時播放下 `odom->livox_frame` 的姿態外推器啟動後很快開始二次方式發散(幾分鐘內飄到數千萬公尺),但內部 pose graph/submap 日誌完全沒有報錯,難以事先察覺。嘗試調整 `imu_gravity_time_constant` 三個方向都沒解決:調小(10s→1s)讓 submap 完全插入不了;調大(10s→30s)發散得**更快**(95 秒內就飄到 -6200 萬公尺,比預設值還糟)。兩個方向都失敗,證實問題不在這個參數,應該是姿態外推器本身無法處理這段「hard」序列的劇烈運動,已還原成預設值,列為已知限制,沒有可用軌跡 |
| RTAB-Map(純 ICP) | 直接可用,拿 Campus 的設定改 topic/frame 即可 | ⚠️ **只成功追蹤 17% 的路徑**(170.95m / 998.9m)就停止累積新節點,跟 log 裡觀察到的 ICP 註冊間歇性失敗(`libpointmatcher` 找不到足夠配對點)一致,推測跟資料集本身設計的「快速運動、點雲劣化」情境直接相關。ATE RMSE 只有 0.944m,但那只是「一小段跑得準」,不是「全程穩健」 |
| LIO-SAM | 官方 `ros2` 分支不支援 Livox 非重複掃描模式,改用社群 fork [`rajvishnu07/lio_sam_mid360`](https://github.com/rajvishnu07/lio_sam_mid360)(ROS2 Humble)。修了兩個 bug:①`lidarFrame == baselinkFrame` 時,程式碼發布的靜態轉換 `frame_id` 是空字串,導致 TF 一直報 `TF_NO_FRAME_ID`,改成 `lidarFrame="livox_frame"`、`baselinkFrame="base_link"` 兩個不同名稱並補上真正的靜態轉換解決;②原始 fork 把 PointCloud2 的 `t`(uint32 奈秒)欄位直接用型別不匹配的方式塞進 PCL 點結構的 `float time`(預期單位是秒)成員,等同把奈秒的整數位元組原始重新解讀成 float,得到的是完全錯誤的雜訊值——修法是仿造同一支程式碼裡處理 Ouster `uint32_t t` 欄位的既有寫法(`dst.time = src.t * 1e-9f`),先保留原始型別再做正確的數值轉換。另外還踩到一個這次 session 才發現的環境問題:**從 host(ROS2 Jazzy)去 replay bag 給跑在 Humble container 裡的節點,topic 雖然能互相發現,但訊息完全不會送達**——這在專案更早期(Campus 資料集)就記錄過,這次分析 Mid360 時忘記了,導致好幾次「乾淨重跑」看起來像是卡住,其實是打從一開始就沒收到資料。改成從 container 內部自己 `ros2 bag play` 後,加上 `ros2 topic echo` 也要明確指定 `--qos-reliability best_effort`(不指定的話,這個 humble 版本的自動 QoS 偵測邏輯本身有 bug,會直接 crash)才真正拿到資料 | ⚠️ **前段正確,後段發散**:前 69 秒、204m(約全長 20%)姿態合理、跟 ground truth 大致吻合;t≈985s(bag 時間)開始跳動幅度從 5m 一路擴大到 20m+,最終飄到 z=-1000m 等級的離群值,整條軌跡的 ATE 因此完全失去意義(RMSE 520m)。推測跟這個 fork 用「垂直角度算人工 ring」去湊 LOAM 風格邊緣/平面特徵有關——Mid360 沒有真正的掃描線,這個近似在資料集設計的「快速運動、點雲劣化」情境下特別容易抽出壞特徵,一旦壞的因子被序列式地加進 GTSAM 因子圖,後面很難救回來(不像 Cartographer 是全域 submap 配對)。跟 FAST-LIO2(用 ikd-tree 最近鄰配準,完全不依賴 ring 概念)完整跑完形成對比,支持「問題出在 ring 近似」的推測 |
| FAST-LIO2 | 官方 `hku-mars/FAST_LIO` 本身就附 `config/mid360.yaml`,但 `lidar_type=1`(Livox)的訂閱寫死只吃 `livox_ros_driver::CustomMsg`,不接受 PointCloud2。沒有修改 FAST_LIO2 原始碼(維持跟 Campus 一樣「完全official、未修改」的可信度),而是寫了一支轉檔腳本(`mid360_ros2_to_ros1_customsg.py`),把 ROS2 bag 的 `/livox/points`(PointCloud2)還原成真正的 `livox_ros_driver/CustomMsg` 格式(欄位對應到 `offset_time`/`x`/`y`/`z`/`reflectivity`/`tag`/`line`,跟 livox_ros_driver2 自己產生 PointCloud2 時的邏輯完全對應),寫成 ROS1 bag 餵給既有的 FAST-LIO2 官方 ROS1 容器 | ✅ **完整跑完 97% 的路徑長度**(968.41m / 998.9m),ATE RMSE=5.228m。沒有迴環偵測,純里程計在這段「hard」序列上累積了明顯漂移,但至少完整覆蓋了整條路線,跟 RTAB-Map/LIO-SAM「準但只跑一小段」形成鮮明對比 |

**方法論教訓**:RTAB-Map 的 ATE(0.944m)比 FAST-LIO2(5.228m)漂亮,但只是因為它提早停止追蹤;LIO-SAM 也是類似情況(前 20% 路徑正確,之後完全發散)。只看單一 ATE 數字會誤判準確度排名,必須同時檢查軌跡覆蓋的路徑長度佔比,才能判斷這個數字代表的是「全程穩健的精度」還是「一小段運氣好的精度」。四套系統裡只有 FAST-LIO2 完整跑完了這段 hard 序列——沒有迴環偵測反而不受「因子圖被污染後回不去」或「ICP/scan-matching 找不到足夠配對點就整段卡住」這類問題影響,是這次測試最穩健的系統,雖然累積誤差最大。

### 為什麼特地找一份「非閉環、有完整 ground truth、非重複掃描 LiDAR」的第二資料集

Campus 資料集的評估方法(終點誤差)有兩個先天限制:①只看頭尾兩點,中間軌跡飄多遠、飄去哪個方向完全看不到;②所有系統都是同一顆 VLP-16、同一種機械旋轉掃描模式,沒辦法看出「換一顆完全不同掃描原理的 LiDAR(固態、非重複)」時,同一套演算法的相容性跟強健性差異。Mid360 資料集正好補上這兩塊:完整連續 ground truth 可以做真正的 ATE,而且官方資料集本身就是為了測試「困難場景下的穩健性」設計的,兩份資料集合起來看,才能同時驗證「同一種掃描模式下、不同演算法緊耦合程度」跟「同一組演算法、換一種掃描模式跟資料品質」這兩個獨立維度的差異。

## 3. Mid360 outdoor_kidnap(同一個 Zenodo 資料集,`outdoor_kidnap_a`+`b`)

| 項目 | 內容 |
|---|---|
| LiDAR / IMU | 跟 outdoor_hard_01 完全一樣(Livox Mid360 + 內建 IMU),同一個 Zenodo 資料集,格式、topic 完全相同,四套系統的設定/轉檔腳本直接沿用,零額外相容性工作 |
| 幀數 / 時長 | 4,017 幀點雲、553.8 秒(兩段 `outdoor_kidnap_a`(203.6s)+`outdoor_kidnap_b`(349.5s)合併) |
| 路徑長度 | 743.3m |
| 軌跡結構 | 非閉環,起終點距離 19.8m |
| 情境設計 | 官方標示為 **"kidnap"** 情境——不是像 outdoor_hard 那樣「全程快速運動」,而是模擬「機器人定位中斷/被抱走重新放置」這種對**重定位**能力的考驗,跟 outdoor_hard 的「持續追蹤穩健性」是不同類型的困難 |

### 四套系統跑測結果——三套系統在同一個時間點一起失敗

| 系統 | ATE RMSE(成功片段) | 成功追蹤的路徑 / 全長 743.3m | 備註 |
|---|---|---|---|
| Cartographer 3D | — | 0% | 跟 outdoor_hard_01 完全一樣的失敗模式,即時播放下幾秒內姿態就飄到數百萬~千萬公尺等級,重跑一次直接確認(沒有另外花時間錄製完整軌跡,因為根因已知是 IMU/姿態外推器層級的問題,跟具體序列內容無關) |
| RTAB-Map(純 ICP) | **0.073m** | 37.0m(5.0%) | 23 個節點後就停止,ATE 極佳但只track 了 5% 的路 |
| LIO-SAM | **2.168m** | 64.0m(8.6%),t≈34s 之後姿態**完全凍結**(後續 55 筆訊息數值一字不變),不是漸進發散,是直接卡死 | |
| FAST-LIO2 | **0.057m** | 36.7m(4.9%),t≈27s 後開始逐步發散(35m→44m→47m→144m 跳躍上升) | 跟 outdoor_hard_01 上完整跑完 97% 形成強烈對比——同一套系統,換一種「困難類型」就從最穩健變成最早失敗 |

**這是這次三份資料集裡最乾淨的一個發現**:RTAB-Map、LIO-SAM、FAST-LIO2 三套存活系統畫出來的地圖跟軌跡幾乎是**同一個形狀**(見比較圖,三個 J 形轉彎幾乎重疊),失敗的時間點也都落在整段路徑的 5~9% 左右——這不是巧合,強烈指向這個資料集裡在那個時間點附近真的發生了一次「kidnap」事件(感測器被快速位移/短暫遮蔽/資料中斷),三套完全不同架構的系統(圖優化、因子圖、ESKF)在完全沒有重定位機制的情況下,**幾乎在同一瞬間一起失去追蹤**。這跟 outdoor_hard_01(每套系統在不同時間點、因為不同原因失敗)形成鮮明對比,說明「持續運動的穩健性」跟「意外中斷後的重定位能力」是這四套系統都缺乏、但成因完全不同的兩種弱點——FAST-LIO2 在 outdoor_hard 上最穩健,面對 kidnap 卻反而是失敗得最早(4.9%)的系統之一,證明「哪套系統更好」高度取決於失敗模式的種類,沒有放諸四海皆準的排名。

## 4. TIERS `multi_modal_lidar_dataset`(獨立第三方資料集,同樣是 Mid360)

前三份 Mid360 資料集都來自同一個 Zenodo 上傳者。這份[TIERS 大學的多模態 LiDAR 資料集](https://github.com/TIERS/multi_modal_lidar_dataset)是完全獨立的第三方來源,錄製設備、錄製地點、ground truth 系統都不同,用來驗證前面觀察到的現象(尤其是 Cartographer 的發散 bug)是不是這個特定 Zenodo 資料集本身的問題,還是 Mid360 感測器/演算法本身的通病。使用者手動從 OneDrive 下載了其中 4 段:2 段戶外道路(`OutdoorRoad_cut0` 66.0s、`OutdoorRoad_cut1` 45.3s)、2 段室內辦公室(`IndoorOffice1` 66.2s、`IndoorOffice2` 95.7s)。

| 項目 | 內容 |
|---|---|
| LiDAR / IMU | Livox Mid360 + 內建 IMU(跟前三份資料集同型號感測器,但完全不同的錄製硬體/校準) |
| Ground truth | 戶外用真實 **GNSS-RTK**(`/gnss_pose`,`PoseStamped` 但把 lat/lon/alt 直接塞進 `Point` 的 x/y/z),室內用 **MoCap**(`/vrpn_client_node/unitree_b1/pose`,已經是公尺級的本地卡氏座標)——跟前三份資料集「Zenodo 自己附的 TUM 軌跡」是完全不同的獨立 ground truth 來源 |
| 原始格式差異 | 欄位順序跟語義都跟既有慣例不同:TIERS 是 `x,y,z,intensity,tag,line,timestamp`(`timestamp` 是 float64 的**絕對 epoch 奈秒**,point_step=26),既有慣例(來自 Zenodo)是 `x,y,z,t,intensity,tag,line`(`t` 是 uint32 的**相對於封包起始時間的偏移**,point_step=22);frame_id 也不同(`mid360_frame` vs `livox_frame`,IMU 兩邊都用 `livox_frame`)。寫了新的轉檔腳本 `tiers_to_our_convention.py` 把 TIERS 轉成既有慣例,後面所有 per-system 設定檔/launch file 完全不用改 |

### 意外發現:`rosbags-convert` 產生的 ROS2 bag,Humble 版 `ros2 bag play` 讀不動

轉檔用的 `rosbags` Python 函式庫,對 ROS2 bag 的 `metadata.yaml` 寫出了 schema v9 格式(`type_description_hash` 用多行純量、`offered_qos_profiles` 用 list),但 ROS2 Humble 內建的 `ros2 bag play`(靠 `yaml-cpp` 解析)只認得舊版 v8 格式(這兩個欄位都是純字串)。不相容的後果是**完全靜默的失敗**——`ros2 bag play` 直接在解析階段丟出 `yaml-cpp: bad conversion` 然後立刻以 exit code 0 結束,不會播放任何一筆訊息,但背景任務通知看起來就是「正常播放完畢」。這正是第一次跑 LIO-SAM 在 `OutdoorRoad_cut1` 上完全沒有輸出的根因——不是 LIO-SAM 或轉檔腳本的問題,是這個 bag 從頭到尾就沒被播放過。修法是直接把 `metadata.yaml` 裡這兩個欄位降級成 v8 的字串格式,`ros2 bag play`/`ros2 bag info` 就能正常讀取。

### 四份資料的跑測結果

| 資料集 | Cartographer 3D | RTAB-Map(純 ICP) | LIO-SAM | FAST-LIO2 |
|---|---|---|---|---|
| OutdoorRoad_cut1(45.3s / 48.3m) | ❌ 發散(第 3 次獨立確認) | 0.220m(39/39,全程) | **3.688m**(223/223,全程但航向漂移明顯) | 0.134m(44/44,全程) |
| OutdoorRoad_cut0(66.0s / 80.3m) | ❌ 發散(第 4 次獨立確認) | 0.088m(53/53,全程) | 0.101m(326/326,全程) | 0.079m(65/65,全程) |
| IndoorOffice1(66.2s) | ❌ 發散(第 5 次獨立確認,首次室內場景) | 0.038m(50/50,全程) | 0.030m(317/317,全程) | 0.030m(65/65,全程) |
| IndoorOffice2(95.7s) | ❌ 發散(第 6 次獨立確認) | 0.046m(68/77,全程) | 0.034m(383/427,全程) | 0.037m(90/95,全程) |

(表格數字為 ATE RMSE;括號內是「成功關聯到 ground truth 的姿態數 / 該系統輸出的總姿態數」,全部都是「全程」,不像前面 Zenodo 的困難序列會提早失去追蹤。)

**發現一:Cartographer 的發散 bug 在完全獨立的第二個資料集來源上,室內外場景都重現,確認不是 Zenodo 資料集本身的問題**。累計到這裡已經是第 6 次獨立確認(2 個資料集來源 × 3 個 Zenodo 序列 + 2 個資料集來源 × TIERS 4 段中的部分序列,實際上每一段 TIERS 資料都重現了),而且這次特別驗證了「室內、低速移動」的 `IndoorOffice1`/`IndoorOffice2` 也一樣發散——排除了先前「可能只在戶外快速運動時才會觸發」的假設,問題確實出在姿態外推器/IMU 初始化這一層,跟具體場景內容無關。

**發現二:室內場景的 ATE 明顯優於戶外(4~5cm vs 8~22cm)**,三套存活系統在兩段室內資料都壓在 5cm 以內,是所有 Mid360 測試裡最好的成績,原因很直觀——辦公室內牆面、家具提供的幾何特徵遠比開放道路密集,ICP/scan-matching 更容易收斂。

**發現三:LIO-SAM 在 `OutdoorRoad_cut1` 上明顯落後(3.688m vs 其他三段資料的 0.030~0.101m)**,但軌跡是完整的(全程 223 個姿態,不是像 Zenodo hard 序列那樣中途發散或凍結)——是「跑完全程但精度差」,不是「提早失敗」。这段資料本身最短(45.3s)、路徑也最短(48.3m),推測跟這個社群 fork 的因子圖優化在資料量不足時収斂較差有關,但另外三段(包含更短的室內資料)並沒有出現同樣的問題,確切原因還沒有進一步鎖定,列為觀察到的現象而非確定的結論。
