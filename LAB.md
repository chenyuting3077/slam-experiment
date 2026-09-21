# 實驗室資料集 SLAM 比較(Compal AMR)

這份文件記錄用同一組 SLAM 系統(Cartographer 3D、RTAB-Map、LIO-SAM、FAST-LIO2)在**實驗室內部錄製的 Compal AMR 資料集**上的測試結果。跟 [README.md](README.md)/[DATASETS.md](DATASETS.md) 記錄的 11 段公開資料集不同,這裡的資料集不是公開來源,所以獨立成這份文件,不計入主要比較清單。

## spot_cartographer 跟官方 Cartographer 的差異

在跑資料集之前,先確認了 [`compal_amr_allen/amr_slam`](https://github.com/chenyuting3077/compal_amr_allen/tree/main/amr_slam) 裡的 `spot_cartographer` 套件跟官方 [`cartographer-project/cartographer`](https://github.com/cartographer-project/cartographer) 的關係:

- **不是原始碼 fork**。`spot_cartographer` 的 `package.xml` 只有 `exec_depend: cartographer_ros`,`CMakeLists.txt` 也只安裝 launch file、Lua 設定檔、rviz 設定——沒有任何 vendored 或修改過的 C++ 原始碼。實際執行時呼叫的是系統 apt 套件 `ros-jazzy-cartographer`/`ros-jazzy-cartographer-ros` 裡的 `cartographer_node`/`cartographer_offline_node`/`cartographer_assets_writer`,跟這個專案其他資料集用的是同一份官方編譯產物,原始碼層級的行為(包含 [CARTOGRAPHER_FAILURE.md](CARTOGRAPHER_FAILURE.md) 記錄的 `imu_tracker.cc` 重力斷言崩潰/靜默發散)完全一致。
- 官方 `cartographer-project` 從來沒有正式支援過 ROS2——`cartographer_ros` 一直是獨立、ROS1 時代的套件。`spot_cartographer` 的 launch file 帶有 `Copyright 2022 Wyca Robotics(for the ros2 conversion)` 標頭,來自社群的 ROS2 移植(`ros-jazzy-cartographer-ros` apt 套件實際上也是編譯自這個移植),不是官方 GitHub 組織下的程式碼。
- 真正的差異只在 Lua 參數調校跟 topic remap:`tracking_frame`/`published_frame` 設成這台機器人的座標系(`imu`/`body`)、`use_odometry = true` 接到 `/odometry`、Ceres/pose graph 權重跟 `num_range_data` 依情境(`spot_3d.lua` 線上 vs. `spot_offline_mapping.lua`/`go2_offline_mapping.lua` 離線)有不同的預設值。這些調整都沒有碰到 `imu_gravity_time_constant` 或 `ImuTracker` 本身,所以理論上前面資料集發現的發散/崩潰風險應該還是存在。

## 資料集一:rosbag2_mapping_dataset(Compal AMR,Vanjee 3D 光達,41.2 秒)

雙顆 Vanjee 3D 光達(前/後已經在錄製端融合成單一 topic `/vanjee_points719e_merged`)、真實 IMU(`/imu/data`)、真實融合里程計(`/odometry`/`/odometry_fused`),41.2 秒室內短距離移動。這份資料**沒有 ground truth**,所以沒辦法算 ATE——改用「四套系統各自獨立建置、獨立跑出來的地圖/軌跡形狀是否一致」互相驗證正確性,只用 `/vanjee_points719e_merged`(忽略未合併的 `_front`/`_rear`)跟 `/odometry`。

| 系統 | 狀態 | 路徑長度 | 姿態數 | 備註 |
|---|---|---|---|---|
| Cartographer 3D | ✅ 成功 | 11.15m | 272 | 用 `compal_amr_allen` 自己的 `spot_cartographer`(`offline_3d.launch.py` + 預設 `spot_offline_mapping.lua`)。是這個專案 12 份資料集歷史中,Cartographer 3D **第一次**兩種已知失敗模式(斷言崩潰、姿態外推器靜默發散)都沒有觸發 |
| RTAB-Map | ✅ 成功 | 10.89m | 31 | 跳過內建的 `icp_odometry`,讓 `rtabmap` 主節點直接訂閱 `/odometry` 當定位骨幹,`scan_cloud` 只用來做幾何迴環偵測跟建圖。過程中修了兩個問題:QoS 預設 BEST_EFFORT 跟 bag 錄製時的 RELIABLE 對不上,同步器完全不觸發;預設把問題當完整 6DoF 優化,在只在平面上動的 AMR 資料上讓 GTSAM 丟出「欠約束」例外,加上 `Reg/Force3DoF=true` 解決。官方的 `rtabmap-export --cloud --scan` 對這個資料庫一直匯出 0 點,原因沒有完全查清,改用「每個關鍵幀姿態配對時間戳最近的原始掃描」手動組出等效點雲 |
| FAST-LIO2(官方,未修改原始碼) | ✅ 成功 | 11.93m | 815 | 踩到預先設想到的雷:合併點雲的 `timestamp` 欄位是絕對 epoch 秒,不是官方 handler 預期的「相對掃描起始的秒數」,寫轉檔腳本修正欄位單位跟座標(含 NaN 點過濾、`base_link→imu_link` 真實外參)後正常運作。`/odometry` 不適用——FAST-LIO2 是純光達-慣性里程計,架構上沒有外部里程計輸入介面,維持官方未修改行為 |
| LIO-SAM | ❌ 發散 | — | 30(第 17 個關鍵幀後開始跳) | 用社群 ROS2 port `pixwyh/LIO-SAM-ROS2`。追出來的根因:這顆光達實際上只有 **4 條掃描線**(`ring` 全程只有 1~4 四種值),LOAM 風格依賴多線光達密度的角點/平面特徵擷取,在這麼稀疏的線數下明顯撐不住——約第 17 個關鍵幀開始出現物理上不可能的跳動(1.4 秒內跳 46 公尺),最終飄到 (-92.5, 32.9, -1.95)。過程中也在這個 port 本身抓到兩個 bug:①IMU 姿態轉換函式裡有一行忘記套用真實方向,永遠回傳單位外參旋轉,等於完全沒有用到這顆 9 軸 IMU 的真實姿態;②`lidarFrame == baselinkFrame` 時,TF 發布分支忘記設定 `frame_id`,狂噴 `TF_NO_FRAME_ID`(不影響 `mapOptimization` 本身的軌跡/點雲輸出)。`/odometry` 同樣不適用——官方 LIO-SAM 沒有外部里程計輸入介面,`odomTopic` 名稱其實是它自己輸出的 IMU 預積分結果 |

**這次最乾淨的發現**:Cartographer 3D、RTAB-Map、FAST-LIO2 三套完全獨立建置、獨立執行的系統,重建出幾乎一模一樣的房間形狀跟同一個 J 形轉彎路徑(路徑長度都落在 10.9~11.9m 之間),互相驗證了正確性,也證明 `compal_amr_allen` 的 `spot_cartographer` 設定檔在這份資料上運作正常。LIO-SAM 是唯一失敗的系統,而且原因明確指向感測器本身只有 4 條掃描線——對 LOAM 風格特徵擷取來說先天資訊量不足,不是移植過程配置錯誤。

⚠️ 這份資料只有 41 秒、動作偏溫和,Cartographer 沒有踩到 `CARTOGRAPHER_FAILURE.md` 記錄的 IMU 重力追蹤 bug,不代表這個 bug 在同一台機器人錄的更長、動作更劇烈的資料上就不會出現。

**額外發現:FAST-LIO2 有真實但輕微的垂直(z)漂移。** 這段路徑理論上全程在平面地板上,z 應該幾乎不動。三套系統裡:Cartographer 的 z 落在 [-4cm, +2.3cm],沒有明顯趨勢;RTAB-Map 因為開了 `Reg/Force3DoF=true` 把 z 強制鎖在 0;但 FAST-LIO2 的 z 從 0 開始**單調爬升**到 0.174m,41 秒內沒有回頭。規模不到「發散」的程度,但方向性很清楚——FAST-LIO2 沒有迴環偵測也沒有平面約束去修正垂直方向的小誤差,純靠 ESKF 積分,累積起來就是這種持續往一個方向飄的結果。

![Compal AMR rosbag2_mapping_dataset 四套系統比較圖(Cartographer/RTAB-Map/FAST-LIO2 成功且互相吻合,LIO-SAM 因光達只有 4 條掃描線而發散)](docs/images/compal_amr/comparison_map_grid_compal_amr.png)

## 資料集二:rosbag2_mapping_dataset_2026_09_21(頭尾相同的閉環路徑,153.2 秒)

同一台 Compal AMR、同一套感測器組合,這次錄製成**閉環路徑**(起點跟終點是同一個實際位置跟朝向),所以可以比照 README.md 裡 Campus 資料集的方法,直接算「頭尾姿態距離」當精度指標,不用再靠「四套系統形狀是否一致」這種間接驗證。一樣只用 `/vanjee_points719e_merged` 跟 `/odometry`。

| 系統 | 狀態 | 頭尾誤差(端到端) | 路徑長度 | 姿態數 | 備註 |
|---|---|---|---|---|---|
| Cartographer 3D | ✅ 成功 | **2.64cm**(0.05% of path) | 52.06m | 1306 | 目前為止最準的結果。離線 log 顯示有 282 個迴環約束生效(平均分數 0.53),z 全程穩定在 [-0.8cm, 16.6cm],閉環確實有被正確偵測跟優化 |
| RTAB-Map | ✅ 成功 | 1.25m(2.4% of path) | 51.75m | 148(116 節點於工作記憶體) | 全程沒有失去追蹤、沒有錯誤,`Reg/Force3DoF=true` 的修法在這份更長的資料上依然有效 |
| FAST-LIO2(官方,未修改) | ✅ 成功 | 1.10m(1.4% of path) | 76.19m | 3057 | 純里程計、沒有迴環偵測,累積誤差比 Cartographer 大上兩個數量級,但符合預期——這正是「有沒有迴環偵測」的差異 |
| LIO-SAM | ❌ 嚴重發散 | 450m(**數字沒有意義**) | 12,608m(同樣沒有意義) | 106(第 8~9 個關鍵幀後開始跳) | 跟資料集一同一個根因(光達只有 4 條掃描線),但這次路徑更長、更複雜,崩得更早更徹底——第 8→9 個關鍵幀就跳了 3m,第 14~23 個之間跳到 13~40m,第 30 個之後單步跳動 150~250m,最終落在 (129, -63, -427)m |

**這次驗證了兩件事**:①`spot_cartographer` 在這份閉環資料上表現得比另外兩套成功的系統好上兩個數量級(2.64cm vs. 1.1~1.25m)——差異主要來自迴環偵測(Cartographer 的 pose graph 優化 + 真的偵測到 282 個約束),FAST-LIO2 完全沒有迴環機制,RTAB-Map 雖然理論上有,但這次頭尾誤差跟 FAST-LIO2 同一個量級,沒有明顯發揮迴環優勢;②LIO-SAM 在同一顆感測器上第二次發散,而且路徑越長、複雜度越高,崩潰得越早越徹底,證實資料集一的診斷(4 線光達撐不住 LOAM 特徵擷取)是穩定重現、不是單次意外。

![Compal AMR rosbag2_mapping_dataset_2026_09_21 四套系統比較圖(Cartographer 頭尾誤差 2.64cm 最準,RTAB-Map/FAST-LIO2 約 1.1~1.25m,LIO-SAM 完全發散)](docs/images/compal_amr/comparison_map_grid_compal_amr_2026_09_21.png)

## 資料集三:rosbag2_2026_09_21-07_28_09_no_camera(809 秒、單向約 172m 的長直線來回,四套系統改用官方版本)

同一台 Compal AMR,錄了 13.5 分鐘,沿一條約 172m 的長直線往返(折返點約在 396 秒)。只用 `/vanjee_points719e_merged`(4 線)、`/imu/data`(約 66Hz)、`/odometry`、`/tf_static`;bag 裡的 Livox(`/livox/lidar`、`/livox/imu`)這次沒跑。**沒有 ground truth**,所以下表的 ATE 是對照 `/odometry`(輪式/融合里程計)算的,而 `/odometry` 本身會漂(首尾相距 9.33m,路徑 426.39m),它只是參考,不是真值。

**這次跟資料集一、二的差別**:四套系統都改成官方版本——Cartographer 用 apt 官方套件加 `spot_cartographer` 的 `spot_offline_mapping.lua`、RTAB-Map 用 apt 官方套件(以 `/odometry` 為骨幹、`Reg/Force3DoF=true`)、FAST-LIO2 用 `hku-mars/FAST_LIO` 未修改原始碼、LIO-SAM 用官方 `TixiaoShan/LIO-SAM` 的 `ros2` 分支(commit `08af3f3`;只加一個編譯用 patch,把 `find_package(Eigen)` 改成 `Eigen3`,見 `slam_comparison/docker/lio_sam_official_build_fix.patch`)。資料集一、二的 LIO-SAM 用的是社群 port `pixwyh/LIO-SAM-ROS2`,不是同一份程式碼。工具鏈打包成三個 docker 映像(`slam_comparison/docker/`),四套系統在同一台機器上平行跑;RTAB-Map、FAST-LIO2、LIO-SAM 是即時播放(1x),Cartographer 是離線處理。

### 全段(809.1 秒)

| 系統 | 狀態 | 首尾距離 | 路徑長度 | 姿態數 | ATE vs `/odometry` | 備註 |
|---|---|---|---|---|---|---|
| Cartographer 3D | ✅ 成功 | **0.044m** | 443.95m | 9600 | 2.94m(最大 5.80m) | 14157 次約束計算裡有 767 個成為新約束,沒有崩潰或斷言。z 範圍 [-1.48, 1.29]m |
| RTAB-Map | ✅ 成功(尾端有一小段異常) | **0.015m** | 469.79m | 556 | 3.10m(最大 23.6m) | 資料庫 634 個節點,1486 條局部空間鄰近連結,沒有全域迴環(沒有影像),6 次迴環被判定為錯誤而拒絕。優化後的位姿裡,t≈798 秒附近少數幾個節點跳離約 22m(最大誤差來源),其餘正常。z 被 `Force3DoF` 鎖在 0 |
| FAST-LIO2(官方) | ❌ 發散 | 2318m | 4543m | 16135 | 196.3m(最大 2254m) | 誤差在 213 秒超過 5m、386 秒超過 50m,終點 z=-2053m;log 有 148 次 `No Effective Points!` |
| LIO-SAM(官方) | ❌ 發散 | — | 244,576m | 681 個關鍵幀 | — | 475 次 `Large velocity, reset IMU-preintegration!`,z 被夾在預設的 ±1000m 上限(`z_tollerance`) |

**能確定的事**:
- Cartographer 和 RTAB-Map 兩套獨立系統都把路徑閉合到公分等級(4.4cm、1.5cm),而 `/odometry` 首尾相距 9.33m,所以這條路徑應該是回到起點的閉環(這是推論,沒有人工確認)。兩者的軌跡彼此吻合:對齊後中位數相差 1.17m、95 百分位 2.0m(不含上面那個 22m 的尖點)。
- FAST-LIO2 在前 3 分鐘還跟參考吻合(ATE 0.16m),之後誤差從約 2 分鐘起持續放大,到折返點附近(396 秒)已超過 50m,終點離起點 2318m。LIO-SAM 全程無法使用。

**不能下的結論**:
- 沒有 ground truth,不能說 Cartographer 比 RTAB-Map「更準」。往返兩趟的橫向間距(回程到去程路徑的最近距離,離起點 20m 以外)是:`/odometry` 平均 1.97m,Cartographer 3.45m,RTAB-Map 3.53m——兩套 SLAM 並沒有比原始里程計更「重疊」,但回程本來就可能走不同車道,所以這個數字也不能當作誤差。
- FAST-LIO2 為什麼發散,我沒有驗證。前 396 秒都是筆直的長路徑,誤差隨距離持續增加,跟 LegKilo `corridor.bag` 上觀察到的「長走廊幾何退化」模式相符,但這只是推測。
- LIO-SAM 的失敗跟資料集一、二同一類(這顆光達只有 4 條線,LOAM 風格特徵擷取先天不足);這次用官方版本,一樣失敗,而且更早——3 分鐘試跑裡第 2 個關鍵幀起 z 每幀掉 1~2m。

![軌跡對照:Cartographer 與 RTAB-Map 都閉合了路徑,FAST-LIO2 誤差隨時間發散](docs/images/compal_amr/trajectory_compare_compal_amr_full.png)

![全段點雲地圖與軌跡四宮格(俯視、依離地高度上色、白線為軌跡;各格比例尺不同)](docs/images/compal_amr/comparison_map_grid_compal_amr_full.png)

讀圖注意:顏色是**離地高度**,配色跟其他資料集的圖一樣(紅=低、藍=高),但四格用同一條固定色帶(-0.5m 到 3.5m,超出範圍的夾在兩端顏色),其他資料集的圖則是各自拉伸,所以同色代表同高度。離地高度是用各系統軌跡起點的 z 加上該座標系離地的高度換算的(假設起點附近地面是平的;`tf_static` 裡 `base_footprint→base_link` 是 0.1457m,Cartographer 與 FAST-LIO2 追蹤的是 `imu_link`,再高 0.077m,共 0.2227m)。發散的兩格(FAST-LIO2、LIO-SAM)大部分點的高度早已飄出色帶,會被夾成同一個顏色,只有起點附近保有正常的顏色。RTAB-Map 開了 `Force3DoF`,位姿是水平的,所以車身傾斜造成的高度差沒有被補償,跟 Cartographer 的 6 自由度結果在細節上會不一樣。每一格的比例尺不同(左下角的白色線段,分別是 10m、10m、100m、1000m),俯視方向已旋轉成軌跡主軸水平,方形標記是終點、圓點是起點。點雲來源也不同:Cartographer 是 `cartographer_assets_writer` 輸出(有移除移動物體的濾波,隨機取 400 萬點繪圖);RTAB-Map 的 `rtabmap-export` 匯出 0 個點,所以是用優化後的節點位姿加上對應的原始掃描自己組出來的(體素 0.1m,`assemble_rtabmap_cloud.py`);FAST-LIO2 是它存下的整張累積地圖(隨機取 400 萬點);LIO-SAM 是 `cloudGlobal.pcd`。RTAB-Map 那格右端往外延伸的白線,就是上面提到尾端約 22m 的尖點。

### 先跑的 3 分鐘試通(前 180 秒、3596 幀)

| 系統 | 結果(ATE 對照 `/odometry`) |
|---|---|
| Cartographer 3D | 0.237m,路徑 56.24m(`/odometry` 為 53.94m) |
| RTAB-Map | 0.128m,路徑 53.85m |
| FAST-LIO2(官方) | 0.164m;xy 吻合,但 z 持續上升到 3.46m |
| LIO-SAM(官方) | 發散(路徑 8315m,終點離起點 1149m) |

### 這次踩到的東西

- **RTAB-Map 匯出**:`export_rtabmap_trajectory.py` 讀的是資料庫 `Node.pose`,那是**未優化的里程計位姿**(這次全程跟 `/odometry` 相差 0.00m),不是迴環優化後的結果。這份資料改用 `rtabmap-export --poses --opt 0` 匯出優化後位姿。
- **FAST-LIO 外參符號**:`extrinsic_T` 是「光達在 IMU 座標系的位置」,所以要填 `base_link→imu_link` 平移量的**負值**(-0.1266, 0, -0.077)。資料集二的設定檔填的是正值。我在 3 分鐘資料上兩種符號都跑過,結果差異可忽略(ATE 0.164m vs 0.155m,z 上飄同樣約 3.5m),所以 z 上飄不是符號造成的。
- **LIO-SAM 官方版**:輸出的 `transformations.pcd` 是 ASCII 格式,`export_liosam_trajectory.py` 已補上支援;關鍵幀時間戳因為精度被截斷,不能用來做時間對齊。官方版一樣會噴 `TF_NO_FRAME_ID`(不影響 `mapOptimization` 的輸出)。
- 重跑方式:`bash slam_comparison/docker/run_lab_all.sh <切好的 bag> <標籤>`;原始資料、轉檔、映像備份都在 `data/`,輸出在 `outputs/compal_amr_<標籤>_*`(兩者都不在 git 裡)。
