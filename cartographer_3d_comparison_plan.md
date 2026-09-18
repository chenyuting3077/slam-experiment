# Cartographer 3D / RTAB-Map SLAM 比較實作計畫(ROS 2 Jazzy)

## 目的

在 LIO-SAM 官方釋出的 **Campus dataset** 上實際跑 **Cartographer 3D** 與 **RTAB-Map**,取得可量化的軌跡誤差與地圖品質數據,並與 LIO-SAM 論文(及 FAST-LIO2 論文,若適用)已發表的數字做對照,產出一份四方比較報告(Cartographer 自跑 vs. RTAB-Map 自跑 vs. LIO-SAM 論文數字 vs. FAST-LIO2 論文數字)。

**關於 RTAB-Map 加入比較的重要說明：**
- RTAB-Map 預設是為視覺/RGB-D SLAM 設計,回環偵測依賴視覺特徵(bag-of-words)。但 `rtabmap_ros` 有**純 3D LiDAR 模式**(`lidar3d:=true`),搭配 `icp_odometry`(需要編譯時啟用 `libpointmatcher`)可以做純點雲的 6DoF 建圖,不依賴相機。
- 這代表 RTAB-Map 在這個比較裡跟 Cartographer 一樣,是「非原生設計對象但技術上可行」的狀況,報告裡應誠實說明 RTAB-Map 在純 LiDAR 模式下的回環偵測改用 ICP-based 幾何比對,而非其招牌的視覺 bag-of-words,效能可能無法反映 RTAB-Map 在視覺/RGB-D 場景下的真實優勢。
- LIO-SAM / FAST-LIO2 論文都沒有報告 RTAB-Map 在 Campus 這組資料上的數字,所以 RTAB-Map 跟 Cartographer 一樣需要**自己實際跑**,不能從論文引用。

**為什麼選 Campus,不選 Walking/Park：**
- LIO-SAM 論文 Table II「End-to-end translation error」只報告 Campus / Park / Amsterdam 三組,Walking 只有定性(視覺化)比較,無法量化對比。
- Park 資料集實際長度約 41 分鐘(24,691 幀),對「不想跑太久」的目標太大。
- Campus 約 16.4 分鐘、9,865 幀、軌跡 1,437m,大小適中,且論文有明確數字:
  - LOAM: 192.43 m
  - LIO-odom: 9.44 m
  - LIO-GPS: 6.87 m
  - **LIO-SAM: 0.12 m**

---

## 執行環境

**全程使用 ROS 2 Jazzy(Ubuntu 24.04)。**
- Cartographer 3D 與 RTAB-Map 皆已確認有官方 Jazzy 版本(`ros-jazzy-cartographer-ros`、`ros-jazzy-rtabmap-ros` 皆為官方 apt 套件,RTAB-Map 的 ROS2 分支 `jazzy-devel` 持續維護中)
- 下方所有指令、launch file 語法、node 名稱皆以 ROS 2(Jazzy)為準,不使用 ROS1 語法

---

## 階段一:環境與資料準備

1. **安裝套件**
   ```bash
   sudo apt update
   sudo apt install ros-jazzy-cartographer ros-jazzy-cartographer-ros ros-jazzy-cartographer-ros-msgs
   sudo apt install ros-jazzy-rtabmap ros-jazzy-rtabmap-ros
   ```
   - `rtabmap_ros` 的 apt 版本要另外確認是否已編譯進 `libpointmatcher`(純 LiDAR 的 `icp_odometry` 需要);若沒有,需從原始碼加上 `-DCMAKE_BUILD_TYPE=Release` 搭配 libpointmatcher 依賴重新編譯。

2. **資料格式轉換(重要)**
   - LIO-SAM 官方釋出的 Campus dataset 是 **ROS 1 bag 格式**(`.bag`),不能直接在 ROS 2 Jazzy 環境播放,需要先轉檔:
     ```bash
     pip install rosbags --break-system-packages
     rosbags-convert campus_dataset.bag --dst campus_dataset_ros2
     ```
   - 轉檔後產出 ROS 2 native 的 `.db3`(或視版本輸出 `.mcap`)+ `metadata.yaml`
   - 用 `ros2 bag info campus_dataset_ros2` 確認:
     - topic 列表(PointCloud2、IMU、可能有 GPS)
     - 訊息數量、時長是否與論文描述(9,865 幀、約 16.4 分鐘)相符
   - 若時長或幀數不符,先停下來確認轉檔或下載是否正確,不要直接往下跑。

3. **確認感測器規格**
   - VLP-16,10Hz,MicroStrain 3DM-GX5-25 IMU(9-axis)
   - 記下 LiDAR-IMU 外參(extrinsic),Cartographer 3D 設定檔需要對應的 TF 或 frame 定義。

---

## 階段二:Cartographer 3D 設定(ROS 2 Jazzy)

1. **建立設定檔**
   - 以 cartographer_ros 官方 `trajectory_builder_3d.lua`(或 backpack_3d 範例設定)為基礎修改,放在自己的 ROS 2 package 的 `config/` 目錄下
   - 調整項目:
     - `TRAJECTORY_BUILDER_3D.num_accumulated_range_data`:依 VLP-16 單線掃描頻率調整掃描累積數
     - `TRAJECTORY_BUILDER_3D.min_range` / `max_range`:依 VLP-16 有效測距範圍(約 0.5~100m,實務上建議設 100m 內)調整
     - IMU 相關參數(gravity 估計、噪聲參數)對應 3DM-GX5-25 規格
     - `POSE_GRAPH` 回環參數(`constraint_builder.min_score` 等)先用預設值起跑,若回環太少再調整(可參考已知 issue #1472 的教訓)

2. **建立 ROS 2 launch file**
   - 用 Python launch file(`launch/cartographer_campus.launch.py`),啟動 `cartographer_node` 並帶入 `-configuration_directory` / `-configuration_basename` 參數
   - 設定 topic remap(ROS 2 launch 用 `remappings=[...]` 參數),把資料集裡的 point cloud/IMU topic 對應到 cartographer_ros 期望的 topic 名稱
   - 若資料集沒有 TF(如 base_link → lidar_link),需要用 `static_transform_publisher`(ROS 2 版本參數是位置+四元數,注意跟 ROS1 版本語法不同)手動補上

3. **先用短片段測試 pipeline 能不能跑**
   - 用 `ros2 bag play campus_dataset_ros2 --duration 30` 先跑 30 秒驗證有沒有報錯、有沒有點雲進來、TF 有沒有問題
   - 確認沒問題後才跑完整資料集,避免跑到一半才發現設定錯誤,浪費時間

---

## 階段二點五:RTAB-Map 設定(純 LiDAR 模式,ROS 2 Jazzy)

1. **選擇正確的建圖模式**
   - 參考 `rtabmap_demos` 的 `demo_husky.launch.py`(ROS 2 版本是 `.launch.py`)用法,啟用 `lidar3d:=true slam2d:=false`(6DoF 純 3D LiDAR 建圖,不開 `camera:=true`,因為沒有相機資料)
   - Odometry 來源設為 `icp_odometry:=true`,這樣才是用點雲 ICP 做前端里程計,而不是視覺特徵

2. **建立對應 ROS 2 launch file**
   - 把 `icp_odometry` node 的輸入 topic remap 到資料集的 point cloud topic(ROS 2 launch 語法)
   - 把 IMU 資料接進去當 initial guess
   - 確認 frame_id 設定與 Cartographer 那邊一致,方便後續軌跡比較時座標系統統一

3. **回環偵測參數**
   - 純 LiDAR 模式下,RTAB-Map 的回環偵測改用幾何(ICP-based)比對,而非預設的視覺 bag-of-words,需要調整 `Icp/` 命名空間下的對應參數(細節依 rtabmap_ros ROS2 版本查官方 wiki)

4. **先用短片段測試**
   - 同 Cartographer,先跑 `ros2 bag play campus_dataset_ros2 --duration 30` 驗證 pipeline

---

## 階段三:執行與資料收集(ROS 2 Jazzy)

1. **正式執行(兩套系統都跑一次完整 Campus dataset)**
   - **Cartographer**:使用 `cartographer_offline_node`(ROS 2 版本一樣提供這個 node),用非即時速度跑,方便重跑與縮短時間
   - **RTAB-Map**:用 `ros2 bag play campus_dataset_ros2 --clock`(依實際即時速度播放,因為 RTAB-Map 的 ICP odometry 通常需要接近即時的輸入節奏,不像 Cartographer offline node 那樣可以任意加速)
   - 兩者都記錄:
     - 實際處理時間(wall clock time)
     - bag 原始時長
     - 兩者相除得到 real-time factor(供跟 LIO-SAM 論文的即時性描述對照)

2. **儲存最終狀態**
   - Cartographer:呼叫 ROS 2 service(`ros2 service call /finish_trajectory ...` + `ros2 service call /write_state ...`),產出 `.pbstream`
   - RTAB-Map:建圖過程會自動存到 `~/.ros/rtabmap.db`(SQLite 資料庫),裡面包含完整的節點位姿與地圖資料

3. **記錄回環資訊**
   - Cartographer:從 log 或 pose graph 裡確認找到幾個 loop closure constraint(呼應已知 issue #1472 的觀察)
   - RTAB-Map:可以用 `rtabmap-databaseViewer` 打開 `.db` 檔案,查看偵測到的回環連結數量與品質

---

## 階段四:數據萃取與計算

1. **取出軌跡**
   - **Cartographer**:沒有內建匯出 TUM/KITTI 格式的工具,需要:
     - 使用社群腳本(例如 cartographer_ros issue #460/#856 討論中提到的 `export_pose_graph.py` 或等價工具)解析 pbstream 裡的 pose graph node
     - 轉成純文字的 (timestamp, x, y, z, qx, qy, qz, qw) 序列
   - **RTAB-Map**:用官方工具 `rtabmap-export` 或 `rtabmap-reprocess`,也可以直接用 Python 讀取 `.db`(SQLite)裡的 `Node` 表取出每個關鍵幀的位姿,一樣轉成同樣格式的文字軌跡檔,方便跟 Cartographer 的輸出用同一套後續腳本處理

2. **計算 End-to-end translation error**
   - 用起點與終點(軌跡回到原點附近)的位置差距,算出歐氏距離(公尺)
   - 這個數字直接對照論文 Table II 的 LOAM/LIO-odom/LIO-GPS/LIO-SAM 數字
   - Cartographer 與 RTAB-Map 都用同一套計算腳本,確保算法一致

3.(可選)**若要算完整 ATE/RPE**
   - 需要 ground truth(Campus 資料集本身沒有 GPS/MoCap ground truth,只能做起終點誤差;若要更完整的軌跡誤差分析,需改用有 GPS 的 Park 資料集,但那組較大,超出目前範圍)
   - 若之後要做,流程是:轉換好的軌跡檔案 + ground truth 檔案 → `evo_ape` / `evo_rpe`(TUM 或 KITTI 格式),Cartographer 跟 RTAB-Map 的軌跡檔案都轉成同一格式後即可直接餵給 evo 一起比較

4. **匯出點雲地圖檔案**
   - **Cartographer**:用官方 `cartographer_assets_writer` 搭配對應的 `assets_writer_backpack_3d.lua` pipeline 設定,把 `.pbstream` 轉成 `.ply`(或 `.pcd`,若走 PCL 轉檔工具)點雲檔
   - **RTAB-Map**:用 `rtabmap-export`(或 `rtabmap-databaseViewer` 內的匯出功能)把 `.db` 匯出成 `.ply`/`.pcd`,建議加上 `--cloud`(輸出組合後的完整點雲地圖)參數
   - 兩者統一輸出成同一種格式(建議 `.pcd`,方便後續用 PCL/Open3D 讀取比較密度、雜訊等指標),檔名建議包含系統名稱與資料集名稱(例如 `cartographer_campus.pcd`、`rtabmap_campus.pcd`)方便管理

5. **地圖品質觀察(定性)**
   - 用 RViz2 或上一步匯出的點雲檔案檢查兩套系統的地圖:
     - 是否有模糊、重疊、z 方向重影(Cartographer 對照已知 issue #1432)
     - RTAB-Map 純 ICP 模式下,觀察是否因為缺少視覺回環而出現漂移或地圖斷裂
     - 主觀評分或截圖留存,作為報告的視覺佐證

---

## 階段五:比較報告產出

整理成一張表:

| 指標 | LOAM(論文) | LIO-odom(論文) | LIO-GPS(論文) | LIO-SAM(論文) | FAST-LIO2(若有報同資料集) | **Cartographer 3D(本次實測)** | **RTAB-Map(本次實測)** |
|---|---|---|---|---|---|---|---|
| End-to-end translation error (m) | 192.43 | 9.44 | 6.87 | 0.12 | — | (實測值) | (實測值) |
| Real-time factor | — | — | — | 官方稱可達 10x | — | (實測值) | (實測值) |
| 回環數量 | — | — | — | — | — | (實測值) | (實測值) |
| 地圖品質(定性) | — | — | — | — | — | (觀察描述) | (觀察描述) |

**報告需誠實註明的限制**
- Cartographer、RTAB-Map 數字是本次實測結果;LIO-SAM/FAST-LIO2 數字為引用各自論文報告值,並非同一硬體/同一次執行環境下的公平競賽,只能視為「參考基準」而非嚴格對照實驗。
- Campus 資料集沒有精確 ground truth(GPS/MoCap),End-to-end translation error 是唯一可直接對照論文的量化指標,其他如 ATE/RPE 無法在此資料集上計算。
- RTAB-Map 在這組資料上是跑「純 LiDAR ICP 模式」,並非其原本以視覺回環見長的典型使用場景,報告中應說明這點,避免讀者誤以為這代表 RTAB-Map 的整體實力。
- 若審閱者質疑比較公平性,應在報告中主動說明以上限制。

---

## 交付項目(給 Claude Code 的具體任務清單)

1. 撰寫 / 修改 cartographer 3D 的 `.lua` 設定檔與對應 ROS 2 Python launch file
2. 撰寫 RTAB-Map 純 LiDAR 模式的 ROS 2 launch file(`icp_odometry` + `rtabmap` 節點,含 topic remap)
3. 撰寫 ROS 1 bag → ROS 2 bag 轉檔腳本(用 `rosbags-convert`),並附帶轉檔後的檢查腳本(確認 topic、時長、幀數)
4. 撰寫或整合軌跡解析腳本:
   - Cartographer:pbstream → 文字軌跡檔
   - RTAB-Map:`.db`(SQLite)→ 文字軌跡檔
   - 統一輸出成同一種格式(如 TUM),方便後續共用計算腳本
5. 撰寫點雲地圖匯出腳本:
   - Cartographer:`.pbstream` → `.pcd`/`.ply`(透過 `cartographer_assets_writer`)
   - RTAB-Map:`.db` → `.pcd`/`.ply`(透過 `rtabmap-export`)
   - 統一輸出格式與檔名規則,並整理到固定輸出目錄(如 `outputs/pointclouds/`、`outputs/trajectories/`)
6. 撰寫 end-to-end translation error 計算腳本(兩套系統共用同一份腳本)
7. 撰寫執行腳本(皆用 ROS 2 指令):
   - Cartographer:自動跑 offline node、記錄 wall clock time、呼叫 `write_state` service
   - RTAB-Map:自動跑 icp_odometry + rtabmap 節點、記錄 wall clock time、確認 db 產生
8. 整理輸出成上述比較表格式(可以是 CSV 或 Markdown 表),欄位涵蓋 Cartographer、RTAB-Map 與論文引用數字
9. 確保最終交付物包含:每套系統各一份**軌跡檔案**(TUM 格式文字檔)與一份**點雲地圖檔案**(`.pcd`/`.ply`),連同比較表一起輸出
