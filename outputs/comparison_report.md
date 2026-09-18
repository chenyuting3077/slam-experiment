# Cartographer 3D / RTAB-Map / LIO-SAM / FAST-LIO2 比較報告

資料集:LIO-SAM 官方 Campus dataset(`campus_large_dataset.bag`,VLP-16 + MicroStrain 3DM-GX5-25 IMU)。轉檔驗證:**9,865 幀點雲、16.58 分鐘(994.97s)、路徑長度 ~1420-1441m**,與論文描述(9,865 幀、16.4 分鐘、1,437m)吻合。

## 比較表

| 指標 | LOAM(論文) | LIO-odom(論文) | LIO-GPS(論文) | LIO-SAM(論文) | **Cartographer 3D(本次實測)** | **RTAB-Map(本次實測)** | **LIO-SAM(本次實測)** | **FAST-LIO2(本次實測)** |
|---|---|---|---|---|---|---|---|---|
| End-to-end translation error (m) | 192.43 | 9.44 | 6.87 | 0.12 | **34.69** | **256.06** | **38.06** | 失敗 |
| 軌跡長度 (m) | — | — | — | — | 1441.45 | 1418.01 | 1426.65 | — |
| Real-time factor | — | — | — | 官方稱可達 10x | ~1.0(即時) | ~1.0(即時) | ~1.0(即時) | — |
| Loop closure 數量 | — | — | — | — | **0** | **0** | **0**(無相關 log) | — |
| 地圖品質(定性) | — | — | — | — | 31.7M 點,結構完整但終點漂移明顯 | 15.4M 點,z 方向嚴重漂移(見下) | 完整全域地圖(Corner+Surf ~110MB×2) | 無(mapping 中途損壞) |

## 各系統詳細結果

### Cartographer 3D — ✅ 成功
- 原計畫用 `cartographer_offline_node`(非即時、快速),但該 ROS2 port 的 bag 內部 topic 比對邏輯有 bug,即使 `-r` remap 參數正確出現在 argv 上也完全不會把資料餵進演算法(pbstream 只有 520 bytes、0 submap、0 constraint)。改用 **live 版 `cartographer_node`** 配合即時 `ros2 bag play`,才真正跑起來(62 個 submap、9844 個 trajectory node)。
- 終點誤差 34.69m。`POSE_GRAPH.constraint_builder` 全程回報 **0 additional constraints**——這個設定(`sampling_ratio=0.03`、`min_score=0.62`)在這條路徑上完全沒找到有效回環,這也是終點漂移的主因。
- 點雲(`cartographer_campus.pcd`,31.7M 點)是我們自己寫的腳本,用 Cartographer 匯出的軌跡把原始 `/points_raw` 逐幀轉到世界座標系累積而成,**不是**用官方 `cartographer_assets_writer`——那個工具內部有個寫死的 grid bit 上限(`hybrid_grid.h: new_bits <= 8`),處理這種大範圍戶外地圖時必定 crash(`Check failed: new_bits <= 8 (9 vs. 8)`),無法透過設定檔調整。

### RTAB-Map(純 LiDAR ICP 模式) — ✅ 成功
- `icp_odometry` + `rtabmap` 純幾何模式,981 個關鍵節點、3767 條連結。
- 終點誤差 256.06m,是四套裡最差的,且 z 方向漂移到 235m(明顯不合理,應該是純 ICP 缺乏視覺回環、僅靠 IMU gravity 先驗撐著,長時間累積誤差)。
- Link 型態分佈:1960 條 neighbor(里程計序列連結)、827 條 pose-prior、980 條 gravity——**沒有任何 loop closure 型態的連結**,證實回環偵測完全沒有生效。
- 這完全符合原計畫的預期警語:RTAB-Map 在純 LiDAR ICP 模式下是「非典型使用場景」,不能反映它招牌的視覺 bag-of-words 回環實力。
- 點雲用官方 `rtabmap-export --cloud --scan --voxel 0.05`,15.4M 點,`outputs/pointclouds/rtabmap_campus.ply`。

### LIO-SAM(Docker/ROS 2 Humble,ros2 分支) — ✅ 成功(但誤差遠高於論文)
- 官方 `ros2` 分支只保證到 Humble,故用官方 Dockerfile 建的 Humble 容器跑,容器內部自己 `ros2 bag play`(避開 Humble/Jazzy 跨版本 DDS 資料層不互通的問題——實測發現：topic discovery 可以跨版本互見,但實際訊息完全不會送達,`ros2 topic echo`/`hz` 等 CLI 工具還會因為新版 TypeHash 欄位直接拋例外)。
- `lio_sam_imuPreintegration` 這個輔助節點在每次啟動後不久就會丟 GTSAM `IndeterminantLinearSystemException`(bias 變數 b0 欠約束)死掉,但確認過這不影響主線——`imageProjection`/`featureExtraction`/`mapOptimization` 三個核心節點全程存活,map 持續在累積。
- 終點誤差 **38.06m**,對照論文報的 **0.12m** 差了超過 300 倍。log 裡完全沒出現任何 loop closure 相關訊息(`loopClosureEnableFlag: true` 但沒觸發),推測跟 imuPreintegration 反覆掛掉導致高頻率初始猜測品質下降、以及這個社群 ROS2 port 相對原始 ROS1 版本可能存在的迴歸有關。
- 透過 `lio_sam/save_map` service 存檔(注意：程式碼的存檔路徑會自動加上 `$HOME` 前綴,即使 `destination` 給絕對路徑也一樣,要注意 mount 對應),`outputs/liosam_campus/`:`GlobalMap.pcd`(113MB)、`transformations.pcd`(關鍵幀 6DoF 軌跡)。

### FAST-LIO2(MIT-SPARK spark-fast-lio) — ❌ 放棄
- 原生 build 在 Jazzy 上沒問題,前 100~270 秒(視每次跑法不同)mapping 正常,之後永久性地開始每幀回報 `No Effective Points!`/`No point, skip this scan!`,再也沒恢復。
- 嘗試過的修法都沒解決:
  1. 懷疑是 PCL VoxelGrid 對全域地圖做 voxel filter 時,座標範圍太大導致 32-bit index 溢位(`Leaf size is too small... Integer indices would overflow`)→ 把 `filter_size_map` 從 0.3 調到 1.0m,溢位訊息消失,但换成一样的 "No Effective Points" 只是更早發生(108s 就開始)。
  2. 懷疑是 `cube_side_length`(區域地圖立方體)在位置超出邊界時的重新置中邏輯有 bug → 從 1000m 調到 5000m,一樣沒用(22806 次失敗)。
- 判定是這個第三方 ROS2 port 本身的問題(可能在 ikd-tree 的執行緒管理或地圖維護邏輯裡),超出這次任務的除錯範圍,不再繼續嘗試。誠實記錄:**FAST-LIO2 在這個環境下沒有產出有效的完整軌跡或地圖**。

## 誠實限制與注意事項

1. **Cartographer、RTAB-Map、LIO-SAM 數字是本次實測結果;LOAM/LIO-odom/LIO-GPS/LIO-SAM 論文數字為引用值**,不是同一硬體/同一次執行環境下的公平競賽,只能當參考基準。
2. **三套成功的系統在這條路徑上都沒有找到有效的 loop closure**(各自的判定機制不同,但結果一致),這是造成終點誤差普遍遠高於論文 LIO-SAM 數字的主要原因。這些系統的預設/移植版參數並未針對這個資料集調優,調整回環偵測參數(例如 Cartographer 的 `min_score`/`sampling_ratio`、RTAB-Map 的 ICP 對應距離、LIO-SAM 的 `historyKeyframeSearchRadius`)有機會改善,但超出本次任務範圍。
3. Campus 資料集沒有精確 ground truth(GPS/MoCap),End-to-end translation error 是唯一能直接對照論文的量化指標,無法計算完整 ATE/RPE。
4. Cartographer 改用 live node 而非原計畫的 offline node,實際執行是「即時播放」而非論文 LIO-SAM 宣稱的「10 倍加速」,real-time factor 因此都落在 ~1.0 附近,無法直接對照 LIO-SAM 論文的加速倍數描述。
5. RTAB-Map 是在「純 LiDAR ICP 模式」下測試,並非其原本以視覺回環見長的典型使用場景,這裡的數字不能代表 RTAB-Map 在視覺/RGB-D 場景下的真實實力。
6. FAST-LIO2 的失敗是**這個特定 ROS2 社群移植版(spark-fast-lio)在這台環境下的問題**,不代表 FAST-LIO2 演算法本身在其他實作或環境下也會有這個問題。

## 交付物清單

- 軌跡(TUM 格式):`outputs/trajectories/{cartographer,rtabmap,liosam}_campus.txt`
- 點雲地圖:`outputs/pointclouds/{cartographer,rtabmap}_campus.{pcd,ply}`、`outputs/liosam_campus/GlobalMap.pcd`
- 原始狀態檔:`outputs/cartographer_campus.pbstream`、`outputs/rtabmap_campus.db`
- 完整執行 log:`outputs/logs/`
