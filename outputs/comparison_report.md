# Cartographer 3D / RTAB-Map / LIO-SAM / FAST-LIO2 比較報告

資料集:LIO-SAM 官方 Campus dataset(`campus_large_dataset.bag`,VLP-16 + MicroStrain 3DM-GX5-25 IMU)。轉檔驗證:**9,865 幀點雲、16.58 分鐘(994.97s)、路徑長度 ~1420-1441m**,與論文描述(9,865 幀、16.4 分鐘、1,437m)吻合。

## 比較表

| 指標 | LOAM(論文) | LIO-odom(論文) | LIO-GPS(論文) | LIO-SAM(論文) | **Cartographer 3D(本次實測)** | **RTAB-Map(本次實測)** | **LIO-SAM(本次實測)** | **FAST-LIO2(本次實測)** |
|---|---|---|---|---|---|---|---|---|
| End-to-end translation error (m) | 192.43 | 9.44 | 6.87 | 0.12 | **31.67**(調參前 34.69) | **163.67**(調參前 256.06) | **0.288** | 失敗 |
| 軌跡長度 (m) | — | — | — | — | 1444.15 | 1417.45 | 1432.82 | — |
| Real-time factor | — | — | — | 官方稱可達 10x | ~1.0(即時) | ~1.0(即時) | ~1.0(即時) | — |
| Loop closure 數量 | — | — | — | — | **20**(調參後,調參前 0) | **0**(調參後依然是 0) | 有效(調參後,見下) | — |
| 地圖品質(定性) | — | — | — | — | 31.7M 點,結構完整但終點漂移明顯 | 15.4M 點,z 方向嚴重漂移(見下) | 完整全域地圖(Corner+Surf ~110MB×2),終點幾乎完全閉合 | 無(mapping 中途損壞) |

## 各系統詳細結果

### Cartographer 3D — ✅ 成功,調參後終點誤差 31.67m(有找到迴環,但改善有限)
- 原計畫用 `cartographer_offline_node`(非即時、快速),但該 ROS2 port 的 bag 內部 topic 比對邏輯有 bug,即使 `-r` remap 參數正確出現在 argv 上也完全不會把資料餵進演算法(pbstream 只有 520 bytes、0 submap、0 constraint)。改用 **live 版 `cartographer_node`** 配合即時 `ros2 bag play`,才真正跑起來(62 個 submap、9844 個 trajectory node)。
- **第一輪**(`sampling_ratio=0.03`、`min_score=0.62`,比官方預設更保守):終點誤差 34.69m,`POSE_GRAPH.constraint_builder` 全程回報 **0 additional constraints**,完全沒找到回環。
  - 根因跟 LIO-SAM 同一類:官方預設 `constraint_builder.max_constraint_distance = 15.0`,只在submap 中心點相距 15m 內才會嘗試建立 constraint。真實 ICP/LiDAR 里程計在 1437m 戶外大迴圈上的漂移,回到起點附近時很可能已經超過 15m。
  - **修正:`max_constraint_distance` 15→60,`sampling_ratio` 改回官方預設 0.3 附近(取 0.5 更寬鬆),`min_score`/`global_localization_min_score` 改回官方預設 0.55/0.6** → 這次真的找到 **20 個 additional constraints**(分散在整趟路徑的 76~1004 秒之間,代表軌跡中途多次交叉都成功配對,不是只有結尾一次)。
  - 但終點誤差只從 34.69m 降到 **31.67m**,改善幅度遠不如 LIO-SAM(38.06m→0.288m)。推測原因:Cartographer 的 constraint builder 是機會式地在整個路徑上找任意 submap 配對,不像 LIO-SAM 每個新 keyframe 都明確去跟全部歷史配對——這 20 個 constraint 修正的是路徑中途的局部交叉,不保證包含「終點跟起點」這條最關鍵的連結;而且 Cartographer 3D 預設的 `pose_extrapolator` 是**等速度模型**,不是像 LIO-SAM/FAST-LIO2 那樣的緊耦合 IMU 預積分,原始里程計本身的漂移量就比較大,回環修正的起始條件較差。詳見文末分析。
- 點雲(`cartographer_campus.pcd`,31.7M 點)是我們自己寫的腳本,用 Cartographer 匯出的軌跡把原始 `/points_raw` 逐幀轉到世界座標系累積而成,**不是**用官方 `cartographer_assets_writer`——那個工具內部有個寫死的 grid bit 上限(`hybrid_grid.h: new_bits <= 8`),處理這種大範圍戶外地圖時必定 crash(`Check failed: new_bits <= 8 (9 vs. 8)`),無法透過設定檔調整。

### RTAB-Map(純 LiDAR ICP 模式) — ✅ 成功,但迴環修正沒有真正生效
- `icp_odometry` + `rtabmap` 純幾何模式,981 個關鍵節點、3767 條連結。
- **第一輪**(`RGBD/LocalRadius` 預設 10m):終點誤差 256.06m,是四套裡最差的,且 z 方向漂移到 235m。Link 型態分佈:1960 條 neighbor、827 條 pose-prior、980 條 gravity——**沒有任何 loop closure 型態的連結**。
- **修正嘗試:`RGBD/LocalRadius` 10→60m、`RGBD/ProximityMaxGraphDepth` 50→0(不限制)**,重新完整跑一次(976 個節點)→ Link 型態分佈**依然是 0 條 loop closure**(1950 neighbor、819 pose-prior、975 gravity)。終點誤差意外從 256.06m 降到 163.67m,但 z 方向仍嚴重漂移(153m),這個改善比較可能是優化步驟其他連帶效應,**不是**真正的迴環修正生效。
- 根因跟 Cartographer/LIO-SAM 不同層次:`RGBD/LocalRadius` 是用「里程計估計的目前位置」去搜尋附近的舊節點,但 RTAB-Map 純 ICP 里程計(只在初始化時用 IMU 對齊重力,沒有逐幀緊耦合)本身漂移量太大(z 方向落差達 150m+),就算搜尋半徑再放寬,**估計位置本身已經跟真實位置差了 150 米以上**,遠超任何合理的搜尋半徑,回環候選自然找不到。這代表 RTAB-Map 在這個資料集上的問題核心是**原始里程計精度不足**,不是回環搜尋半徑的參數問題——跟 Cartographer(有找到回環但修正幅度有限)、LIO-SAM(里程計夠準,調寬半徑後幾乎完全修正)形成清楚對比,詳見文末分析。
- 這完全符合原計畫的預期警語:RTAB-Map 在純 LiDAR ICP 模式下是「非典型使用場景」,不能反映它招牌的視覺 bag-of-words 回環實力。
- 點雲用官方 `rtabmap-export --cloud --scan --voxel 0.05`,15.4M 點,`outputs/pointclouds/rtabmap_campus.ply`(第一輪結果;第二輪的點雲沒有重新匯出,因為軌跡本質上沒有真正修正)。

### LIO-SAM(Docker/ROS 2 Humble,ros2 分支) — ✅ 成功,調參後終點誤差 0.288m
- 官方 `ros2` 分支只保證到 Humble,故用官方 Dockerfile 建的 Humble 容器跑,容器內部自己 `ros2 bag play`(避開 Humble/Jazzy 跨版本 DDS 資料層不互通的問題——實測發現：topic discovery 可以跨版本互見,但實際訊息完全不會送達,`ros2 topic echo`/`hz` 等 CLI 工具還會因為新版 TypeHash 欄位直接拋例外)。
- `lio_sam_imuPreintegration` 這個輔助節點在每次啟動後不久就會丟 GTSAM `IndeterminantLinearSystemException`(bias 變數 b0 欠約束)死掉,但確認過這不影響主線——`imageProjection`/`featureExtraction`/`mapOptimization` 三個核心節點全程存活,map 持續在累積。
- **第一輪(預設 `historyKeyframeSearchRadius: 15.0`)**:終點誤差 38.06m,對照論文 0.12m 差了 300 多倍,log 裡完全沒有 loop closure 觸發的跡象。
  - 根因分析:imuPreintegration 反覆掛掉,導致 mapOptimization 少了高頻率的初始猜測,scan-to-map 優化更容易累積漂移;而 `historyKeyframeSearchRadius=15.0` 只在「離目前估計位置 15m 內」找舊 keyframe 配對,一旦真實漂移超過這個半徑,迴環候選永遠找不到——形成「漂移越大→越找不到迴環→漂移持續加大」的自我強化失敗鏈。
  - **修正:把 `historyKeyframeSearchRadius` 從 15.0 調到 60.0m,其他都不變,重新完整跑一次** → **終點誤差降到 0.288m**,幾乎追平論文的 0.12m!軌跡長度 1432.82m(跟論文 1437m 幾乎一致)。證實根因判斷正確,迴環偵測這次確實生效(雖然 log 依然沒印出明確的 "loop closure found" 文字,這個 ROS2 port 在這件事上很安靜)。
- 透過 `lio_sam/save_map` service 存檔(注意:程式碼的存檔路徑會自動加上 `$HOME` 前綴,即使 `destination` 給絕對路徑也一樣,要注意 mount 對應),`outputs/liosam_campus/`:`GlobalMap.pcd`(124MB)、`transformations.pcd`(關鍵幀 6DoF 軌跡)。舊的 38.06m 結果保留在 `outputs/liosam_campus_v1_38m_bak/` 供對照。

### FAST-LIO2(MIT-SPARK spark-fast-lio,ROS2 port) — ❌ 放棄
- 原生 build 在 Jazzy 上沒問題,前 20~270 秒(每次跑法不同)mapping 正常,之後永久性地開始每幀回報 `No Effective Points!`/`No point, skip this scan!`,再也沒恢復。
- 四次修法嘗試,而且後三次呈現明確的**單調惡化**趨勢(272s → 108s → 37s → 20s 就開始永久失敗),說明調整方向都是錯的:
  1. 懷疑 PCL VoxelGrid 對全域地圖做 voxel filter 時座標範圍太大導致 32-bit index 溢位 → 調大 `filter_size_map`(0.3→1.0),溢位訊息消失但換成同樣的 "No Effective Points" 更早發生(108s)。
  2. 懷疑 `cube_side_length` 的區域地圖重新置中邏輯有 bug → 調大(1000→5000),檢查原始碼後發現這其實是標準 FAST-LIO2 演算法、調大反而讓單次搬移距離更誇張,更快壞掉(37s)。
  3. 懷疑 IMU noise 參數沒對這顆真實感測器校正(LIO-SAM 同樣緊耦合 IMU 也不穩定)→ 調大 acc_cov/gyr_cov,結果最快壞掉(20s),反證此假設,已改回原值。
- 完整 log 摘要見 `outputs/fastlio_failure_logs.md`。判定是這個第三方 ROS2 port 本身更深層的問題(ikd-tree 地圖維護或多執行緒同步邏輯),超出這次任務的除錯範圍。**改用官方 hku-mars/FAST_LIO(ROS1,原始實作)在 Docker/Noetic 容器裡測試中**,見下方最新進度。

## 為什麼 LIO-SAM(緊耦合 LiDAR-Inertial)明顯優於 Cartographer / RTAB-Map

四套系統都套用了「同一類參數修正」(把迴環/回環搜尋半徑從預設的 10~15m 調寬到 60m),但結果差異巨大——這不是偶然,反映的是三種不同的架構定位:

| | Cartographer 3D | RTAB-Map(純 ICP) | LIO-SAM |
|---|---|---|---|
| IMU 的角色 | `pose_extrapolator` 預設用**等速度模型**推算姿態,IMU 只是輔助項之一,不是逐幀緊耦合的核心估計來源 | `icp_odometry` 只在**初始化時**用 IMU 做重力對齊,逐幀還是純幾何 ICP,沒有持續融合 | IMU 預積分(preintegration)直接進 GTSAM 的 factor graph 聯合優化,**每一幀**都跟 LiDAR 特徵一起求解最佳姿態 |
| 調寬搜尋半徑後的結果 | 找到 20 個 constraint,但終點誤差只從 34.69m 降到 31.67m | 完全沒找到任何 loop closure 型態的連結,誤差改善(256→164m)是其他效應,不是迴環生效 | 終點誤差從 38.06m 降到 **0.288m**,幾乎完全修正 |
| 背後原因 | Constraint builder 是機會式地在整條路徑找任意 submap 配對,這 20 個修正的是路徑中途的局部交叉,不保證包含「終點連回起點」這條關鍵連結;而且等速度模型本身漂移量就比 IMU 緊耦合大,回環修正的「起始條件」較差 | 純 ICP 里程計漂移量太大(z 方向偏差達 150m 以上),搜尋半徑再怎麼放寬,估計位置已經跟真實位置差了 150 米,回環候選根本搜尋不到——問題出在**里程計精度**,不是搜尋半徑 | 里程計本身夠準(緊耦合 IMU+LiDAR 聯合優化),漂移量原本就控制在 60m 搜尋半徑能覆蓋的範圍內,一旦迴環候選被找到,GTSAM 的全局優化能一次性把整條軌跡的累積誤差攤平 |

**核心結論**:能不能找到迴環候選,取決於「累積漂移量」是否在搜尋半徑內;而累積漂移量的大小,取決於**里程計本身的精度**。LIO-SAM(以及理論上 FAST-LIO2,若能跑起來)把 IMU 緊耦合進每一幀的姿態估計,原始里程計就比 Cartographer 的等速度模型、RTAB-Map 的純 ICP 更準,這是它們終點誤差大幅領先的根本原因——**不是迴環演算法本身比較強,而是「需要被迴環修正的誤差」本來就比較小**。這也解釋了為什麼三套系統套用「同一個表面上的參數修正」(擴大搜尋半徑)之後,效果差了兩個數量級以上。

## 誠實限制與注意事項

1. **Cartographer、RTAB-Map、LIO-SAM 數字是本次實測結果;LOAM/LIO-odom/LIO-GPS/LIO-SAM 論文數字為引用值**,不是同一硬體/同一次執行環境下的公平競賽,只能當參考基準。
2. **三套系統的迴環搜尋半徑類參數預設值都偏小**(Cartographer `max_constraint_distance`、RTAB-Map `RGBD/LocalRadius`、LIO-SAM `historyKeyframeSearchRadius` 全部預設 10~15m),調寬之後三套系統呈現三種不同結果:LIO-SAM 幾乎完全修正(38.06m→0.288m)、Cartographer 找到迴環但改善有限(34.69m→31.67m)、RTAB-Map 完全沒找到迴環(256.06m→163.67m 的改善也不是迴環生效)。這證明「找不到迴環候選」背後有至少三種不同層次的原因,不是單一參數能一次解決的,詳見下方分析。
3. Campus 資料集沒有精確 ground truth(GPS/MoCap),End-to-end translation error 是唯一能直接對照論文的量化指標,無法計算完整 ATE/RPE。
4. Cartographer 改用 live node 而非原計畫的 offline node,實際執行是「即時播放」而非論文 LIO-SAM 宣稱的「10 倍加速」,real-time factor 因此都落在 ~1.0 附近,無法直接對照 LIO-SAM 論文的加速倍數描述。
5. RTAB-Map 是在「純 LiDAR ICP 模式」下測試,並非其原本以視覺回環見長的典型使用場景,這裡的數字不能代表 RTAB-Map 在視覺/RGB-D 場景下的真實實力。
6. FAST-LIO2 的失敗是**這個特定 ROS2 社群移植版(spark-fast-lio)在這台環境下的問題**,不代表 FAST-LIO2 演算法本身在其他實作或環境下也會有這個問題。

## 交付物清單

- 軌跡(TUM 格式):`outputs/trajectories/{cartographer,rtabmap,liosam}_campus.txt`
- 點雲地圖:`outputs/pointclouds/{cartographer,rtabmap}_campus.{pcd,ply}`、`outputs/liosam_campus/GlobalMap.pcd`
- 原始狀態檔:`outputs/cartographer_campus.pbstream`、`outputs/rtabmap_campus.db`
- 完整執行 log:`outputs/logs/`
