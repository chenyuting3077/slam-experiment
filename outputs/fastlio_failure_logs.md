# FAST-LIO2(spark-fast-lio)失敗紀錄 — 三次獨立跑的關鍵 log 摘要

三次都是**獨立跑**(沒有跟其他系統同時搶 TF/資源),排除環境污染的可能性,純粹測試參數修正是否有效。三次都在跑到一半後永久性地開始每幀 "No Effective Points!" / "No point, skip this scan!",再也沒恢復。

---

## v3 — 預設參數(filter_size_map=0.3, cube_side_length=1000.0)

```
FASTLIO_V3_START=1789755238.557858274
FASTLIO_V3_END=1789756234.313267806
WALL_CLOCK=995.755409532
```

失敗次數:**17793** 次 "No Effective Points!" / "No point, skip this scan!"

關鍵行:
```
line 19  [WARN] [1789755239.305102617] No point, skip this scan!          <- 開機瞬間正常的暫態,可忽略
line 96  [WARN] [1789755417.342517138] No point, skip this scan!          <- t+178s,第一次零星失敗
line 98  [WARN] [1789755450.686571040] No point, skip this scan!          <- t+212s,零星失敗
line 2098 [WARN] [1789755510.750658320] No Effective Points!             <- t+272s,開始密集/永久失敗
```
同一次跑裡也出現過 PCL 內部溢位訊息(其他次也有,直接證據):
```
[pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.
```
→ 懷疑是全域地圖累積範圍太大,對 `filter_size_map=0.3` 的 voxel grid 做 32-bit index 計算時溢位。

---

## v4 — 修正嘗試 1:調大 filter_size_map(0.3 → 1.0)

```
FASTLIO_V4_START=1789756348.089537384
```

失敗次數:**5520** 次 —— 比 v3 少,但**更早發生**(反而更快壞掉)。

關鍵行:
```
line 19  [WARN] [1789756349.569806492] No point, skip this scan!          <- 開機瞬間,正常
[INFO] Gravity alignment complete! ...                                    <- t+5.7s,初始化完成
line 84  [WARN] [1789756456.370919774] No Effective Points!               <- t+108s,開始永久失敗(比 v3 的 272s 更早)
line 85  [WARN] [1789756456.374042179] No Effective Points!
```
這次**沒有**出現 PCL VoxelGrid 溢位訊息——調大 leaf size 確實解決了溢位本身,但底層的地圖/mapping 崩潰問題沒解決,只是換了個症狀更早出現。

---

## v5 — 修正嘗試 2:調大 cube_side_length(1000.0 → 5000.0),filter_size_map 維持 1.0

```
FASTLIO_V5_START=1789756595.176846986
```

失敗次數:**23767** 次 —— 三次裡最多。

關鍵行:
```
line 15  [WARN] [1789756584.139119215] IMU loopback, clearing buffers      <- 開機瞬間暫態(正常,節點啟動時的queue reset)
line 16  [ERROR][1789756584.181660993] Lidar loopback detected, clearing buffers
line 17  [WARN] [1789756584.187279677] No point, skip this scan!
[INFO]   [1789756588.608337859] Gravity alignment complete! ...            <- t+4.5s,初始化完成
line 2823 [WARN][1789756621.612037486] No Effective Points!                <- t+37s(相對重力對齊完成後),開始永久失敗
```
懷疑「region cube 重新置中」邏輯的假設也被推翻——調大 cube_side_length 讓失敗**更早**出現,而不是更晚,說明根本原因不是 cube 邊界問題。

---

## v6 — 修正嘗試 3:調大 IMU noise(acc_cov/gyr_cov 0.1→0.4,b_acc_cov/b_gyr_cov 0.0001→0.001),cube_side_length 改回 1000.0

假設:LIO-SAM(另一套緊耦合 IMU 的系統)在同一份資料上也不穩定,懷疑是 IMU noise 參數沒有針對這顆真實感測器校正,讓濾波器太相信 IMU。檢查原始碼確認 `lasermap_fov_segment`(cube 搬移邏輯)其實是標準 FAST-LIO2 演算法,不是移植版的 bug——先前調大 cube_side_length 反而讓單次搬移距離(mov_dist)更誇張,方向錯了,cube_side_length 改回業界常用的 1000.0。

結果:**更快壞掉**——約 **20 秒**就開始永久性 "No Effective Points!",比 v3(272s)、v4(108s)、v5(37s)都早,呈現明確的單調惡化趨勢(272→108→37→20)。這個結果**反證**了 IMU noise 假設,已改回原始參數值。

## 結論

四次修正嘗試(調大 filter_size_map、調大 cube_side_length、調大 IMU noise)都**沒有解決**根本問題。特別值得注意的是後三次呈現明確的單調惡化趨勢(272s → 108s → 37s → 20s 就開始永久失敗),說明這幾個參數調整方向都是**錯的**,而不是「調得不夠」。已確認 cube 搬移邏輯是標準 FAST-LIO2 演算法(非移植版 bug)。這暗示真正的根因可能在這個第三方 ROS2 port(`MIT-SPARK/spark-fast-lio`)的 ikd-tree 地圖維護或多執行緒同步邏輯裡有更深層的 bug,需要對照原始 FAST-LIO2(ROS1)或其他 ROS2 port 逐行比對差異才有機會抓到,超出本次任務的除錯範圍,已放棄。

完整原始 log 檔案:
- `outputs/logs/fastlio_full_run_v3.log`
- `outputs/logs/fastlio_full_run_v4.log`
- `outputs/logs/fastlio_full_run_v5.log`
