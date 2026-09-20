# FAST_LIO_SAM(第 5 套系統)加入筆記

第 5 套加入比較的系統:[`kahowang/FAST_LIO_SAM`](https://github.com/kahowang/FAST_LIO_SAM)(社群 fork,`Front_end: fastlio2, Back_end: lio_sam`,把 LIO-SAM 風格的 GTSAM 迴環後端接到 FAST-LIO2 的前端上)。跑在 `fastlio_official` 這個既有的 ROS1 Noetic container 裡,額外裝了 GTSAM 4.0.3(`ppa:borglab/gtsam-release-4.0`,Focal 官方 apt 沒有)跟 `libgeographic-dev`(`CMAKE_MODULE_PATH=/usr/share/cmake/geographiclib` 才能讓 `find_package(GeographicLib)` 找到)。

## 真的原始碼 bug:per-point 時間欄位單位錯誤

`preprocess.cpp`(`velodyne_handler`)裡寫死:

```cpp
added_pt.curvature = pl_orig.points[i].time / 1000.0; // units: ms
```

假設輸入的 `time` 欄位是**微秒**,除以 1000 轉成毫秒存進 `curvature`(後面拿去做逐點動態去畸變)。但我們所有 VLP-16 資料集(LIO-SAM 官方 demo bag 系列)per-point `time` 欄位其實是**秒**(這件事在專案更早期處理官方 FAST-LIO2 時就已經驗證過,見 `campus_config/campus_velodyne.yaml` 裡的 `timestamp_unit: 0` 註解)。單位差了 100 萬倍,等於逐點去畸變的時間修正量被砍到幾乎是零,整個 scan 在做 scan-matching 時被當成「瞬間拍完」處理,完全沒有修正 VLP-16 旋轝掃描期間(每圈 0.1 秒)累積的真實運動。

修法:把兩個對應分支都改成 `* 1000.0`(改成把秒轉成毫秒),重新 `catkin_make`。

**確認有效**:`rotation_dataset`(58.6 秒,幾乎原地快速旋轉)重測,修正前軌跡總長 88.24m,修正後降到 17.27m,跟 LIO-SAM/FAST-LIO2 官方基準(~12.6m)非常接近——這是一個真實、有效的修正。

## 我自己犯的錯:拿錯資料集比較,誤判成另一個嚴重 bug

在 `campus_small_dataset.bag`(407.78 秒)上測,結果軌跡總長只有 546.5m,直覺拿去跟 README 裡記錄的 Campus「1437m / 端到端誤差 9.575m(FAST-LIO2)」比較,以為又是一個嚴重 bug(62% 位移低估)。

後來核對才發現:**README 裡所有既有系統(Cartographer/RTAB-Map/LIO-SAM/官方 FAST-LIO2)的 Campus 數字,全部都是用 `campus_large_dataset.bag`(989.67 秒,論文對應的 16.58 分鐘完整版)測出來的**,不是 `campus_small_dataset.bag`。這兩個是 LIO-SAM 官方 Google Drive 資料夾裡**兩個獨立的檔案**,small 只是一個更短的子集/獨立錄製版本,本來就不該有 1437m 那麼長的路徑。改用 `campus_large_dataset.bag` 重測後,軌跡總長 1432.52m(跟真實值幾乎完全吻合),端到端誤差 10.034m,跟官方 FAST-LIO2(9.575m)幾乎一樣——**FAST_LIO_SAM 在 Campus 上從頭到尾都沒有問題,是我自己拿錯了比較基準**。

這個教訓：以後每次幫新系統挑「跑哪個資料集檔案」時,一定要先核對既有系統的軌跡檔案時間戳範圍(`head -1`/`tail -1`),確認是同一個檔案,不能只看檔名相似就假設是同一份資料。

## 執行時間(real-time factor)

| 資料集 | 時長 | 實際 wall-clock 播放時間 | real-time factor |
|---|---|---|---|
| campus_small(第一次測,後來發現拿錯) | 407.78s | ~412.8s | ~1.01 |
| campus_large(正確版本) | 989.67s | ~1009.4s | ~1.02 |

跟其他四套系統一樣,大致貼著即時速度跑,沒有明顯的效能瓶頸。

## 目前狀態

- Campus(large,正確版本):✅ 10.034m,跟 FAST-LIO2 同等級。
- rotation_dataset:✅ 已用修正後的版本重測,17.27m 路徑長度(對照真實 ~12.6m),但端到端誤差仍有 7.8m(比官方 LIO-SAM/FAST-LIO2 的 <1m 差,但纯旋轉本身對所有系統都是壓力測試,見 `CARTOGRAPHER_FAILURE.md` 的討論)。
- garden、park、TIERS 四段、Mid360 三段:尚未用修正後的版本測試,是接下來要補齊的部分。
