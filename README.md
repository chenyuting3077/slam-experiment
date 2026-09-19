# SLAM 系統比較實驗

用四套 SLAM 系統(Cartographer 3D、RTAB-Map、LIO-SAM、FAST-LIO2)在兩份不同感測器、不同性質的資料集上跑,比較精度跟強健性。兩份資料集的細節、格式差異、跟四套系統各自的移植過程,見 [DATASETS.md](DATASETS.md)。

## 資料集一:Campus(LIO-SAM 官方資料集,Velodyne VLP-16)

閉環路徑,1437m,用「終點誤差」(首尾姿態距離)對照論文數字。

| 指標 | LOAM(論文) | LIO-odom(論文) | LIO-GPS(論文) | LIO-SAM(論文) | **Cartographer 3D** | **RTAB-Map** | **LIO-SAM** | **FAST-LIO2** |
|---|---|---|---|---|---|---|---|---|
| End-to-end translation error (m) | 192.43 | 9.44 | 6.87 | 0.12 | 31.67 | 163.67 | 0.288 | 9.575 |

詳細分析、迴環偵測除錯過程、為什麼緊耦合系統明顯領先,見 [outputs/comparison_report.md](outputs/comparison_report.md)。

![Campus 四套系統比較圖](docs/images/campus/comparison_map_grid.png)

## 資料集二:Mid360 outdoor_hard_01(Zenodo,Livox Mid360)

非閉環路徑(起終點相距 18.4m),998.9m,官方資料集本身設計成「快速運動、點雲劣化」的困難場景,附完整連續 ground truth(5147 筆),用 ATE(絕對軌跡誤差,Umeyama SE3 對齊後算 RMSE)評估。

| 系統 | 狀態 | ATE RMSE | 軌跡長度 | 備註 |
|---|---|---|---|---|
| Cartographer 3D | ❌ 發散 | — | — | 即時播放下,`odom->livox_frame` 的姿態外推器很快開始二次方式發散(幾分鐘內飄到數千萬公尺)。`imu_gravity_time_constant` 調小(→1s)讓 submap 插不進去,調大(→30s)發散得更快(95秒內飄到 -6200 萬公尺),兩個方向都沒用,判定不是這個參數能解的問題,姿態外推器本身無法處理這段路的劇烈運動,列為已知限制 |
| RTAB-Map(純 ICP) | ⚠️ 部分成功 | **0.944m** | 170.95m(全長 998.9m 的 17%) | ATE 數字看起來很漂亮,但只是因為它在追蹤到 170m 左右就停止累積新的節點(跟 log 裡觀察到的 ICP 註冊間歇性失敗一致)——不是穩健地跑完全程,是提早放棄後剩下的一小段track 得準 |
| LIO-SAM | ⚠️ 前段正確,後段發散 | 520.76m(全段,已失去意義) | 前 69 秒/204m 正確 | 用社群 fork `rajvishnu07/lio_sam_mid360`(ROS2 Humble)+ 修正兩個 bug(TF 空 frame_id、時間欄位單位錯誤)後,前 20% 的路徑姿態合理,但 t≈985s 開始跳動幅度一路擴大到 20m+,最終飄到 z=-1000m 級的離群值。推測跟這個 fork 用垂直角度湊出的人工「ring」在快速運動下抽出壞特徵有關,壞的因子被序列式地加進 GTSAM 因子圖後回不去 |
| FAST-LIO2 | ✅ 成功 | **5.228m** | 968.41m(全長 998.9m 的 97%) | 官方 ROS1 版,寫了轉檔腳本把 PointCloud2 還原成真正的 `livox_ros_driver::CustomMsg` 餵給官方未修改的 `mid360.yaml`。沒有迴環偵測,純里程計在這段「hard」序列上飄了 5.2m,但完整跑完了 97% 的路徑長度 |

**這次最有意思的發現**:RTAB-Map 的 ATE 數字(0.944m)比 FAST-LIO2(5.228m)漂亮,但那是因為它只成功追蹤了 17% 的路徑就停滯,LIO-SAM 也是同樣的模式(前 20% 正確、之後完全發散)——都不是「更準」,是「提早放棄/失去追蹤」。單看 ATE 數字會誤導,必須同時看軌跡覆蓋率才能公平比較。**四套系統裡只有 FAST-LIO2 完整跑完了整段路**:它沒有迴環偵測,累積誤差確實最大,但也因此不受「因子圖被污染後回不去」「ICP 找不到配對就整段卡住」這類問題影響,是這次測試最穩健的系統。

![Mid360 四套系統比較圖(Cartographer 發散、RTAB-Map/LIO-SAM 圖中只畫發散/停止前成功追蹤的那一段、FAST-LIO2 完整跑完)](docs/images/mid360/comparison_map_grid_mid360.png)

## 目錄結構

- `DATASETS.md` — 兩份資料集的特性、格式差異、四套系統移植細節
- `outputs/comparison_report.md` — Campus 資料集的完整分析報告
- `docs/images/` — 靜態地圖渲染圖(campus/、mid360/)
- `slam_comparison/scripts/` — 所有前處理、匯出、渲染、評估腳本
- `slam_comparison/config/`、`slam_comparison/launch/` — 各系統的設定檔跟 launch file
