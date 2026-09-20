# Cartographer 3D 失敗原因調查

## 背景

Cartographer 3D 在這個專案的 4 個 Mid360 資料集來源(Zenodo `outdoor_hard_01`、`outdoor_kidnap`、TIERS 的 `OutdoorRoad_cut0`/`cut1`/`IndoorOffice1`/`IndoorOffice2`,共 6 個獨立序列)上,全部都在播放開始後幾秒內就發散——姿態飄到數千萬公尺外,pose graph/submap 日誌完全沒有報錯,難以事先察覺。嘗試調整 `TRAJECTORY_BUILDER_3D.imu_gravity_time_constant`(預設 10s)兩個方向都沒解決:調小(→1s)讓 submap 插不進去,調大(→30s)發散得更快。

在測試 VLP-16 的 `garden_dataset`(LIO-SAM 官方 demo bag)時,Cartographer 3D **直接硬崩潰**(不是靜默發散),留下完整的 stack trace 跟明確的錯誤訊息,才第一次抓到問題的確切根源。

## 崩潰現場

```
[cartographer_node] local_trajectory_builder_3d.cc:149] IMU not yet initialized.
[cartographer_node] local_trajectory_builder_3d.cc:159] Extrapolator is still initializing.
[cartographer_node] pose_graph_3d.cc:136] Inserted submap (0, 0).
[cartographer_node] F imu_tracker.cc:68] Check failed: (orientation_ * gravity_vector_).normalized().z() > 0.99 (0 vs. 0.99)
[cartographer_node]     cartographer::mapping::ImuTracker::AddImuLinearAccelerationObservation()
[cartographer_node]     cartographer::mapping::PoseExtrapolator::AdvanceImuTracker()
[cartographer_node]     cartographer::mapping::PoseExtrapolator::ExtrapolateRotation()
[cartographer_node]     cartographer::mapping::PoseExtrapolator::ExtrapolatePose()
[cartographer_node]     cartographer_ros::Node::PublishLocalTrajectoryData()
process has died [pid 274723, exit code -6, ...]
```

崩在播放開始後約 27 秒——同一段時間,`/imu_correct` 的原始讀數顯示加速度計幅值飆到 11+ m/s²(遠高於重力 9.8)、角速度衝到 0.5 rad/s(一個急轉彎),是**真實的劇烈運動**,不是壞資料或雜訊。

## 原始碼追出的機制

用 `apt-get source ros-jazzy-cartographer` 抓官方原始碼(版本 2.0.9004),對照 `cartographer/mapping/imu_tracker.cc`:

```cpp
void ImuTracker::AddImuLinearAccelerationObservation(
    const Eigen::Vector3d& imu_linear_acceleration) {
  // 用指數移動平均更新 gravity_vector_,平滑速度由 imu_gravity_time_constant 控制
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);
  gravity_vector_ =
      (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
  // 用當前的 gravity_vector_ 反推姿態
  const Eigen::Quaterniond rotation = FromTwoVectors(
      gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());
  orientation_ = (orientation_ * rotation).normalized();
  CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
  CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);  // <- 這裡崩潰
}
```

以及 `cartographer/mapping/pose_extrapolator.cc`:

```cpp
void PoseExtrapolator::AdvanceImuTracker(const common::Time time,
                                         ImuTracker* const imu_tracker) const {
  ...
  // 把佇列裡累積的所有 IMU 訊息,從上次的時間點到現在,一次性 replay 過一遍
  while (it != imu_data_.end() && it->time < time) {
    imu_tracker->Advance(it->time);
    imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);
    imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
    ++it;
  }
  ...
}
```

`AdvanceImuTracker` 在每次姿態外推(`ExtrapolatePose`/`ExtrapolateRotation`,即時運作下高頻率呼叫)時,把佇列裡累積的 IMU 訊息一次性 replay。這整套機制的隱含假設是:**加速度計讀數大部分時間都接近純重力**,只有在這個前提下,把它直接拿來做指數移動平均、當作重力方向的估計,才是合理的。

`gravity_vector_` 這個內部狀態,本質上是「目前估計的重力方向,表示在 IMU 本體座標系下」。每次新的加速度讀數進來,就用 `alpha`(由 `imu_gravity_time_constant` 決定平滑速度)把它混進來。當感測器承受**真實的、非重力方向的劇烈加速度**(急轉彎、快速啟停)時,這個估計值會被拖離真正的垂直方向。如果偏離得夠多,`FromTwoVectors` 算出來的姿態校正跟已經累積的姿態狀態不再自洽,就會直接踩到第 68 行那個斷言,整個程序 `abort()`。

## 崩潰 vs. 靜默發散,是同一個根因的兩種呈現方式

- **VLP-16(garden_dataset)**:走路時的一個急轉彎,剛好讓數值踩到 `CHECK_GT(..., 0.99)` 這條邊界,直接崩潰、留下明確的錯誤訊息。
- **Mid360(6 個資料集)**:官方標示的「快速運動」情境,觸發的是同一個機制,但數值沒有剛好踩到那條斷言的邊界——`gravity_vector_` 被拖偏,反推出一個「數學上合法但物理上錯誤」的姿態,沒有觸發 CHECK,程序不會崩潰、也不會報錯,但姿態外推器從此開始用這個錯誤的姿態繼續推算,一步步二次方式發散,幾分鐘內飄到數千萬公尺外。

這也解釋了為什麼調 `imu_gravity_time_constant` 兩個方向都沒用:這個參數只控制指數移動平均的**平滑速度**,並不會修正「劇烈運動時,加速度計讀數本來就不該直接當重力向量用」這個根本的建模假設缺陷。調小讓平滑跟不上真實運動的變化速度;調大則讓一次被拖偏的估計值花更久才能被真實重力訊號洗回來——兩個方向都只是在同一個脆弱機制裡調整暴露的時間點跟嚴重程度,無法根治。

## 第二次獨立確認:純旋轉(沒有明顯線性加速度)也會觸發,而且是另一條斷言

在同一個 Google Drive 資料夾裡另一份官方 demo bag `rotation_dataset.bag`(58.6 秒,官方設計成「原地快速旋轉」測試,幾乎沒有平移,最大角速度 3.73 rad/s,約 214°/s)上,Cartographer **又崩潰了**,但這次踩到的是同一支程式碼裡的**另一條斷言**:

```
F imu_tracker.cc:67] Check failed: (orientation_ * gravity_vector_).z() > 0. (-1.02958e-86 vs. 0)
```

第 67 行(`CHECK_GT((orientation_ * gravity_vector_).z(), 0.)`)比第 68 行的容忍度更寬鬆(只要求同向,不要求對齊到 0.99 的餘弦相似度),但這次算出來的值是**幾乎精確的 0**(浮點雜訊等級的 `-1.03e-86`)——代表反推出來的重力方向跟垂直軸完全垂直,是徹底退化的結果,不是「有點偏」而已。

這確認了一個重要的細節:**觸發這個 bug 不需要真實的線性加速度衝擊,單純足夠劇烈的旋轉就夠了**。因為 `ImuTracker::Advance()` 每次都用當前累積的角速度去旋轉 `gravity_vector_`(`gravity_vector_ = rotation.conjugate() * gravity_vector_`),旋轉越劇烈、旋轉之間累積的次數越多,這個內部狀態就越容易被帶到跟真實重力方向完全脫節的位置。garden_dataset 的急轉彎(線性加速度 11+ m/s²、角速度 0.5 rad/s)跟 rotation_dataset 的快速原地旋轉(角速度 3.73 rad/s,線性加速度反而更平緩)分別從「加速度計混入太多非重力訊號」跟「角速度累積旋轉太劇烈」兩個不同路徑,踩到了同一套機制裡的兩條不同斷言——但根因是同一個:`ImuTracker` 沒有任何機制偵測「目前這個重力估計已經不可信」,只會在數值徹底不自洽時用斷言崩潰收場。

同一次測試裡,RTAB-Map 的 `icp_odometry` 也再次失去追蹤(`libpointmatcher` 回報旋轉量 0.64~0.93 rad 超出 `Icp/MaxCorrespondenceDistance` 對應的旋轉限制 0.78 rad),整段 58.6 秒只留下 1 個關鍵幀——跟 garden_dataset 的失敗模式完全一致,进一步確認「劇烈旋轉本身」(不需要搭配線性加速度)就足以讓 frame-to-model ICP 完全失能。LIO-SAM(249 個姿態)跟 FAST-LIO2(57 個姿態)都完整跑完,但因為這是近乎原地旋轉、路徑長度只有 ~12.6m,端到端誤差(0.44m / 0.54m)相對路徑長度的比例明顯比 garden_dataset 的大迴圈(誤差 <6cm、相對誤差 <0.02%)差很多——說明**純旋轉對所有系統的角度追蹤精度都是一種額外壓力,只是沒有讓 LIO-SAM/FAST-LIO2 徹底失去追蹤而已**。

## 交叉驗證:同一個急轉彎,也讓 RTAB-Map 永久失去追蹤

同一次 `garden_dataset` 測試裡,RTAB-Map(純 ICP,`icp_odometry`)在完全獨立的機制下,對同一個事件做出了不同但同樣是失敗的反應。log 顯示:

```
OdometryF2M.cpp:622::computeTransform() Registration failed: "libpointmatcher has failed: limit out of bounds: rot: 0.291957/0.78 tr: 2.70239/2"
```

在 t≈15-27s 這段窗口,ICP 註冊持續失敗(旋轉/平移量超出 `Icp/MaxTranslation=2` 的限制),`icp_odometry` 從此進入「追蹤丟失」狀態,`publish_null_when_lost=true` 讓它從那之後只發布空的姿態,RTAB-Map 的 SLAM 節點因此整段 362 秒的錄製只收到 1 個有效關鍵幀(`WM=1` 到結束都沒變),之後每一幀點雲都被記錄成「no odometry is provided, Image 0 is ignored」。

這代表**同一個真實世界的急轉彎事件,分別以兩種完全不同的機制,讓兩套架構迥異的系統雙雙失效**:Cartographer 的 IMU 重力假設被打破後觸發硬斷言崩潰;RTAB-Map 的 frame-to-model ICP 找不到合理的配準解後永久丟失追蹤、沒有重定位機制挽回。LIO-SAM(GTSAM 因子圖 + IMU 預積分)跟 FAST-LIO2(ESKF 緊耦合)在同一段資料上完全沒有受影響,端到端誤差都在 6 公分以內——再次印證這四套系統對「劇烈運動」的容錯能力,取決於各自完全不同的架構假設,不是任何單一參數能調出來的差異。

## 試過但沒用:完全不接 IMU

既然根因是 IMU 重力追蹤機制,一個直覺的想法是:能不能乾脆不要餵 IMU 給 Cartographer,讓它純粹靠 LiDAR scan-matching 跑?3D 的 `TRAJECTORY_BUILDER_3D` 沒有像 2D 版本那樣的 `use_imu_data` 開關,但理論上如果 IMU 訂閱的 topic 完全沒有任何 publisher,`imu_data_` 佇列會一直是空的,`PoseExtrapolator::AdvanceImuTracker` 應該會走進「沒有 IMU 資料」的分支,用固定的假重力向量 `Eigen::Vector3d::UnitZ()` 取代真實讀數——理論上永遠不會踩到那個斷言。

實測(`cartographer_mid360_noimu.launch.py`,故意不 remap `imu` topic,在 TIERS `OutdoorRoad_cut1` 上測試,用獨立的 `ROS_DOMAIN_ID` 避免跟其他工作衝突):**不會崩潰,但也完全不會動**。整個 45 秒的播放期間,`cartographer_node` 只用了 1 秒的 CPU 時間,沒有任何 `/tf` 輸出,pose graph 沒有任何 submap 被插入。

原因是 Cartographer 用一個 `ordered_multi_queue` 機制確保多個感測器來源的訊息按時間正確排序處理——它會等到**每一個已註冊的感測器 topic 都至少收到一筆訊息**之後,才會開始釋放任何資料出去處理(前面幾次崩潰的 log 都能看到這一行:`ordered_multi_queue.cc:172] All sensor data for trajectory 0 is available starting at ...`,代表這個等待機制真實存在)。IMU topic 永遠沒有 publisher,這個條件永遠不會滿足,所以連點雲都不會被處理——不是「繞過 IMU 用純 LiDAR 跑」,是整個 pipeline 直接卡死。這條路不通,已停止嘗試,列為跟調整 `imu_gravity_time_constant` 一樣「試過但沒用」的方向。

## 結論

這是 Cartographer 3D(至少到 2.0.9004 這個版本)IMU 初始化/重力追蹤邏輯本身的限制,不是我們的設定或參數問題:它的重力估計機制假設感測器大部分時間接近靜止或勻速,一旦平臺出現真實的劇烈加速度或轉彎,就沒有任何容錯機制——輕則靜默發散,重則直接斷言崩潰。四套系統裡,LIO-SAM、FAST-LIO2、RTAB-Map 都用不同方式(GTSAM 因子圖的 IMU 預積分、ESKF、純幾何 ICP)處理 IMU 訊號,沒有這種對加速度計讀數的強假設,因此在同樣的劇烈運動場景下都不會發散。
