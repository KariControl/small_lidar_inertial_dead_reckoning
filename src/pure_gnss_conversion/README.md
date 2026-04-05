# pure_gnss_conversion（standalone / bringup 統合版）

元の `gnss_ins_localizer` から `pure_gnss_conversion` だけを切り出し、以下の用途に絞って独立動作するように整理した版です。

- `sensor_msgs/msg/NavSatFix` → `geometry_msgs/msg/PoseStamped`
- ublox の `fix_velocity`（GNSS Doppler）→ 標準 `nav_msgs/msg/Odometry`
- 車輪速依存を削除
- `localization_msgs` 依存を削除
- `fix` と `fix_velocity` を **メッセージバッファで時刻同期**

## 入力

- `fix` (`sensor_msgs/msg/NavSatFix`)
- `fix_velocity` (`geometry_msgs/msg/TwistWithCovarianceStamped`)

launch では既定で次へ remap しています。出力は `/localization/global_pose` / `/localization/gnss_odometry` / `/localization/gnss_fusion_input` に remap されます。

- `/ublox_gps_node/fix`
- `/ublox_gps_node/fix_velocity`

## 出力

- `global_pose` (`geometry_msgs/msg/PoseStamped`)
  - `NavSatFix` をガウス・クリューゲル投影してローカル座標へ変換
  - GNSS アンテナ TF を使って **常に `map -> base_link`** へ変換
  - Doppler heading が無効なときは、前サンプル heading（起動直後は `initial_heading_deg`）で補正

- `gnss_odometry` (`nav_msgs/msg/Odometry`)
  - `pose.pose` に **`map -> base_link`** の位置と heading
  - `twist.twist.linear.x` に水平速度[m/s]
  - `twist.covariance[0]` に速度分散
  - `pose.covariance[35]` に yaw 分散

- `gnss_fusion_input` (`pure_gnss_msgs/msg/GnssFusionInput`)
  - `fix_status` に GNSS 品質状態を格納
  - `has_odom=true` のとき、同期済み `gnss_odometry` を内包
  - `has_odom=false` のとき、品質状態だけを流すので no-fix / stale 監視に使えます

## 同期方法

`message_filters` は使わず、ノード内部で `fix` / `fix_velocity` を時刻付きバッファへ保持し、
**`header.stamp` の差が `sync_tolerance_sec` 以内の最も近い組**をペアにして publish します。

- ペアが見つかったときだけ `global_pose` / `gnss_odometry` / `gnss_fusion_input(has_odom=true)` を publish
- ペア未成立のデータはバッファに保持
- `buffer_retention_sec` を超えた未使用データは破棄
- 同じメッセージは 1 回だけ使用

`sync_tolerance_sec` は **GNSS 更新周期の 1/2 より小さめ** にしておくと、
前後 epoch の取り違えを避けやすいです。

## 元コードからの主な修正点

1. **車輪速依存を削除**
   - 元コードは `wheel_velocity_ > wheel_velocity_th_` 条件のため、車輪速を使わない構成だと Doppler 速度/heading が常に無効になりやすい状態でした。
   - 本版は **GNSS Doppler 自身の速度と共分散だけ** で heading 更新可否を判定します。

2. **fix / fix_velocity をバッファ同期に変更**
   - 以前の standalone 版は「最後に来た値」を寄せ集めて publish していました。
   - 本版は **時刻差が敷居値以内の組だけ** を使うため、位置と Doppler の epoch が揃います。

3. **共分散の扱いを修正**
   - 元コードは `TwistWithCovarianceStamped` の共分散をそのまま「誤差」として扱っていました。
   - 本版は速度分散・heading 分散を **誤差伝播** で計算します。
   - `covariance[1]` / `covariance[6]` の東西-南北の相関も反映します。

4. **Y 方向共分散参照バグを修正**
   - 元コードは東西・南北の両方で `covariance[0]` を参照していました。
   - 本版は `covariance[0]` と `covariance[7]` を正しく使い分けます。

5. **出力 timestamp を入力 stamp 準拠に修正**
   - 元コードは出力 stamp に `now()` を使っていました。
   - 本版は **同期ペアの `NavSatFix` stamp** を出力へ使います。

6. **heading 無効時の lever-arm 補正を連続化**
   - Doppler heading が無効なときも、GNSS アンテナ位置をそのまま `base_link` として出さず、
     **前サンプル heading（起動直後は `initial_heading_deg`）** を使って
     `T_map_base = T_map_antenna * T_antenna_base` を計算します。
   - これにより、heading 有効/無効の切り替わりで `gnss_odometry` の座標系が飛ぶことを防ぎます。

7. **未初期化値依存を解消**
   - 元コードは wheel speed / heading / pdop が未初期化のまま使われ得る状態でした。
   - 本版は未受信時の状態を明示的に管理し、未受信でも安全に動くようにしています。

8. **標準メッセージへ置き換え**
   - 元の `localization_msgs/msg/GnssConv` は使わず、`PoseStamped` と `Odometry` に置き換えました。

## ビルド

```bash
colcon build --packages-select pure_gnss_msgs pure_gnss_conversion
source install/setup.bash
ros2 launch pure_gnss_conversion conversion_run.py
```

## パラメータ

`param/param.yaml` を参照してください。

特に調整しやすい項目:

- `p0`, `gnss0`: ローカル原点
- `min_x`, `min_y`: 既存マップ座標との差分補正
- `doppler_min_speed_mps`: heading 更新を許す最低速度
- `doppler_speed_sigma_th`: Doppler 速度標準偏差の上限
- `heading_sigma_deg_th`: heading 標準偏差の上限[deg]
- `initial_heading_deg`: 起動直後に有効な heading がまだ無いときの初期 heading[deg]
- `sync_tolerance_sec`: `fix` と `fix_velocity` を同一 epoch とみなす許容時間差[s]
- `buffer_retention_sec`: 未マッチデータを保持する最大時間[s]
- `max_buffer_size`: バッファ最大件数

## 補足

- `global_pose` / `gnss_odometry` は **同期ペア成立時のみ** 出ます。
- `gnss_fusion_input` は `fix` ごとに出て、同期済み pose がある epoch だけ `has_odom=true` になります。
- heading が無効な Doppler サンプルでも、前サンプル heading（起動直後は `initial_heading_deg`）を使って lever-arm 補正を継続します。
- その間の yaw 共分散は大きく保ち、fusion 側では yaw を無効扱いできます。
- `min_x`, `min_y` を使わない場合は `subtract_min_offset: false` か、両方を `0.0` にしてください。
