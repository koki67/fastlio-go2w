# Plan

- Created: 2026-07-22T17:20:48+09:00
- Snapshot: 2026-07-22T17:20:48+09:00
- Status: final
- Language: ja
- English translation: `2026-07-22_172048_plan_fastlio-xt16-go2w-imu.en.md`
- Session: unavailable
- Branch: `main` (`a8180308f7a15dcb7412eb5b458b0792db6d6220`) → `feat/xt16-go2w-imu`
- Workspace: `/home/user/ws/fastlio-go2w`
- Scope: Pandar XT16の既存録画 `/points_raw` とGo2W内蔵IMU `/go2w/imu` だけでFAST-LIOをオフライン実行し、成果物と成立判定を残す。実機・ライブセンサ起動は対象外とする。

## Context and current state

- 現在の`main`はMID-360専用。既存の`feat/offline-dual-lidar-fusion`はMID-360＋XT16実験であり、今回のブランチにはマージしない。
- XT16点群は `x/y/z/intensity: FLOAT32`、`timestamp: FLOAT64`絶対秒、`ring: UINT16`を持つ。FAST-LIOのVelodyne入力は相対秒の`time: FLOAT32`を要求するため、専用変換が必要。
- `dlio-go2w`から採用するTFは以下。
  - `base_link→imu`: translation `[-0.02557, 0, 0.04232]`、回転なし
  - `base_link→hesai_lidar`: translation `[0.1634, 0, 0.116]`、Z軸+90°
  - FAST-LIO用`hesai_lidar→imu`: translation `[0.18897, 0, 0.07368]`、rotation matrix `[0,-1,0, 1,0,0, 0,0,1]`
- 実データ調査から実行可能性は高い。
  - `stair2`: XT16 940フレーム、IMU 46,990件、通常点時刻幅約100 ms、最近傍IMU差中央値0.577 ms／p95 1.143 ms
  - `long3`: XT16 11,962フレーム、IMU 597,701件、最近傍IMU差中央値0.486 ms／p95 0.991 ms
- 約7.2秒ずれた点群は既知のXT16ドライバ不具合。現データでの処理継続用に検出・破棄するが、これをFAST-LIO実装失敗とは扱わない。修正版ドライバによる新規データ取得後の再検証を別途必須とする。

## Decisions and constraints

- 時刻同期は固定オフセット方式とする。実行時自動推定は実装しない。
- 変換後LiDAR時刻を `raw_lidar_time + lidar_time_offset_sec` と定義し、Go2W IMU時刻は変更しない。FAST-LIO組み込みの`time_sync_en`と`time_offset_lidar_to_imu`は無効／0にして二重補正を防ぐ。
- 候補は `-10/-5/0/+5/+10 ms`。同一区間で比較し、連続性を悪化させず面厚p95を5%超改善した非ゼロ候補だけ採用する。それ以外は0 msを維持する。
- 元バッグは変更せず直接再生する。異常フレームは変換ノードで丸ごと破棄し、診断カウンタへ記録する。
- MID-360の既存デフォルト挙動、既存成果物形式、`main`、実験的dual-LiDARブランチを変更・統合しない。
- 新規録画、物理ロボット、Jetson、絶対精度はUNVERIFIEDのまま残す。現バッグにはground truthがない。
- 実装・検証後はローカルブランチへコミットする。pushやmergeは依頼があるまで行わない。

## Final plan

1. 実装開始前にこの最終版を `~/.codex/memories/rollout_plans/2026-07-22_172048_plan_fastlio-xt16-go2w-imu.md` と `docs/plans/2026-07-22_172048_plan_fastlio-xt16-go2w-imu.md` に同内容で保存し、新しいコンテキストで読み直す。リポジトリ状態を再確認後、`main`から`feat/xt16-go2w-imu`を作成する。
2. `config/sensor/go2w_xt16_calibration.yaml`を追加し、D-LIO由来のbase・IMU・XT16のTF、合成済みLiDAR→IMU変換、トピック、較正の由来を一元化する。Go2W URDFへ`hesai_lidar`固定リンクを追加し、既存の`imu`リンクと整合させる。
3. 新規ROS 2 C++パッケージ`fastlio_go2w_hesai`に`hesai_pointcloud_adapter`を実装する。
   - 入力 `/points_raw`、出力 `/points_raw_fastlio`
   - 出力フィールドは `x/y/z/intensity/time/ring`。`time`は元の各点絶対時刻から元ヘッダ時刻を引いた相対秒
   - 点を時刻順にstable sortし、ヘッダだけ固定オフセット分シフトする
   - endian、row padding、フィールド型、ring範囲、有限値を検証する
   - ヘッダ逆行、非有限点時刻、点時刻がヘッダから200 msを超えるフレームは全体を破棄する。無効XYZ・intensity・ringは点単位で除外し、残存点不足ならフレームを破棄する
   - `/fastlio_go2w_hesai/diagnostics`へ受信数、変換数、破棄理由別件数、無効点数、最新スキャン幅、適用オフセットを発行する
4. XT16＋Go2W IMU用FAST-LIO設定を追加する。
   - `lidar_type: 2`, `scan_line: 16`, `scan_rate: 10`, `timestamp_unit: 0`
   - `lid_topic: /points_raw_fastlio`, `imu_topic: /go2w/imu`
   - `extrinsic_est_en: false`と上記LiDAR→IMU固定変換
   - 初期値は`point_filter_num: 4`, `blind: 0.5 m`, `fov_degree: 360`, `det_range: 100 m`
   - Go2W IMU用の初期ノイズ値はD-LIO較正参照値を転記し、実測チューニング値ではないことをコメントする
5. 既存launchをセンサプロファイル対応にする。
   - `offline_fastlio.launch.py sensor:=mid360|xt16`を公開し、デフォルトは従来どおり`mid360`
   - `xt16`時だけ変換ノードを起動し、odom adapterのIMUフレームを`imu`へ切り替える
   - `lidar_time_offset_sec`をlaunch引数として公開する
   - ライブセンサdriverや`sensors.launch.py`にはXT16経路を追加しない
6. 既存オフラインrunnerへ `--sensor xt16` と `--lidar-time-offset-sec` を追加する。
   - XT16では元バッグから `/points_raw` と `/go2w/imu`だけを再生する
   - 変換ノード、FAST-LIO、odom adapter、recorderを必須プロセスとして監視する
   - 設定、実行パラメータ、較正、実行ファイルhash、アダプタ診断、採用オフセットをmanifestへ保存する
   - MID-360時のトピック、既定設定、成果物契約は維持する
7. 固定オフセット比較スクリプトと判定レポートを追加する。
   - `stair2`の`--start-offset 20`以降を同条件で5回実行する
   - 未完了、非有限値増加、サンプル被覆1%以上低下、gap/jump増加を候補失格とする
   - 合格候補のうち、0 msに対してlocal-plane thickness p95を5%超低減し、median thicknessとplanarityを悪化させない候補だけ採用する
   - 複数候補はp95面厚、絶対オフセット、数値順で決定する。該当なしなら0 ms
   - 比較JSON/CSVと各runへの参照を保存し、採用値をXT16既定設定とドキュメントへ反映する
8. テストと実バッグ検証を実施し、最終報告では「実行可能」「実行可能だが軌跡品質未確認」「実行不能」を分けて明記する。既知のドライバ異常による期待どおりのフレーム破棄は実装失敗に数えない。
9. 差分、生成物混入、既存MID-360回帰がないことを確認し、意味のまとまりごとにコミットする。ブランチはローカルに保持する。

## Validation

- C++単体テスト:
  - 正常なXT16 schema、absolute timestamp→relative `time`、時刻sort、ring保持、符号付き固定オフセット
  - big/little endian、row padding、欠落・重複・誤型フィールド
  - NaN/Inf、ring範囲外、ヘッダ逆行、200 ms超スキャン、空・極小クラウド
  - 異常フレームが出力されず診断カウンタが正しく増えること
- 設定テスト:
  - 較正YAML、URDF、FAST-LIO `extrinsic_R/T`の数値一致
  - XT16トピック・16ライン・秒単位設定
  - MID-360既定値と既存オフライン設定の不変性
- 静的・ビルド確認:
  - `bash -n`を変更したshellへ実行
  - `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`でPythonテスト
  - ROS 2 Humbleコンテナで対象パッケージを`colcon build`、`colcon test`
  - `git diff --check`
- 実データ:
  1. `experiment_stair2_20260713_115313`の20秒以降で短時間smoke run
  2. 同区間で5候補の固定オフセットsweep
  3. 採用値で`experiment_long3_20260714_014823`元バッグ全体をrate 1.0で実行
  4. `long3`では11,962入力中、既知異常7フレームが診断付きで破棄され、残りが処理されることを確認
  5. `manifest.json`, `summary.json`, `trajectory.csv`, map PCD、resource metrics、diagnosticsを保存
  6. 修正版XT16ドライバによる新規バッグ取得後、同じpreflight・sweep・長時間検証を再実行

## Acceptance criteria

- 既存録画だけからXT16点時刻を失わず、Go2W IMUとともにFAST-LIOへ入力できる。
- `stair2`の正常区間でrunnerがcompletedになり、有限な`/odom`と`/cloud_registered`、非空の地図成果物を生成する。
- 正常区間で非有限pose、0.2秒超gap、1 m超translation jump、15°超orientation jumpがない。
- `long3`元バッグを派生クリーニングなしで最後まで処理し、既知の7異常フレームを安全に隔離できる。
- オフセット選択が同一データ・同一基準で再現でき、採用理由がJSONに残る。
- MID-360の既存オフラインテストとデフォルト実行が回帰しない。
- 現データで成立しても絶対精度・実機運用可能とは断定せず、新規正常データ検証を未完了として残す。

## Risks / cautions

- 現バッグの約7.2秒異常は既知のdriver defectであり、アルゴリズム品質評価には使わない。
- ground truthがないため、D-LIOとの差や軌跡長をATE/RPEとして扱わない。
- 固定オフセットsweepの採用値は現データ上の暫定値。新規録画で再評価する。
- LiDAR→IMU変換の向きやZ軸+90°を逆転すると、FAST-LIOは起動しても地図が破綻するため、自動テストで固定する。
- フレームを部分利用すると取得窓が曖昧になるため、時刻異常はフレーム単位でfail-closedに処理する。
