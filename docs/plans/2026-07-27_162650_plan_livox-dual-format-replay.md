# Plan

- Created: 2026-07-27T16:26:50+09:00
- Snapshot: 2026-07-27T16:26:50+09:00
- Status: final
- Language: ja
- English translation: 2026-07-27_162650_plan_livox-dual-format-replay.en.md
- Session: 019fa23e-0621-7571-93e5-b63d55b77709
- Branch: main
- Workspace: /home/user/ws/fastlio-go2w
- Scope: MID-360 rosbag再生について、旧`livox_ros_driver2/msg/CustomMsg`と新`sensor_msgs/msg/PointCloud2`を後方互換で自動選択できるようにする。interactive RViz replayとheadless offline runnerの両方に同じ入力契約、診断、検証を適用する。XT16実験ブランチの統合、実機・Jetson検証、絶対軌跡精度の主張は対象外とする。

## Context and current state

- リポジトリは`/home/user/ws/fastlio-go2w`、現在のブランチは`main`、HEADは`a818030`で`origin/main`と一致している。作業ツリーにはユーザー所有の未追跡`results/`があり、変更・削除・commit対象に含めない。
- `scripts/fastlio/replay.sh`は`fastlio_go2w_bringup/replay.launch.py`を起動し、既定の`mid360_go2w.yaml`を使用する。設定は`common.lid_topic=/livox/lidar`、`common.imu_topic=/livox/imu`、`preprocess.lidar_type=1`である。
- FAST-LIOの`lidar_type=1`は`/livox/lidar`を`livox_ros_driver2/msg/CustomMsg`として購読する。旧bag`/mnt/data1/experimental_data/go2w-experiment-recorder/bags/experiment_long3_20260714_014823`はこの型であり、現経路と一致する。
- 新bag`/mnt/data1/experimental_data/go2w-experiment-recorder/bags/experiment_save-test_20260727_022320`は`/livox/lidar`を`sensor_msgs/msg/PointCloud2`として14,183件記録している。再生実測では`/livox/imu`は約200 Hzだが、PointCloud2 publisherとCustomMsg subscriberが接続できず、`/cloud_registered`と`/Odometry`は出力されなかった。3つのSQLite chunkはすべて`PRAGMA quick_check=ok`で、bag破損ではなく型契約不一致が直接原因である。
- 新しいLivox PointCloud2は、`x/y/z/intensity` FLOAT32、`tag/line` UINT8、`timestamp` FLOAT64を持つ。現driverはheader stampを`pkg.base_time`、各点timestampをabsolute nanoseconds、intensityを元のuint8 reflectivityから生成する。したがって`offset_time = round(point.timestamp) - header_stamp_ns`としてCustomMsgへほぼ逆変換できる。double化によるsub-microsecond量子化以上の精度は復元できないが、scan motion compensationに必要な点時刻は保持できる。
- FAST-LIOの既存MID-360 PointCloud2 handlerへ単純に`lidar_type=4`を渡す案は採用しない。現handlerは新フィールド`timestamp`を読まず角度から点時刻を再構成し、`intensity`ではなく`reflectivity`を期待するため、表示が出ても従来経路と同等の時刻契約にならない。
- ローカル`feat/xt16-go2w-imu`には`/points_raw`と`/go2w/imu`用のXT16 adapterが3 commit存在するが、安定したMID-360 `main`から意図的に分離されている。本実装ではこのbranchをmerge/cherry-pickせず、MID-360の同一トピックにおける旧・新メッセージ型の互換性だけを扱う。
- 実行はROS 2 Humble devcontainerを正とする。対象bagのcontainer内パスは`/mnt/go2w-experiment-recorder/bags/...`であり、ホストの`/mnt/data1/...`ではない。ホストのROS 2 Jazzy overlayは検証環境として使わない。

## Decisions and constraints

- 旧CustomMsg経路は処理内容を変えずに残す。旧bagでは変換adapterを起動しない。
- 新PointCloud2だけを専用境界adapterでCustomMsgへ正規化し、FAST-LIO本体のLivox CustomMsg preprocessingを再利用する。third-party FAST-LIOの`preprocess.cpp`へ新形式固有処理を追加しない。
- bag formatは再生開始前に`metadata.yaml`から判定する。実行中に同名トピックへ異型subscriberを同時生成して推測しない。
- CLIは`--lidar-format auto|custom-msg|pointcloud2`を提供し、既定は`auto`とする。明示指定がmetadataと矛盾する場合、不明型、対象トピック欠落、同名複数型、壊れたmetadataはfail closedで停止する。
- format判定はトピック名だけでなく、`/livox/lidar`の正確なROS型まで確認する。対応型は`livox_ros_driver2/msg/CustomMsg`と`sensor_msgs/msg/PointCloud2`だけとする。
- PointCloud2 adapterは固定26-byte strideに依存せず、field metadata、datatype、offset、count、endianness、`point_step`、`row_step`、organized cloud、row padding、data lengthを検証する。現在の26-byte layoutと、同じfield契約を持つpadding layoutの両方を許容する。
- 入力PointCloud2の必須fieldは`x/y/z/intensity` FLOAT32、`tag/line` UINT8、`timestamp` FLOAT64とする。header、全点の有限値、intensityの`0..255`範囲と整数性、timestampのheader相対範囲、uint32 offsetへの収まり、frame headerの単調性を検証する。critical anomalyを含むframeは部分利用せずframe単位で破棄し、理由別counterを出す。
- 各点timestampはabsolute nanosecondsとして`llround`し、header stamp nanosecondsを引いてCustomPoint `offset_time`へ変換する。同時刻の順序を保持したstable sortを使用し、並べ替えたframeを診断counterへ記録する。
- 変換後CustomMsgはheaderとtimebaseを入力headerから設定し、x/y/z、reflectivity、tag、lineを保持する。PointCloud2に存在しない`lidar_id`とreserved fieldは0とし、FAST-LIOが使用しないことをunit testとコード確認で固定する。
- adapter outputは`/livox/lidar_fastlio`という別トピックにし、同名異型endpointを作らない。PointCloud2 inputは`/livox/lidar`のままとする。FAST-LIOの`common.lid_topic`だけをlaunch parameter overrideで切り替える。
- adapterは受信・変換・drop理由・並べ替え・最新scan幅を`diagnostic_msgs/msg/DiagnosticArray`で公開する。offline実行では最終counterをJSON artifactとmanifestへ保存する。
- interactive replayとheadless offline runnerで同じdetector、format名、adapter、topic overrideを使用する。片方だけを直して契約を分岐させない。
- `--config`で与えられたチューニング値は維持し、format選択によって変更するのは入力topic overrideとadapter有無だけとする。
- ソフトウェアが完走して点群・odometryを出したことと、軌跡品質、実機、Jetson、絶対精度は別判定とする。ground truthなしでATE/RPEや物理運用適合を主張しない。
- 実装branchはfresh contextで`main`から`feat/livox-dual-format-replay`を作成する。`codex/` prefixは使用しない。ユーザーから依頼がない限りpush、merge、既存branch削除は行わない。

## Final plan

1. Fresh implementation contextで状態を再確認する。
   - `/home/user/ws/fastlio-go2w`へ移動し、適用される`AGENTS.md`をすべて読み、本計画を全文読む。
   - `git status --short --branch`、`git rev-parse HEAD`、`git branch -a`を確認する。`main`と`origin/main`の関係、ユーザー所有の`results/`、実行中のFAST-LIO・rosbag・RViz processを確認し、外部変更があれば計画との差分を評価する。
   - cleanな`main`から`git switch -c feat/livox-dual-format-replay`を実行する。既存dirty fileはstash、restore、削除しない。

2. rosbag metadataのLivox format detectorを追加する。
   - `scripts/fastlio/detect_livox_bag_format.py`を追加し、PyYAMLで`rosbag2_bagfile_information.topics_with_message_count`を読む。
   - exact topic `/livox/lidar`のtypeを一意に抽出し、`livox_ros_driver2/msg/CustomMsg`を`custom-msg`、`sensor_msgs/msg/PointCloud2`を`pointcloud2`としてstdoutへ1 tokenだけ返す。
   - metadata欠落・parse error・topic欠落・重複型・unsupported typeは診断をstderrへ出してnonzero終了する。message countが0の場合も入力として拒否する。
   - detectorのpure functionとCLIを分け、`scripts/fastlio/test_detect_livox_bag_format.py`に旧形式、新形式、欠落、0件、重複、unsupported、malformed YAMLのunit testを追加する。

3. PointCloud2からLivox CustomMsgへの専用C++ adapter packageを追加する。
   - `humble_ws/src/fastlio_go2w_livox/`を新しい`ament_cmake` packageとして作成し、`rclcpp`、`sensor_msgs`、`livox_ros_driver2`、`diagnostic_msgs`を依存にする。
   - conversion libraryとnodeを分離し、libraryはROS graphなしでunit test可能にする。node parameterは`input_topic`、`output_topic`、`diagnostics_topic`、`max_point_header_delta_sec`、`minimum_points`、diagnostics publish periodを持つ。
   - field名からoffsetを解決し、fixed strideを仮定せず、little/big endian、row padding、organized cloudを安全に読む。整数overflowとpointer範囲を事前検証する。
   - headerをnanosecondsへ安全に変換し、frame header regression、invalid stamp、schema/layout error、nonfinite coordinate/intensity/timestamp、intensity範囲・整数性違反、negative/too-large offset、too-few-pointsをframe drop reasonとしてcounter化する。
   - valid pointをabsolute timestampでstable sortし、CustomMsgのheader、timebase、point_num、x/y/z、reflectivity、tag、line、offset_timeを生成する。output publisherはFAST-LIOのreliable depth-20 subscriberと互換なQoSにし、input subscriptionはbag publisherと互換なQoSを使う。
   - `test/test_pointcloud_adapter.cpp`でdriver由来26-byte sample、padding/organized/big-endian layout、値保持、timestamp差分、double nanosecond量子化境界、stable sort、各drop path、header regression、uint32境界を網羅する。元のdriver-style PointCloud2から期待CustomMsgを復元できることをfixtureで確認する。

4. launch graphにformat選択とtopic overrideを通す。
   - `fastlio_go2w_bringup/package.xml`に新adapter packageのruntime dependencyを追加する。
   - `fastlio.launch.py`へoptionalな`lid_topic_override` launch argumentを追加する。空ならconfig YAMLをそのまま尊重し、非空時だけ`common.lid_topic`を追加parameterでoverrideする。
   - `bringup.launch.py`から`lid_topic_override`をforwardする。
   - `replay.launch.py`へresolved `lidar_format` argumentを追加する。`custom-msg`ではadapterなし・既存topic、`pointcloud2`では`fastlio_go2w_livox` nodeを起動して`/livox/lidar_fastlio`をFAST-LIOへ渡す。未知値はlaunch開始前に例外で停止する。
   - `offline_fastlio.launch.py`にも同じargumentとnode構成を適用し、interactive/offlineで処理graphが一致するよう共通helperをbringup package内へ置くか、同じ最小ロジックをtestで同期確認する。
   - RViz topic `/cloud_registered`、odometry adapter、robot description、既存config tuningは変更しない。

5. `scripts/fastlio/replay.sh`を後方互換CLIへ更新する。
   - usageへ`--lidar-format auto|custom-msg|pointcloud2`を追加し、既定`auto`でdetectorを呼ぶ。
   - explicit formatでもmetadataを検査し、検出結果と矛盾すれば再生前に停止する。resolved formatを表示し、`replay.launch.py lidar_format:=...`へ渡す。
   - ROS環境preflightも堅牢化し、`/opt/ros/humble/setup.bash`が存在すれば現在の`ROS_DISTRO`がJazzy等でもHumbleを正しくsourceする。Humbleがない・overlayが現在sourceと一致しない場合は、devcontainerとcontainer内bag pathを示して停止する。
   - 既存`--rviz/--no-rviz`、`--rate`、`--config`の意味と既定値を維持する。
   - `bash -n`とCLI error testで未知option、format mismatch、host path/container path errorを確認する。

6. headless offline runnerへ同じformat契約とprovenanceを統合する。
   - `scripts/offline/run_fastlio_offline.sh`へ`--lidar-format`を追加し、同じdetectorでresolved formatを決める。player topicは引き続き`/livox/lidar`と`/livox/imu`だけに限定する。
   - launch commandへresolved formatを渡す。custom-msg時は従来のendpoint readinessを維持し、pointcloud2時は`/livox/lidar`のPointCloud2 publisher→adapter subscriberと`/livox/lidar_fastlio`のCustomMsg publisher→FAST-LIO subscriberをtype-awareに確認する。topic名のpublisher/subscriber数だけでなく`ros2 topic info -v`相当のtypeを検証する。
   - live parameter validationの`common.lid_topic`期待値をformat別に切り替える。adapter node readiness、diagnostics topic、process metrics targetもpointcloud2時だけ追加する。
   - pointcloud2時はadapter diagnosticsをresult bagまたは専用collectorで保存し、`livox_adapter_diagnostics.json`へ最終counterを抽出する。抽出scriptとunit testを追加し、received/converted/drop/reordered/scan widthをmanifestへ入れる。
   - manifestへmetadataで検出したtype、resolved format、format override、adapter enabled/topic/config、adapter source/runtime hash、diagnostics artifact/hashを追加する。custom-msg時はadapter disabledを明記し、既存artifact contractを壊さない。
   - runtime overlay preflightへ新package executable/hashをpointcloud2時だけ追加し、stale buildを拒否する。cleanup/drain/error handlingでadapter processがlaunch groupと共に確実に終了することをtestする。

7. documentationとoperator guidanceを更新する。
   - `README.md`のreplay節にauto detection、対応する2型、container内path、resolved format表示、explicit override例を追加する。
   - `docs/offline-result-artifacts.md`へformat provenance、adapter diagnostics artifact、旧形式ではadapterなしであることを追加する。
   - 「新PointCloud2を`lidar_type=4`へ直接渡さない理由」「ソフトウェア出力成立と軌跡品質は別」「XT16は本変更の対象外」を明記する。

8. static/unit/build testを実行する。
   - hostで`bash -n scripts/fastlio/replay.sh scripts/offline/run_fastlio_offline.sh`を実行する。
   - ROS非依存Python testを実行する。例: `python3 -m pytest -q scripts/fastlio/test_detect_livox_bag_format.py`および追加diagnostics extractor test。
   - Humble devcontainer内でworkspaceをrebuildし、新package、bringup、FAST-LIOを対象に`colcon test`と`colcon test-result --verbose`を実行する。source/runtime launch hashとadapter executableがpreflightを通ることを確認する。
   - 既存bringup Python testとoffline script testを全件実行し、旧CLI/config/result artifact contractの回帰がないことを確認する。

9. 旧・新実bagで段階的runtime validationを行う。
   - devcontainer内の旧bag`/mnt/go2w-experiment-recorder/bags/experiment_long3_20260714_014823`に対してdetectorが`custom-msg`を返すこと、adapter nodeが存在しないこと、`/livox/lidar`が単一型でFAST-LIOへ接続することを確認する。
   - 新bag`/mnt/go2w-experiment-recorder/bags/experiment_save-test_20260727_022320`に対してdetectorが`pointcloud2`を返すこと、入力とcanonical outputが別topic・別型で接続すること、adapter diagnosticsが受信・変換を増加させcritical drop 0であることを確認する。
   - それぞれ30秒のisolated-domain headless smokeを新しい空output directoryで実行し、`/cloud_registered`、`/Odometry`、`/odom`が非zero、finiteであること、processがcleanup後に残らないこと、manifestが正しいformat/provenanceを持つことを確認する。
   - 新bagは必要に応じてより長い区間を実行し、frame coverage、timestamp regression、trajectory jump、nonfinite poseを解析する。artifact生成やexit codeだけで軌跡品質合格としない。
   - 最後にinteractive `replay.sh`を旧・新bagで各1回確認し、RVizの`/cloud_registered`表示、Fixed Frame、TF errorの有無を確認する。GUI目視結果は自動testと分けて記録する。

10. 変更範囲と結果をhandoffする。
   - `git diff --check`、`git status --short`、branch diffを確認し、ユーザーの`results/`やbagを含めていないことを確認する。
   - 実装したformat matrix、test結果、旧・新bagの各runtime evidence、未検証境界を簡潔に報告する。
   - ユーザーが明示しない限りcommit、push、PR、mergeは行わない。

## Validation

実装時に最低限実行するcommandは次の通り。実際のpackage名・test pathに合わせて必要最小限調整し、調整理由を記録する。

```bash
cd /home/user/ws/fastlio-go2w
git status --short --branch
bash -n scripts/fastlio/replay.sh
bash -n scripts/offline/run_fastlio_offline.sh
python3 -m pytest -q scripts/fastlio/test_detect_livox_bag_format.py
```

Humble devcontainer内:

```bash
cd /workspaces/fastlio-go2w
bash .devcontainer/postCreate.sh
source /opt/ros/humble/setup.bash
source .devcontainer/desktop_ws/install/setup.bash
colcon test --base-paths humble_ws/src --packages-select fastlio_go2w_livox fastlio_go2w_bringup
colcon test-result --verbose
```

Format detection:

```bash
python3 scripts/fastlio/detect_livox_bag_format.py \
  /mnt/go2w-experiment-recorder/bags/experiment_long3_20260714_014823/metadata.yaml
python3 scripts/fastlio/detect_livox_bag_format.py \
  /mnt/go2w-experiment-recorder/bags/experiment_save-test_20260727_022320/metadata.yaml
```

期待値は順に`custom-msg`、`pointcloud2`。explicit override mismatchはnonzeroで停止すること。

Headless smokeは既存outputを上書きしない新規directoryで行う。

```bash
OLD_OUT="$(mktemp -d /tmp/fastlio-old-custom-XXXXXX)/run"
NEW_OUT="$(mktemp -d /tmp/fastlio-new-pointcloud2-XXXXXX)/run"

bash scripts/offline/run_fastlio_offline.sh \
  /mnt/go2w-experiment-recorder/bags/experiment_long3_20260714_014823 \
  --duration 30 --output "$OLD_OUT" --no-analyze

bash scripts/offline/run_fastlio_offline.sh \
  /mnt/go2w-experiment-recorder/bags/experiment_save-test_20260727_022320 \
  --duration 30 --output "$NEW_OUT" --no-analyze
```

各runでmanifest stateがcompleted、result bagに`/cloud_registered`、`/Odometry`、`/odom`が非zeroで入り、旧runはadapter disabled、新runはadapter enabledかつreceived/converted>0、critical drop=0であることを確認する。smokeは実行成立確認であり、trajectory quality acceptanceではない。

## Acceptance criteria

- 引数なしの`replay.sh BAG`がmetadataから旧CustomMsgと新PointCloud2を正しく自動判定する。
- 旧CustomMsg bagはadapterなしで現在と同じFAST-LIO Livox callbackへ入り、`/cloud_registered`とodometryを出す。
- 新PointCloud2 bagは同名異型endpointを作らず、専用adapterを経てCustomMsgとしてFAST-LIOへ入り、`/cloud_registered`とodometryを出す。
- `--lidar-format` explicit指定、auto検出、矛盾拒否、不明型拒否がtestされている。
- adapterは26-byte固定layoutに依存せず、field/schema/layout/time/value anomalyを診断付きでfail closedに扱う。
- 新bag smokeでadapter critical dropが0、旧bag smokeでadapterが起動しない。
- offline manifestとartifactから、入力ROS型、resolved format、adapter有無・hash・counter、実行config、bag metadata hashを再現可能に追跡できる。
- 既存`--config`、rate、RViz、old offline artifact、MID-360 tuningに回帰がない。
- ユーザー所有の`results/`、元bag、`feat/xt16-go2w-imu`を変更しない。
- software outputの成立とtrajectory quality、Jetson、実機、絶対精度の境界が報告と文書に保持される。

## Risks / cautions

- PointCloud2のabsolute nanosecondsはFLOAT64であり、epoch近傍では整数nanosecondを完全表現できない。adapterは`llround`後にheaderとの差を取り、量子化誤差と許容範囲をtestするが、失われた下位bitは復元できない。
- intensityからuint8 reflectivityへの変換はcurrent Livox driver出力なら可逆だが、別producerのfractional/out-of-range intensityを黙って丸めない。対応contract外としてframe dropし、overrideで弱めない。
- topic publisher/subscriber数だけでは異型接続を見逃すため、readinessはtype-awareである必要がある。
- launch parameter overrideの順序を誤るとユーザー指定configを意図せず上書きする。`lid_topic_override`以外のtuningを変更しないtestが必要である。
- adapter追加でserialization/copy負荷が増える。offline desktopでは許容見込みだが、CPU/メモリをresource metricsで測定する。性能不足が実測された場合のみ、FAST-LIO native PointCloud2 handlerへの移行を別設計として評価する。
- 旧bagと新bagは同時刻・同一走行ではないため、両者のtrajectoryを直接比較してadapter精度を断定できない。unit-level round-tripと各bagの個別sanity checkを分ける。
- 旧`long3`や過去XT16 validationには既知の軌跡品質問題があり得る。30秒outputが出てもfull-run qualityを合格としない。
- devcontainer mount pathとhost pathが異なる。validation commandをhost Jazzyで誤実行しない。
- 本計画はMID-360 format compatibilityに限定する。XT16 branch統合や`--sensor xt16`のmain導入は、branch境界と既存validationを再評価する別作業とする。
