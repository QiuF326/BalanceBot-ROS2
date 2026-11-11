# 双手机实时语音与触控控制 ROS2

版本：`2026.10.30-final.2`。适用于当前 Ubuntu 22.04 / Python 3.10 / ROS2 Humble 部署。

手机麦克风通过 HTTPS/WSS 连续发送 PCM，Python 中两个独立 Vosk 识别器直接控制 ROS2。语音、触控、单机和双机使用同一份权限状态，雷达保护在最终输出前统一检查。本目录是独立 Python 程序，不需要重新 colcon 编译。

## 当前虚拟机启动

部署目录 `/home/yahboom/Downloads/realtime_voice_control`，`ROS_DOMAIN_ID=20`，Agent UDP 端口 `8899`。

双击 Ubuntu 桌面的“平衡车控制台”，会启动服务并弹出独立二维码窗口，不再依赖 Firefox 是否能启动。也可以在终端运行：

```bash
cd /home/yahboom/Downloads/realtime_voice_control
bash start_control.sh --open
```

手机和电脑连接同一热点，扫码后选择角色。启动器会显示当前控制地址；二维码和首次证书安装页使用启动器检测出的局域网 IP，Ubuntu 本机也可打开 `http://127.0.0.1:8081/` 查看二维码。

启动程序自动读取默认联网网卡的 IPv4，使用已有 CA 给当前 IP 续签服务端证书。已经信任该 CA 的手机无需重新安装证书。固定 `.local` 名称在当前手机热点上未通过验证，不作为日常入口。热点改变后，小车固件中 Agent 目标 IP 仍需对应更新，二维码不能修改固件配置。

启动器通过文件锁和端口检查避免重复服务。首次启动保持人工急停；重复点击已经运行的服务不会擅自复位。窗口关闭后服务继续运行。服务日志和进程记录分别位于 `logs/`、`run/`；更新前备份位于 `setup-cache/before-final-*`。

不要再单独启动另一个 Agent、旧自动避障节点、`cmdvel2bl` 或旧双端服务器，它们可能占用端口或竞争 `/cmd_vel_bl`。

## 控制角色、口令与优先级

最多两个终端。点选角色可完成连接和角色分配。

| 角色 | 权限 | 可配合角色 |
| --- | --- | --- |
| FULL 单机全控 | 前后及转向 | 独占运动权限 |
| LINEAR 前后 | 前进、后退 | ANGULAR |
| ANGULAR 转向 | 左转、右转 | LINEAR |
| LEFT / RIGHT 原协同 | 各改变一个输入，沿用旧公式 | 对侧角色 |

正常双机建议 LINEAR + ANGULAR，两端指令互不覆盖。支持前进、后退、左转、右转、加速、减速、停止、急停及固定同义短语。分贝大小不改变速度。

- 人工急停和已启用的 IMU 保护优先，不能被脱困绕过。IMU 保护默认关闭；显式 `--imu-safety` 可启用。
- 普通操作始终经过雷达检查。触控更新后，本端语音运动口令暂停接管 600ms，另一端仍独立工作；停止、急停口令始终优先。
- 普通方向键在按下时立即发送，默认前后 ±7、转向 ±150；点击后持续，中央停止。普通与人工脱困共用同一套方向键、滑杆及语音入口；点击后持续运行，停止键归零。
- “停止”在普通双端模式下归零本端；“急停”归零双方并锁定。人工脱困也遵循本端停止、整车急停的双端权限规则。
- 断线、切换角色、保护停车、服务重启会使旧请求失效。恢复连接不会重放运动、复位急停或自动开启麦克风。

“加速/减速”每次调整归一化输入 0.10，默认范围 0.15–0.65，仅调整本端正在运动的轴，静止时不会起步。语音默认前后 0.35、转向 0.50；停止保留档位，人工复位或重新分配角色恢复初始档位。

底盘沿用自定义接口 `linear.x = linear × 20`、`angular.z = angular × 300`，不是 SI 速度。默认语音输出为前后 ±7、转向 ±150；不要将其理解成 m/s 或 rad/s。FULL 的方向口令归零另一轴，两轴滑杆可以组合输入。

旧 LEFT/RIGHT 公式为 `(LEFT + RIGHT)/2 × 20` 和 `(RIGHT - LEFT)/2 × 300`。仅剩一侧在线时只支持前后；整车停车使用急停。

## 雷达监测与人工脱困

正常保护默认开启，不需要另发避障口令。阈值以雷达原点距离为准：

| 行为 | 前后 | 左右 |
| --- | --- | --- |
| 手机预警 | 45cm | 22.5cm |
| 限速 | 30cm | 15cm |
| 停车并清除旧指令 | 25cm | 10cm |

纯前进只检查前方，纯后退只检查后方；转弯会检查四个方向，因为车身转动会扫过周围。3cm 释放回差减少边界抖动。限速时两轴按相同比例限制归一化幅度至最多 0.15，障碍消失不会自动加速；停车后也不会自行起步。

采用最新帧、深度 1、BEST_EFFORT 雷达订阅。超过 0.5 秒无扫描、扫描损坏或所需方向无有效测距时，普通运动停止。无回波不视为道路畅通。手机以约 4Hz 刷新四向距离、限制原因和报警事件；保留最近有效测距，并给出每个方向的数据年龄。过期读数标为旧读数，仅供观察，不参与放行，可启用提示音，支持的安卓浏览器可振动；请保持页面在前台。

驾驶卡片只保留一个“人工脱困”开关，正常与脱困共用同一套方向键、滑杆和语音入口。

1. 开启后，雷达仍订阅、计算四向距离并更新页面，但不限制任何一端的语音或触控运动。未收到扫描、扫描过期、坏帧、近障和限速区都不会通过雷达联锁阻止运动。
2. 输出沿用正常输入幅度：普通方向键/默认语音前后 ±7、转向 ±150，双端可以组合前后和转向。没有固定3秒上限，不要求另按一套按钮或按住续期。
3. 模式由服务器统一保存，两台手机显示同一个开关状态。切换时先清除双方旧输入，随后重新发令；关闭后立即恢复正常雷达联锁。
4. 发起端离线、心跳超时或释放/更换角色会清除双方输入并退出人工脱困。页面隐藏时仍停止本端运动；急停、底盘通信和已启用的 IMU 保护保持有效。

人工脱困是有意旁路雷达联锁，操作者须目视控制路径。雷达没接上时能够通过语音和按键发布命令，但 Agent 没运行、网络断开或底盘没有连上 Agent 时，任何软件模式都不能让底盘收到命令。开关不会自动复位急停。

此前的“按住低速脱困”接口仅保留为旧客户端兼容，不出现在新版页面中，也不作为人工脱困开关的实现。

默认 0° 为车头方向，可用 `--scan-front-degrees` 校正；安装位置及车身尺寸需要实测。尚未测量实车制动距离，25cm/10cm 阈值不等于保证车身净距。

## 已处理的故障与诊断

`failed to create array for field 'intensities'`：改为原始 CDR 雷达订阅，解析前限制帧长、数组长度和编码，不再创建用不到的 intensities 数组。结构错误帧会拒绝、计数，并使普通运动停车；不会将损坏数据当作安全。此处理不修复雷达固件或无线丢帧。

Agent 的 `Cdr::deserialize(bool)` 异常发生在 Agent 进程内，Python 优先级无法修复该异常。本版优先使用虚拟机已缓存的官方 `microros/micro-ros-agent:humble` 镜像，按本地镜像 ID 启动，不在控制时下载镜像、不使用 privileged。没有缓存时使用现有 `~/uros_ws` Agent。当前镜像实际使用 XRCE Agent 2.4.1；之前本机工作区使用 2.4.2。尚未证实具体异常的唯一根因。

新增 Agent 进程守护：异常退出后间隔 2 秒、4 秒重启；60 秒内累计 3 次异常则停止重试并在 `run/agent-status.json` 标为 FAILED。控制器检查本机 UDP8899 监听，发现丢失会锁定急停、清空旧输入；恢复后必须人工复位。端口存在只能证明本机 Agent 在监听，不能证明小车已连接或电机执行成功。

诊断入口：

```bash
cd /home/yahboom/Downloads/realtime_voice_control
cat run/agent-status.json
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=20
ros2 topic info /cmd_vel_bl --verbose
ros2 topic info /scan --verbose
```

手机“运行详情”包含版本、实际发布频率、Agent 监听状态/中断次数、拒绝雷达帧数量及最近错误。没有新雷达或里程计回报时，先检查小车供电、雷达旋转、热点、固件目标 IP 与 Agent 日志。不要通过延长扫描超时来掩盖失联。

当前接口用于可信局域网，没有账户认证；主机掉电或进程被强杀时无法继续发送零值，底盘仍须具备命令超时停车能力。本版没有刷写底盘固件。

## 语音实时处理

AudioWorklet 连续重采样到 16kHz 单声道 PCM16，每帧 20ms。两端独立识别线程，不在 ROS 发布或 HTTP 线程等待识别。稳定中间结果约 80ms 可触发运动；停止优先，无需等待句末。

每路最多 12 帧队列，音频超过 350ms 或积压时丢弃旧帧、清除本端语音输入并恢复实时收音。音频中断 0.5 秒停止本端语音输入，心跳中断 1 秒释放角色。保护与输入检查以 50Hz 运行；新接受的指令（含重复指令）立即发布，稳态约 10Hz 保持，减少嵌入式链路上的重复可靠消息。触控只保留最新待发送输入，不堆积滑杆请求。

以上是处理参数，不是完整语音响应时延。实际响应还受说话、识别、热点和底盘影响。双手机可能同时听见同一人的声音；固定短口令不能预知稍后说出的否定后半句。

## 新环境与开发

压缩包只包含源码、测试及说明，不包含模型、虚拟环境或任何证书私钥。现有虚拟机无需重新安装这些依赖。

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=20
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements.txt
# 下载并解压 vosk-model-small-cn-0.22 到本目录
# 首次创建并配置手机信任的 CA 后，准备 certs/phone-cert.pem 和 phone-key.pem
.venv/bin/python dual_control_voice_server.py \
  --model vosk-model-small-cn-0.22 \
  --cert certs/phone-cert.pem --key certs/phone-key.pem --start-estopped
```

桌面二维码窗口使用系统 Python 的 GTK3/GdkPixbuf（Ubuntu 的 python3-gi、gir1.2-gtk-3.0）。自动证书更新依赖 mkcert/openssl 及原 `certs/ca/rootCA.pem`、`rootCA-key.pem`，公开 CA 位于 `certs/public/`。保留原 CA，不要把私钥提供给手机或打包分发。

麦克风要求受信任 HTTPS：[浏览器说明](https://developer.mozilla.org/en-US/docs/Web/API/MediaDevices/getUserMedia)、[iPhone CA 完全信任](https://support.apple.com/en-us/102390)、[mkcert 手机说明](https://github.com/FiloSottile/mkcert#mobile-devices)。已有 CA 信任可保留。

离线界面开发可运行 `python dual_control_voice_server.py --dry-run --no-voice --no-collision-guard`。`--no-collision-guard` 仅用于离线联调，页面会显示保护关闭。`--imu-safety`、`--confirmed-only`、`--scan-topic` 等参数可通过 `--help` 查看。带自定义参数的手动实例不要与桌面启动器重复运行。

现有 `src/yahboomcar_laser`、激光里程计、SLAM、导航模块保持独立。未来接导航时应统一仲裁底盘输出，保留一个 `/cmd_vel_bl` 发布者。

## 验证

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=20
.venv/bin/python -m unittest test_voice_control test_collision_guard test_final_control test_access
node test_audio_worklet.cjs
node test_collision_alerts.cjs
node test_control_session.cjs
```

本版在真实 Ubuntu/Humble 上通过 130 项 Python 测试，包括随机私有话题的实际 ROS2 发布/订阅、真实 LaserScan 序列化、畸形 CDR、雷达超时、双端权限、HTTP/WSS、触控优先、人工模式无雷达双端触控与语音/坏帧及过期数据旁路/退出恢复联锁/角色与急停、Agent 有限重试、IP/证书与公钥文件访问限制。测试不向真实底盘话题发布非零运动。

调整发送节奏后，实车约 12 秒的 48 次状态采样均为新鲜扫描，最大数据年龄约 192ms，扫描频率约 8.4Hz；调整前一次 12 秒被动订阅仅收到 5 帧、最长间隔约 7.7 秒。该结果证明本轮回传改善，不能保证无线链路始终无间歇。

三个 Node 测试文件验证 16/44.1/48kHz 连续音频重采样、页面脚本、两端报警、阈值、角色恢复及实际页面中的统一脱困开关、普通/脱困共用方向键和保留语音入口。用户已确认 Ubuntu 桌面能弹出新版二维码窗口，精简页能显示距离和更新时间，并反馈整体操控体感改善。双路真实 Vosk 静音各 12 秒、600 帧均持续连接，无恢复或丢帧，峰值单帧解码约 24ms/19ms；静音测试不代表口令准确率。

软件测试不能替代实车制动、雷达盲区和热点丢帧验证。请先保持急停核对四向距离；再在空旷处低速确认方向、停止与报警，最后目视验证人工脱困开关与退出恢复防撞。语音识别准确率与完整端到端时延尚未统计。

## 参考

- [Vosk API](https://github.com/alphacep/vosk-api)
- [ROS2 Humble rclpy 订阅接口](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/node.py)
- [micro-ROS Agent](https://github.com/micro-ROS/micro-ROS-Agent)
- [Micro-XRCE-DDS bool 反序列化问题讨论](https://github.com/eProsima/Micro-XRCE-DDS/issues/173)：相似错误的参考，不能据此确定本车根因。
