# BalanceBot-ROS2

双轮平衡车的 ROS 2 控制项目。当前提交的可运行模块位于 [`tools/realtime_voice_control`](tools/realtime_voice_control)，对应雷达监测与人工脱困版本 `2026.10.30-final.2`。

## 当前功能

- 两部手机通过网页进行独立语音识别与触控操控，可分配前后、转向角色，也支持单机全控。
- Python 控制程序统一仲裁运动请求，并向 ROS 2 底盘话题 `/cmd_vel_bl` 发布指令。
- 雷达四向距离实时显示；普通模式在近障、数据过期或无有效测距时限制相应运动。
- 人工脱困开关暂时旁路雷达联锁，沿用原方向键、滑杆和语音入口；人工急停及通信保护仍生效。
- 初次启动保持急停，角色断开、切换或保护停车时清除旧运动请求。

## 运行

目标环境为 Ubuntu 22.04、ROS 2 Humble、Python 3.10。已配置语音模型、HTTPS 证书和 micro-ROS Agent 的主机上运行：

```bash
cd tools/realtime_voice_control
bash start_control.sh --open
```

启动器会检测当前局域网 IPv4，输出手机控制页和二维码入口。首次部署、证书配置、雷达阈值、操作规则及诊断方法见[控制程序说明](tools/realtime_voice_control/README.md)。源码包不含语音模型、证书及私钥。

## 验证

```bash
cd tools/realtime_voice_control
python3 -m unittest test_voice_control test_collision_guard test_final_control test_access
node test_audio_worklet.cjs
node test_collision_alerts.cjs
node test_control_session.cjs
```

控制器可独立运行；不需要重新编译整个 ROS 2 工作区。底盘固件、雷达驱动及其他 ROS 2 包未包含在本次提交中。
