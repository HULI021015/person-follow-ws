# ROS 2 移动抓取机器人（Ubuntu 22.04 / Humble）

项目位于 WSL 的 `/home/zheng/person_follow_ws`，已将人员跟随小车与 Piper 机械臂抓取工程整合到同一 ROS 2 Humble 工作区。

- 小车：大华 NetSDK + PlaySDK 低延迟取流、YOLOv8 + BoT-SORT 人员跟踪、YK CAN115 底盘控制。
- 机械臂：Orbbec 腕部 RGB-D、TensorRT YOLOv8 实例分割、Piper IK 与当前 SAM3-style 抓取流程。
- 机械臂只接受 `bottle`；旧的第二种抓取方式及专用变量已移除。
- “数智巡检平台”网页：一键启停、双路识别画面、底盘速度/航向、由实时关节反馈驱动的 Piper URDF/STL 三维模型，以及抓取任务状态。

详细安装、启动、网页使用和分层调试方法见 [整合项目启动调试与网页使用指南.md](docs/整合项目启动调试与网页使用指南.md)。原有底盘专项调试仍可参考 [小车运动控制调试指南.md](docs/小车运动控制调试指南.md)。

快速启动网页控制台：

```bash
cd ~/person_follow_ws
./scripts/start_dashboard.sh
```

启动脚本检测到 `can0` 时会自动配置 Piper 的 1 Mbps SocketCAN；首次需要时会在当前终端请求 `sudo`。浏览器打开 `http://127.0.0.1:8080`。首次实车运行前必须确认急停有效、机械臂工作区无人，并按文档完成分层测试。
