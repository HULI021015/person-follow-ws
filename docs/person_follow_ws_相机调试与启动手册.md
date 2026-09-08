# person_follow_ws 相机识别调试与日常启动手册

> 适用项目：`person_follow_ws`  
> 当前相机链路：**大华摄像头 → Windows 转发 → WSL → NetSDK / PlaySDK → YOLOv8 → BoT-SORT → /cmd_vel**  
> 当前状态：已验证相机取流、人员识别、目标跟踪和 `/cmd_vel` 输出稳定。

---

## 1. 当前整体架构

当前项目不再使用旧的 RTSP + OpenCV 拉流方式，而是：

```text
大华摄像头
192.168.1.21:37777
        │
        ▼
Windows
端口转发 37778
        │
        ▼
WSL Ubuntu 22.04
        │
        ▼
大华 NetSDK
        │
        ▼
PlaySDK 解码
        │
        ▼
只保留最新帧
        │
        ▼
YOLOv8 人员检测
        │
        ▼
BoT-SORT 人员跟踪 / ID
        │
        ▼
计算 linear.x / angular.z
        │
        ▼
/cmd_vel
```

### 为什么要使用这一套

之前的 RTSP 方案虽然 `/cmd_vel` 发布频率可以达到约 10 Hz，但视频缓存可能不断累积，导致：

```text
现实画面
↓
十几秒甚至几十秒
↓
程序才处理到这一帧
```

这样即使控制指令发布得很快，实际上也是在控制“过去的人”。

当前新版使用 NetSDK + PlaySDK，并尽量只处理最新帧，核心目标是：

```text
宁可丢旧帧，也不要积压旧帧。
```

因此以后判断系统是否实时，不能只看 `/cmd_vel` 的 Hz，更重要的是看：

```text
frame_age
```

---

# 2. 首次环境检查

进入 WSL Ubuntu：

```bash
cd ~/person_follow_ws
```

加载 ROS：

```bash
source /opt/ros/humble/setup.bash
```

加载工作空间：

```bash
source install/setup.bash
```

加载大华 SDK：

```bash
source scripts/netsdk_env.sh
```

---

## 2.1 检查 NetSDK / PlaySDK

```bash
python3 -c "from NetSDK.NetSDK import NetClient; sdk=NetClient(); print('NetSDK:', hasattr(sdk,'sdk')); print('PlaySDK:', hasattr(sdk,'play_sdk'))"
```

正常应看到：

```text
NetSDK: True
PlaySDK: True
```

---

## 2.2 检查 CUDA

```bash
python3 -c "import torch; print('torch:', torch.__version__); print('CUDA:', torch.cuda.is_available()); print('GPU:', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'None')"
```

正常建议：

```text
CUDA: True
GPU: ...
```

如果 CUDA 为 `False`，YOLO 仍可能运行，但推理速度通常会明显下降。

---

## 2.3 检查 Ultralytics

```bash
python3 -c "import ultralytics; print(ultralytics.__version__)"
```

能够正常输出版本号即可。

---

# 3. Windows 摄像头端口转发

## 3.1 为什么需要 Windows 转发

当前摄像头真实地址：

```text
192.168.1.21
```

大华 NetSDK 默认端口：

```text
37777
```

Windows 可以直接访问摄像头，但 WSL NAT 环境不一定能够直接访问 `192.168.1.21:37777`。

因此采用：

```text
WSL
  ↓
Windows 在 WSL 中的 IP:37778
  ↓
Windows
  ↓
192.168.1.21:37777
```

---

## 3.2 Windows PowerShell 启动转发

项目位于 WSL：

```text
/home/zheng/person_follow_ws
```

Windows PowerShell 可以直接进入：

```powershell
cd \\wsl.localhost\Ubuntu-22.04\home\zheng\person_follow_ws
```

如果 PowerShell 提示符已经类似：

```text
PS Microsoft.PowerShell.Core\FileSystem::\\wsl.localhost\Ubuntu-22.04\home\zheng\person_follow_ws>
```

说明已经在正确目录，不需要再次 `cd`。

启动转发：

```powershell
PowerShell -ExecutionPolicy Bypass -File scripts\start_camera_relay_windows.ps1
```

正常会出现类似：

```text
Camera relay starting: 172.xx.xx.xx:37778 -> 192.168.1.21:37777
Keep this window open. Press Ctrl+C to stop.
```

### 注意

这个 PowerShell 窗口必须保持运行。

它不是“卡死”，而是在持续负责摄像头端口转发。

停止转发时按：

```text
Ctrl + C
```

---

# 4. WSL 中验证摄像头转发

新开一个 Ubuntu / WSL 终端。

查看 Windows 在 WSL 中的地址：

```bash
grep nameserver /etc/resolv.conf
```

或者：

```bash
WIN_IP=$(awk '/nameserver/{print $2; exit}' /etc/resolv.conf)
echo $WIN_IP
```

例如：

```text
172.26.16.1
```

然后测试：

```bash
timeout 2 bash -c "</dev/tcp/$WIN_IP/37778" && echo CAMERA_RELAY_OK
```

如果出现：

```text
CAMERA_RELAY_OK
```

说明：

```text
WSL → Windows → 摄像头
```

网络链路已经完全打通。

---

# 5. 配置当前终端的摄像头地址

每次新开 WSL 终端，都建议重新执行：

```bash
WIN_IP=$(awk '/nameserver/{print $2; exit}' /etc/resolv.conf)

export DAHUA_IP=$WIN_IP
export DAHUA_PORT=37778
```

检查：

```bash
echo $DAHUA_IP
echo $DAHUA_PORT
```

应类似：

```text
172.26.16.1
37778
```

---

# 6. 单独测试摄像头取流

在第一次部署、网络环境变化、出现明显延迟时，建议先运行摄像头延迟测试。

```bash
cd ~/person_follow_ws

source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/netsdk_env.sh

WIN_IP=$(awk '/nameserver/{print $2; exit}' /etc/resolv.conf)
export DAHUA_IP=$WIN_IP
export DAHUA_PORT=37778
```

运行：

```bash
ros2 run person_follower camera_latency_test \
  --duration 10 \
  --stream extra \
  --snapshot /tmp/dahua_latest.jpg
```

正常会看到类似：

```text
RESULT frames=...
observed_fps=...
source_fps=...
age_mean_ms=...
age_p95_ms=...
age_max_ms=...
playsdk_input_drops=...
SNAPSHOT /tmp/dahua_latest.jpg
```

---

## 6.1 重点看哪些参数

### observed_fps

程序实际收到并处理摄像头帧的速度。

例如：

```text
observed_fps=25
```

表示约 25 FPS。

---

### age_mean_ms

平均帧龄。

表示：

```text
这一帧产生
↓
到程序真正拿到它
↓
已经过去多少毫秒
```

这个参数非常重要。

理想情况：

```text
几十 ms
```

例如：

```text
age_mean_ms=40
```

一般可以认为实时性很好。

---

### age_p95_ms

95% 的帧延迟低于该值。

相比单纯平均值，更适合判断偶发延迟。

---

### playsdk_input_drops

PlaySDK 输入丢帧数量。

少量丢帧不一定是问题。

对于实时控制而言：

```text
丢掉旧帧
```

通常比：

```text
把旧帧全部排队处理
```

更安全。

---

# 7. 启动 YOLO 人员识别

确认摄像头链路正常后：

```bash
cd ~/person_follow_ws

source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/netsdk_env.sh

WIN_IP=$(awk '/nameserver/{print $2; exit}' /etc/resolv.conf)
export DAHUA_IP=$WIN_IP
export DAHUA_PORT=37778
```

启动：

```bash
ros2 launch person_follower follow.launch.py \
  start_chassis:=false \
  show_window:=true
```

---

## 7.1 为什么先使用 start_chassis:=false

```text
start_chassis:=false
```

表示：

```text
启动摄像头
启动 YOLO
启动跟踪
计算 /cmd_vel
但不启动真实底盘
```

这是调试视觉系统时最安全的模式。

---

# 8. 正常运行时应该看到什么

终端通常会出现类似：

```text
大华相机登录成功
```

然后：

```text
YOLO 预热完成
```

然后：

```text
人员跟随节点就绪：大华 NetSDK + PlaySDK + YOLOv8 + BoT-SORT
```

之后持续输出类似：

```text
处理 15 FPS
推理 40 ms
当前帧龄 50 ms
PlaySDK 输入丢弃 0
```

---

# 9. 识别窗口怎么看

窗口中人物会出现检测框。

例如：

```text
ID 1
```

BoT-SORT 会为每个人分配跟踪 ID。

当前被系统选中的目标可能显示：

```text
target=1
```

并显示：

```text
v=...
w=...
```

其中：

```text
v = linear.x
w = angular.z
```

---

# 10. 判断人员跟踪是否稳定

测试时建议：

1. 画面中先只有一个人。
2. 左右移动。
3. 前后移动。
4. 短暂遮挡。
5. 再重新出现。

重点观察：

```text
ID 是否稳定
```

理想状态：

```text
ID 1
ID 1
ID 1
ID 1
```

而不是频繁：

```text
ID 1
ID 2
ID 4
ID 7
```

同时：

```text
target
```

应尽量保持锁定同一个人。

---

# 11. 检查控制输出 /cmd_vel

保持：

```text
start_chassis:=false
```

再开一个 WSL 终端：

```bash
source /opt/ros/humble/setup.bash
source ~/person_follow_ws/install/setup.bash
```

查看：

```bash
ros2 topic echo /cmd_vel
```

输出类似：

```text
linear:
  x: 0.20

angular:
  z: -0.15
```

这时这些指令不会真正控制底盘，只是用于验证视觉控制逻辑。

---

# 12. /cmd_vel 五项测试

## 12.1 人在远处正中间

应该：

```text
linear.x > 0
angular.z ≈ 0
```

表示：

```text
向前追人
```

---

## 12.2 人在画面左边

正常 ROS 坐标系下：

```text
angular.z > 0
```

表示：

```text
向左转
```

---

## 12.3 人在画面右边

应：

```text
angular.z < 0
```

表示：

```text
向右转
```

---

## 12.4 人逐渐靠近

`linear.x` 应逐渐减小：

```text
0.30
↓
0.20
↓
0.10
↓
0.00
```

当前项目通过：

```text
人体检测框宽度 / 整幅画面宽度
```

估计人与摄像头之间的相对距离。

不是使用深度摄像头直接测量实际米数。

---

## 12.5 人完全离开画面

必须最终：

```text
linear.x = 0
angular.z = 0
```

即：

```text
丢失目标 → 停车
```

这是后续连接真实底盘前非常重要的安全条件。

---

# 13. 检查 /cmd_vel 发布频率

```bash
ros2 topic hz /cmd_vel
```

会看到类似：

```text
average rate: 11.xx
```

注意：

```text
/cmd_vel Hz 高
```

并不等于：

```text
控制实时
```

例如：

```text
11 Hz 发布十几秒前的画面结果
```

仍然完全不可用于人员跟随。

因此以后判断视觉控制性能，应优先看：

```text
1. frame_age
2. 推理时间
3. 图像实际体感延迟
4. /cmd_vel Hz
```

而不是只看第四项。

---

# 14. frame_age 是当前最重要指标之一

窗口或终端可能显示：

```text
frame_age=50ms
```

说明当前处理的是大约 50 ms 前的画面。

这是正常状态。

如果出现：

```text
frame_age=3000ms
```

说明已经延迟约 3 秒。

如果：

```text
frame_age=15000ms
```

说明已经延迟 15 秒。

这种情况下即使 YOLO 检测正常，也绝对不能直接控制底盘。

---

# 15. 当前项目的延迟安全保护

当前配置存在类似：

```text
max_control_latency = 0.50
```

即：

```text
500 ms
```

如果当前帧太旧，控制逻辑应停止继续使用这一帧控制车辆。

可能会看到：

```text
本帧处理延迟 xxx ms 超限，已停车
```

这个机制非常重要，应保留。

---

# 16. 日常启动 —— 最简流程

以后如果系统、代码和网络都没有变化，不需要每次重新做完整调试。

只需要两个窗口。

---

## 窗口 1：Windows PowerShell

进入项目：

```powershell
cd \\wsl.localhost\Ubuntu-22.04\home\zheng\person_follow_ws
```

启动摄像头转发：

```powershell
PowerShell -ExecutionPolicy Bypass -File scripts\start_camera_relay_windows.ps1
```

看到类似：

```text
37778 -> 192.168.1.21:37777
```

后保持窗口开启。

---

## 窗口 2：WSL Ubuntu

```bash
cd ~/person_follow_ws
```

加载环境：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/netsdk_env.sh
```

获取 Windows IP：

```bash
WIN_IP=$(awk '/nameserver/{print $2; exit}' /etc/resolv.conf)
```

设置摄像头：

```bash
export DAHUA_IP=$WIN_IP
export DAHUA_PORT=37778
```

启动人员识别：

```bash
ros2 launch person_follower follow.launch.py \
  start_chassis:=false \
  show_window:=true
```

即可进入：

```text
摄像头 + YOLO + BoT-SORT + /cmd_vel
```

模式。

---

# 17. 每次启动前建议做的 10 秒检查

### 1. 摄像头转发窗口还在吗？

Windows PowerShell 中：

```text
Camera relay starting ...
```

必须保持运行。

### 2. 网络通吗？

```bash
timeout 2 bash -c "</dev/tcp/$WIN_IP/37778" && echo CAMERA_RELAY_OK
```

### 3. YOLO 是否运行 CUDA？

终端启动日志应确认：

```text
device=cuda:0
```

### 4. frame_age 是否正常？

建议至少确认：

```text
不是秒级延迟
```

### 5. 丢失人员是否归零？

```text
linear.x = 0
angular.z = 0
```

以上正常后，再考虑连接真实底盘。

---

# 18. 常见问题

## 问题 1：CAMERA_RELAY_OK 不出现

先检查 Windows 转发 PowerShell 是否仍然打开。

重新执行：

```powershell
PowerShell -ExecutionPolicy Bypass -File scripts\start_camera_relay_windows.ps1
```

再在 WSL：

```bash
WIN_IP=$(awk '/nameserver/{print $2; exit}' /etc/resolv.conf)

timeout 2 bash -c "</dev/tcp/$WIN_IP/37778" && echo CAMERA_RELAY_OK
```

---

## 问题 2：WSL 的 Windows IP 变了

WSL / Windows 重启后：

```text
172.xx.xx.xx
```

可能变化。

因此不要长期写死：

```text
172.26.16.1
```

推荐每次：

```bash
WIN_IP=$(awk '/nameserver/{print $2; exit}' /etc/resolv.conf)
```

然后：

```bash
export DAHUA_IP=$WIN_IP
```

---

## 问题 3：识别窗口卡，但 /cmd_vel Hz 很高

首先看：

```text
frame_age
```

如果 frame_age 很大，说明旧帧仍然发生积压。

不要以：

```text
ros2 topic hz /cmd_vel
```

作为唯一实时性判断。

---

## 问题 4：摄像头时间和现实时间差十几秒

需要区分：

### 情况 A

现实动作本身十几秒以后才出现在窗口。

这是：

```text
视频链路延迟
```

### 情况 B

动作几乎实时，但画面上的摄像头时间水印慢十几秒。

这是：

```text
摄像头本机系统时钟没有同步
```

两者不是一个问题。

---

## 问题 5：YOLO 使用 CPU

检查：

```bash
python3 -c "import torch; print(torch.cuda.is_available())"
```

如果 CUDA 正常，可启动时强制：

```text
device:=cuda:0
```

---

# 19. 不再推荐使用的旧摄像头启动方式

旧方案曾使用类似：

```bash
ros2 run person_follower person_follower_node \
  --ros-args \
  -p camera_ip:=172.26.16.1 \
  -p camera_port:=10554 \
  -p camera_subtype:=1 \
  -p device:=cuda:0 \
  -p show_window:=true
```

其中：

```text
10554
```

属于之前 RTSP 转发链路的思路。

当前项目已经改为：

```text
NetSDK + PlaySDK
```

因此以后默认不要再使用：

```text
camera_port:=10554
```

进行人员跟随调试。

---

# 20. 当前相机部分调试结论

目前已经验证：

```text
[✓] Windows 可以访问大华摄像头

[✓] Windows → WSL 37778 转发成功

[✓] WSL 可检测 CAMERA_RELAY_OK

[✓] NetSDK 加载正常

[✓] PlaySDK 加载正常

[✓] 摄像头实时取帧正常

[✓] YOLOv8 可以检测人员

[✓] BoT-SORT 可以分配并跟踪人员 ID

[✓] 目标锁定正常

[✓] /cmd_vel 输出稳定

[✓] 人在左 / 右 / 远 / 近时控制方向正确

[✓] 人离开画面后控制输出可以归零
```

因此：

```text
相机识别与视觉控制部分
```

已经基本完成调试。

下一阶段应该单独进入：

```text
底盘控制调试
```

推荐顺序：

```text
1. 不运行 person_follower
2. 单独启动 chassis_node
3. 手动发送低速 /cmd_vel
4. 验证前进方向
5. 验证后退方向
6. 验证左转方向
7. 验证右转方向
8. 验证停止
9. 标定线速度
10. 标定角速度
11. 最后才连接视觉人员跟随
```

---

# 21. 日常启动命令速查

## Windows PowerShell

```powershell
cd \\wsl.localhost\Ubuntu-22.04\home\zheng\person_follow_ws

PowerShell -ExecutionPolicy Bypass -File scripts\start_camera_relay_windows.ps1
```

保持窗口打开。

---

## WSL Ubuntu

```bash
cd ~/person_follow_ws

source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/netsdk_env.sh

WIN_IP=$(awk '/nameserver/{print $2; exit}' /etc/resolv.conf)

export DAHUA_IP=$WIN_IP
export DAHUA_PORT=37778

ros2 launch person_follower follow.launch.py \
  start_chassis:=false \
  show_window:=true
```

---

## 查看控制输出

新终端：

```bash
source /opt/ros/humble/setup.bash
source ~/person_follow_ws/install/setup.bash

ros2 topic echo /cmd_vel
```

---

## 查看控制频率

```bash
ros2 topic hz /cmd_vel
```

---

## 只测试摄像头延迟

```bash
ros2 run person_follower camera_latency_test \
  --duration 10 \
  --stream extra \
  --snapshot /tmp/dahua_latest.jpg
```

---

# 最终一句话记忆版

以后启动相机识别，只需要记住：

```text
Windows 先开 37778 摄像头转发
        ↓
WSL 加载 ROS + workspace + NetSDK
        ↓
自动获取 Windows IP
        ↓
DAHUA_IP=$WIN_IP
DAHUA_PORT=37778
        ↓
follow.launch.py
```

调试时最重要的实时性指标：

```text
frame_age
```

而不是只看：

```text
/cmd_vel Hz
```
