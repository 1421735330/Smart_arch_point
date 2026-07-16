# Smart Place Point 上位机

一个面向 Windows 的轻量 BLE IMU 上位机。界面使用 Python 自带的 Tkinter，
曲线由 Canvas 原生绘制；运行时仅依赖 Bleak，不引入 Qt、NumPy 或 Matplotlib。

## 已实现功能

- 扫描并连接 BLE 设备，订阅 VG6328A-MS 的 `FFE2` Notify 特征。
- 自动重组被 BLE 拆分或合并的 17 字节数据帧，并从噪声或坏帧中重新同步。
- 校验累加和，显示有效帧数、近 1 秒帧率、校验错误及丢弃字节数。
- 实时显示三轴加速度、三轴角速度和温度。
- 原生绘制加速度与角速度三轴滚动曲线，不需要绘图库。
- 使用六轴互补滤波解算 Roll、Pitch 和相对 Yaw，并判断朝向与运动状态。
- “电路板状态”窗口在三维 X/Y/Z 坐标系中用正方体显示实时姿态，并列出数据延迟、
  合加速度、合角速度和数据质量。
- “校准”按钮执行 2 秒平放静止校准，自动估算并应用加速度与陀螺仪零偏。
- 将缩放值、姿态解算结果和原始值记录为带 UTF-8 BOM 的 CSV，便于 Excel 直接打开。
- 内置演示模式，无硬件时也能检查界面和曲线。
- 可选打包为单个 `SmartPlacePoint.exe`。

## 轻量开发环境

要求 Windows 10/11、Bluetooth Low Energy 和 Python 3.10 或更高版本。

```powershell
cd Software
.\setup.ps1
.\run.ps1
```

`setup.ps1` 会在本目录创建隔离的 `.venv` 并只安装运行依赖。首次启动前需打开
Windows 蓝牙；在软件中点击“扫描设备”，选择目标模块并连接。模块默认透传服务为
`FFE0`，本软件订阅完整 UUID `0000FFE2-0000-1000-8000-00805F9B34FB`。

如 PowerShell 禁止执行本地脚本，可在当前终端临时允许：

```powershell
Set-ExecutionPolicy -Scope Process Bypass
```

## 数据图形与记录

界面以 20 FPS 刷新图形，最多保留 360 个显示点。数据接收和绘图解耦，因此 BLE
通知不会在 GUI 线程中执行。曲线纵轴采用对称自动量程：

- 加速度：X/Y/Z，单位 `g`，固件量程为 ±16 g。
- 角速度：X/Y/Z，单位 `°/s`，固件量程为 ±2000 °/s。
- 温度：单位 `°C`。

点击“开始记录”可选择 CSV 路径，默认目录为 `Software/data`。建议先用“演示模式”
确认本机界面正常，再连接真实硬件。

点击“电路板状态”可打开实时三维解算窗口。正方体表示电路板，并按照解算得到的
Roll、Pitch 和 Yaw 在固定世界坐标系中旋转；X、Y、Z 轴分别显示为红、绿、蓝色。
Roll 和 Pitch 使用重力方向修正陀螺仪
积分误差；当前硬件没有磁力计，因此 Yaw 是每次连接或启动演示模式后的相对角度，
不能作为绝对方位角，静态放置时也可能缓慢漂移。

### IMU 校准

连接设备并确认数据持续更新后，将电路板水平平放、`+Z` 板面朝上，然后点击
“校准”。接下来的 2 秒内不要触碰电路板。软件会检查样本数量、重力幅值和传感器
波动；检测到移动、方向错误或数据不足时会拒绝结果。校准成功后，修正值会用于
实时曲线、姿态解算和 CSV 物理量，CSV 中的 `raw_*` 字段仍保存固件原始值。

## 固件数据协议

每帧固定 17 字节，多字节字段为大端有符号整数：

| 偏移 | 字段 | 换算 |
| ---: | --- | --- |
| 0–1 | 帧头 `AA 55` | — |
| 2–3 | 温度原始值 | `raw / 132.48 + 25` °C |
| 4–9 | X/Y/Z 加速度 | `raw / 2048` g |
| 10–15 | X/Y/Z 角速度 | `raw / 16.4` °/s |
| 16 | 校验和 | 字节 2–15 累加和低 8 位 |

## 测试

协议解析测试只使用 Python 标准库，无需安装 Bleak：

```powershell
cd Software
python -m unittest discover -s tests -v
```

## 构建单文件 EXE（可选）

```powershell
cd Software
.\build.ps1
```

产物位于 `Software/dist/SmartPlacePoint.exe`。PyInstaller 不是交叉编译器，因此
Windows 可执行文件应在 Windows 上构建。打包只是发布步骤，日常开发不需要安装
PyInstaller。

## 目录

```text
Software/
|-- app.pyw                 GUI、实时曲线、CSV 记录和演示模式
|-- ble_transport.py        BLE 扫描、连接和 Notify 接收
|-- imu_protocol.py         帧重组、校验、解码和物理量换算
|-- attitude_solver.py      六轴姿态融合、朝向和运动状态判断
|-- imu_calibration.py      平放静止校准与传感器零偏修正
|-- tests/                  零第三方依赖的协议单元测试
|-- data/                   默认采集数据目录
|-- setup.ps1               创建最小运行环境
|-- run.ps1                 启动上位机
|-- build.ps1               可选的单文件 EXE 打包
|-- requirements.txt        唯一运行依赖 Bleak
`-- requirements-build.txt  可选打包依赖
```
