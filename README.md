# AutoDrive Simulator - 第一人称自动驾驶演示

一个使用 Pygame + OpenCV 实现的**第一人称视角自动驾驶模拟器**，旨在帮助开发者理解车道保持（Lane Keeping）、人机共驾、PID 控制、3D 透视投影等核心概念。

### 主要特性
- 真实感第一人称视角（行车记录仪风格）
- 平滑弯道生成 + 动态路灯参照物（判断居中效果）
- 人机共驾模式：A/D 键持续操控方向盘，轻微修正自动回正，强转向智能换道
- 方向盘物理模拟（渐进转向 + 阻尼回正）
- 实时 OpenCV 边缘检测预览（模拟感知模块）
- 可调参数配置文件（Config.py），轻松修改 FOV、车速、PID 等

### 快速开始
```bash
pip install pygame opencv-python numpy
python autodrive_simu/main.py
