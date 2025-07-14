# URDF机器人关节控制器 (Isaac Gym版本)

一个基于PyQt5和Isaac Gym的URDF机器人关节可视化和控制工具，支持实时操控机器人关节并观察仿真效果。

## 功能特性

- 🤖 加载URDF机器人模型
- 🎮 实时关节位置控制（滑块界面）
- 📊 关节状态监控和显示
- 🔄 自动同步关节目标值和当前值
- 🎯 关节限位检查和显示
- 📱 响应式GUI布局
- ⚡ GPU加速物理仿真

## 系统要求

### 硬件要求
- NVIDIA GPU（支持CUDA）
- 至少4GB显存
- Intel/AMD x64处理器

### 软件要求
- Python 3.8
- CUDA Toolkit 11.x
- Ubuntu 20.04

## 依赖库安装

### 创建虚拟环境（推荐）
```
conda create -n linker python=3.8
conda activate linker

# 安装依赖
pip install PyQt5 numpy urdfpy
```
### PyTorch（GPU版本）
按照[这些说明](https://pytorch.org/get-started/locally/)安装Pytorch。

### Isaac Gym（需要从NVIDIA开发者网站下载）
从[官网](https://developer.nvidia.com/isaac-gym)下载Isaac Gym Preview 4版本，然后按照文档中的安装说明进行安装  
```
cd isaacgym/python
pip install -e .
```


### 核心依赖
```bash
# PyQt5界面库
pip install PyQt5

# 数值计算
pip install numpy

# URDF解析
pip install urdfpy

```


## 使用方法

### 启动程序
```bash
python isaac_urdf.py
```

### 基本操作流程
1. **加载URDF文件**：点击"Load URDF"按钮选择机器人URDF文件
2. **控制关节**：使用左侧滑块调节各关节角度
3. **监控状态**：右侧面板显示当前关节位置和仿真状态
4. **重置位置**：点击"Reset Positions"恢复初始姿态

### 界面说明
- **左侧面板**：URDF文件加载和关节控制滑块
- **右侧面板**：仿真状态显示和当前关节位置监控
- **底部状态栏**：显示当前操作状态和错误信息

## 注意事项

⚠️ **重要提醒**：
1. 确保URDF文件路径中不含中文字符
2. 第一次运行Isaac Gym可能需要较长初始化时间
3. 如果出现GPU内存不足，可以尝试减少仿真复杂度
4. 某些复杂URDF模型可能需要调整物理参数

## 常见问题

### Q: 程序启动时报错"Failed to create PhysX simulation"
A: 检查NVIDIA驱动和CUDA是否正确安装，确保GPU支持CUDA

### Q: 加载URDF后界面卡死
A: 可能是URDF文件过于复杂，尝试简化模型或检查文件格式

### Q: 关节控制不响应
A: 检查"Auto-update simulation"选项是否开启

## 文件结构
```
.
├── isaac_urdf.py    # 主程序文件
├── README.md          # 说明文档
└── [URDF文件目录]     # 存放机器人URDF模型文件
```

