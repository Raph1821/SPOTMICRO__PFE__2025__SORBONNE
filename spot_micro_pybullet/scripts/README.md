# SpotMicro PyBullet 测试环境

## 概述

这个目录包含两个 PyBullet 仿真脚本：

1. **`spot_micro_pybullet_sim.py`** - 原始 ROS 节点版本（需要 ROS 环境）
2. **`env_tester.py`** - 新的独立测试脚本（类似 spot_mini_mini 的 env_tester.py）

## 为什么原来的 pybullet 不能用？

### 问题 1: ROS 依赖
- 原脚本是 ROS 节点，必须运行 `roscore`
- 需要其他节点发送 `/servos_proportional` 话题
- 无法独立运行

### 问题 2: URDF 路径问题
- URDF 使用 `package://` 路径，PyBullet 无法直接解析
- 需要 ROS 环境才能解析 `package://` 路径

### 问题 3: 缺少 GUI 控制
- 原脚本只能被动接收 ROS 话题
- 没有交互式 GUI 参数滑块

## 解决方案：env_tester.py

新创建的 `env_tester.py` 解决了这些问题：

✅ **独立运行** - 不依赖 ROS  
✅ **自动解析 URDF 路径** - 处理 `package://` 路径  
✅ **GUI 控制** - 参数滑块和键盘控制  
✅ **类似 spot_mini_mini** - 使用相同的交互方式  

## 快速开始

### 运行 env_tester.py

```bash
cd src/SPOTMICRO__PFE__2025__SORBONNE__SACLAY/spot_micro_pybullet/scripts
python3 env_tester.py
```

### 命令行参数

```bash
python3 env_tester.py [选项]

选项:
  -hf, --HeightField    使用高度场地形
  -r, --DebugRack        将机器人放在架子上
  -p, --DebugPath        绘制足端路径（待实现）
  --urdf PATH           指定 URDF 文件路径
```

## GUI 使用说明

### 右侧参数滑块

- **位置控制**: `x`, `y`, `z` - 身体位置
- **姿态控制**: `roll`, `pitch`, `yaw` - 身体姿态
- **步态参数**:
  - `Step Length` - 步长
  - `Yaw Rate` - 偏航速率
  - `Lateral Fraction` - 横向分数
  - `Step Velocity` - 步速
  - `Clearance Height` - 抬腿高度
  - `Penetration Depth` - 穿透深度

### 键盘控制

- `A/D` - 左右旋转相机
- `C/F` - 上下俯仰相机
- `Z/X` - 缩放相机
- `ESC` - 退出程序

## 与 spot_mini_mini 的对比

| 特性 | spot_mini_mini | SPOTMICRO (原版) | SPOTMICRO (env_tester.py) |
|------|---------------|-----------------|-------------------------|
| 独立运行 | ✅ | ❌ | ✅ |
| GUI 控制 | ✅ | ❌ | ✅ |
| URDF 路径 | ✅ | ❌ (package://) | ✅ (自动解析) |
| 步态生成 | ✅ (Bezier) | ❌ (需要 ROS) | ⚠️ (待实现) |
| 运动学 | ✅ (Python) | ❌ (C++) | ⚠️ (待实现) |

## 待完成的功能

1. **完整的逆运动学计算**
   - 当前只是框架
   - 需要实现完整的运动学计算

2. **8 相步态生成器**
   - 实现相位管理
   - 实现足端轨迹生成

3. **关节角度应用**
   - 正确映射关节名称
   - 应用计算出的关节角度

## 故障排除

### URDF 加载失败

如果遇到 URDF 加载错误：

1. 检查 STL 文件是否存在：
   ```bash
   ls src/SPOTMICRO__PFE__2025__SORBONNE__SACLAY/spot_micro_pybullet/urdf/stl/
   ```

2. 检查 URDF 文件路径：
   ```bash
   ls src/SPOTMICRO__PFE__2025__SORBONNE__SACLAY/spot_micro_pybullet/urdf/spot_micro_pybullet_gen_ros.urdf
   ```

3. 使用 `--urdf` 参数指定路径：
   ```bash
   python3 env_tester.py --urdf /path/to/urdf/file.urdf
   ```

## 参考文档

- [项目差异和问题分析.md](项目差异和问题分析.md) - 详细的差异分析
- [诊断问题.md](诊断问题.md) - 问题诊断指南

## 相关文件

- `spot_micro_pybullet_sim.py` - 原始 ROS 节点版本
- `env_tester.py` - 新的独立测试脚本
- `项目差异和问题分析.md` - 详细分析文档

