# SO-ARM101 视觉抓取系统

基于腕部摄像头的机械臂视觉抓取与放置系统。

## 功能特性

- **物体检测**: 检测红色物体，支持正方形和圆柱形分类
- **抓取策略**: 支持横向抓取和纵向抓取两种方式
- **放置功能**: 检测100mm×100mm红色方框，将物体放入其中
- **手眼标定**: 完整的手眼标定工具

## 文件说明

| 文件 | 说明 |
|------|------|
| `soarm101_sdk.py` | 机械臂控制SDK |
| `wrist_camera.py` | 腕部摄像头模块 |
| `object_detector.py` | 物体检测模块 |
| `grasping_strategy.py` | 抓取策略模块 |
| `placing_strategy.py` | 放置策略模块 |
| `hand_eye_calibration.py` | 手眼标定模块 |
| `visual_grasping.py` | 主程序入口 |
| `calibration_data/` | 标定数据目录 |

## 使用方法

### 交互模式
```bash
python visual_grasping.py COM18
```

### 演示模式
```bash
python visual_grasping.py COM18 --demo
```

### 执行完整任务
```bash
python visual_grasping.py COM18 --task
```

### 指定物体形状
```bash
python visual_grasping.py COM18 --task --shape square
python visual_grasping.py COM18 --task --shape cylinder
```

### 指定抓取方式
```bash
python visual_grasping.py COM18 --task --grasp horizontal
python visual_grasping.py COM18 --task --grasp vertical
```

### 手眼标定
```bash
python hand_eye_calibration.py COM18
```

## 在代码中使用

```python
from visual_grasping import VisualGrasping

with VisualGrasping('COM18', camera_id=0) as vg:
    # 执行完整任务
    vg.run_full_task(
        shape_filter='square',
        grasp_type='auto'
    )
```

## 硬件要求

- SO-ARM101 机械臂
- SO101 臂载摄像头模块
- Waveshare 控制板
- 12V 电源适配器

## 依赖

```
opencv-python
numpy
pyserial
pyyaml
```

## 注意事项

1. 首次使用需要运行手眼标定
2. 确保工作区域光照稳定
3. 运行时确保机械臂周围无障碍物
