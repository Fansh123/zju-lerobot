# SO-ARM101 腕部摄像头视觉抓取实现计划

## 项目概述

实现基于SO-ARM101臂载摄像头模块的视觉抓取与放置功能。

## 硬件配置

| 组件 | 型号/规格 |
|------|----------|
| 机械臂 | SO-ARM101 专业版 |
| 摄像头 | SO101臂载摄像头模块 |
| 控制器 | 树莓派 / PC |
| 控制板 | Waveshare总线舵机适配器 |

## 功能需求

### 1. 抓取目标
- **正方形物体** - 支持横向抓取和纵向抓取
- **圆柱形物体** - 支持横向抓取和纵向抓取

### 2. 抓取方式
- **横向抓取** - 夹爪从侧面夹取
- **纵向抓取** - 夹爪从顶部夹取

### 3. 放置目标
- **红色方框** - 100mm × 100mm
- 基于视觉识别放置区域
- 将物体放入方框内

---

## 实现步骤

### 第一阶段：摄像头配置与基础功能

#### 1.1 摄像头初始化
- [ ] 创建 `WristCamera` 类
- [ ] 支持树莓派摄像头和USB摄像头
- [ ] 图像采集与显示
- [ ] 图像预处理（畸变校正、去噪）

#### 1.2 目标检测
- [ ] 颜色检测（红色方块、红色方框）
- [ ] 形状检测（正方形、圆柱形）
- [ ] 轮廓提取与中心点计算
- [ ] 目标尺寸估计

#### 1.3 放置区域检测
- [ ] 检测100mm×100mm红色方框
- [ ] 计算方框中心位置
- [ ] 验证放置空间

### 第二阶段：手眼标定

#### 2.1 标定准备
- [ ] 创建 `HandEyeCalibration` 类
- [ ] 准备标定板（棋盘格）
- [ ] 标定流程设计

#### 2.2 标定执行
- [ ] 采集多组标定图像
- [ ] 计算手眼变换矩阵
- [ ] 标定结果验证
- [ ] 参数保存/加载

#### 2.3 坐标变换
- [ ] 图像坐标 → 相机坐标
- [ ] 相机坐标 → 末端坐标
- [ ] 末端坐标 → 基座坐标

### 第三阶段：抓取策略实现

#### 3.1 物体识别与分类
- [ ] 创建 `ObjectDetector` 类
- [ ] 区分正方形和圆柱形
- [ ] 计算物体位姿（位置+方向）
- [ ] 确定最佳抓取点

#### 3.2 横向抓取实现
```
横向抓取流程：
1. 移动到物体上方
2. 调整腕部角度使夹爪朝向侧面
3. 下降到物体高度
4. 横向接近物体
5. 闭合夹爪
6. 提升物体
```

#### 3.3 纵向抓取实现
```
纵向抓取流程：
1. 移动到物体正上方
2. 调整腕部角度使夹爪朝下
3. 下降到物体高度
4. 闭合夹爪
5. 提升物体
```

#### 3.4 抓取姿态规划
- [ ] 根据物体形状选择抓取方式
- [ ] 计算夹爪张开宽度
- [ ] 规划接近轨迹
- [ ] 避免碰撞

### 第四阶段：放置功能实现

#### 4.1 放置区域检测
- [ ] 检测红色方框边界
- [ ] 计算方框中心坐标
- [ ] 验证方框尺寸（100mm×100mm）
- [ ] 检查放置空间是否可用

#### 4.2 放置策略
```
放置流程：
1. 检测红色方框位置
2. 移动到方框上方
3. 调整物体姿态
4. 下降到放置高度
5. 打开夹爪释放物体
6. 提升并返回
```

#### 4.3 精确定位
- [ ] 视觉伺服精确定位
- [ ] 实时反馈调整
- [ ] 放置精度验证

### 第五阶段：完整流程集成

#### 5.1 主控制流程
```
完整流程：
1. 初始化机械臂和摄像头
2. 扫描工作区域寻找目标
3. 识别目标物体（形状、位置）
4. 选择抓取策略（横向/纵向）
5. 执行抓取
6. 检测放置区域
7. 移动到放置位置
8. 放置物体
9. 返回初始位置
```

#### 5.2 状态机设计
- [ ] 定义系统状态
- [ ] 状态转换逻辑
- [ ] 错误处理与恢复
- [ ] 安全保护机制

### 第六阶段：测试与优化

#### 6.1 功能测试
- [ ] 单元测试各模块
- [ ] 抓取成功率测试
- [ ] 放置精度测试

#### 6.2 参数优化
- [ ] 调整视觉检测参数
- [ ] 优化运动轨迹
- [ ] 提高抓取稳定性

---

## 文件结构

```
zju-lerobot/
├── soarm101_sdk.py              # 现有SDK
├── wrist_camera.py              # 腕部摄像头类
├── object_detector.py           # 物体检测类
├── hand_eye_calibration.py      # 手眼标定
├── grasping_strategy.py         # 抓取策略
├── placing_strategy.py          # 放置策略
├── visual_grasping.py           # 主程序
└── calibration_data/            # 标定数据
    ├── camera_params.yaml       # 相机内参
    └── hand_eye.yaml            # 手眼变换
```

---

## 核心类设计

### 1. WristCamera 类
```python
class WristCamera:
    def __init__(self, camera_id=0)
    def get_frame() -> np.ndarray
    def detect_color_object(color='red') -> Tuple[int, int, float]
    def detect_square() -> dict
    def detect_cylinder() -> dict
    def detect_red_frame() -> dict  # 检测100mm红色方框
```

### 2. ObjectDetector 类
```python
class ObjectDetector:
    def detect_objects(frame) -> List[dict]
    def classify_shape(contour) -> str  # 'square' or 'cylinder'
    def get_grasp_pose(obj) -> dict
    def suggest_grasp_type(obj) -> str  # 'horizontal' or 'vertical'
```

### 3. GraspingStrategy 类
```python
class GraspingStrategy:
    def horizontal_grasp(target_pos, arm) -> bool
    def vertical_grasp(target_pos, arm) -> bool
    def execute_grasp(obj_info, arm) -> bool
```

### 4. PlacingStrategy 类
```python
class PlacingStrategy:
    def detect_place_area(camera) -> dict
    def place_object(target_pos, arm) -> bool
    def verify_placement(camera) -> bool
```

### 5. VisualGrasping 主类
```python
class VisualGrasping:
    def __init__(self, arm_port, camera_id)
    def connect()
    def disconnect()
    def scan_and_detect() -> dict
    def grasp_object(grasp_type='auto') -> bool
    def place_in_frame() -> bool
    def run_full_task() -> bool
```

---

## 关键参数配置

### 相机参数
```yaml
camera:
  resolution: [640, 480]
  fps: 30
  focal_length: 500  # 需标定
  
color_detection:
  red_hsv_low: [0, 100, 100]
  red_hsv_high: [10, 255, 255]
  red_hsv_low2: [160, 100, 100]
  red_hsv_high2: [180, 255, 255]
```

### 抓取参数
```yaml
grasping:
  horizontal:
    approach_height: 0.05
    gripper_open: 1.5
    gripper_close: 0.2
  vertical:
    approach_height: 0.1
    gripper_open: 1.5
    gripper_close: 0.3
    
placing:
  frame_size: [100, 100]  # mm
  place_height: 0.02
```

---

## 运动姿态定义

### 横向抓取姿态
```python
HORIZONTAL_GRASP = {
    'shoulder_pan': 0,      # 根据目标位置调整
    'shoulder_lift': 0.3,   # 肩部下压
    'elbow_flex': -0.5,     # 肘部弯曲
    'wrist_flex': 1.2,      # 腕部旋转90度（横向）
    'wrist_roll': 0,        # 腕部滚动
    'gripper': 1.5          # 夹爪打开
}
```

### 纵向抓取姿态
```python
VERTICAL_GRASP = {
    'shoulder_pan': 0,      # 根据目标位置调整
    'shoulder_lift': 0.5,   # 肩部抬升
    'elbow_flex': -0.8,     # 肘部弯曲
    'wrist_flex': 0,        # 腕部朝下
    'wrist_roll': 0,        # 腕部滚动
    'gripper': 1.5          # 夹爪打开
}
```

---

## 安全考虑

1. **运动限制**: 设置关节角度限制，防止碰撞
2. **速度控制**: 视觉伺服时使用较低速度
3. **紧急停止**: 检测异常时立即停止
4. **力反馈**: 监控舵机负载，防止过载
5. **工作空间**: 确保操作在工作空间内

---

## 使用示例

```python
from visual_grasping import VisualGrasping

# 创建实例
vg = VisualGrasping(
    arm_port='/dev/ttyUSB0',  # 树莓派上通常是这个
    camera_id=0
)

# 连接设备
vg.connect()

# 执行完整任务：抓取物体并放入红色方框
vg.run_full_task(
    grasp_type='auto',      # 自动选择抓取方式
    target_shape='square'   # 或 'cylinder'
)

# 或分步执行
vg.scan_and_detect()
vg.grasp_object(grasp_type='horizontal')
vg.place_in_frame()

# 断开连接
vg.disconnect()
```

---

## 实现优先级

| 优先级 | 功能 | 说明 |
|--------|------|------|
| P0 | 摄像头初始化 | 基础功能 |
| P0 | 红色物体检测 | 抓取目标识别 |
| P0 | 纵向抓取 | 最简单的抓取方式 |
| P1 | 手眼标定 | 精确定位必需 |
| P1 | 红色方框检测 | 放置目标识别 |
| P1 | 放置功能 | 完整流程必需 |
| P2 | 横向抓取 | 更灵活的抓取方式 |
| P2 | 形状分类 | 自动选择抓取策略 |
| P3 | 视觉伺服 | 提高精度 |

---

## 预计工作量

| 阶段 | 预计时间 |
|------|---------|
| 摄像头配置与检测 | 2-3小时 |
| 手眼标定 | 2-3小时 |
| 纵向抓取实现 | 2-3小时 |
| 横向抓取实现 | 2-3小时 |
| 放置功能实现 | 2-3小时 |
| 完整流程集成 | 2-3小时 |
| 测试与优化 | 2-3小时 |
| **总计** | **14-21小时** |
