# Tasks

- [x] Task 1: 实现视觉伺服居中功能
  - [x] 添加 `visual_servo_center()` 方法：循环检测物块位置并调整机械臂
  - [x] 设置居中阈值（偏差<15像素）
  - [x] 设置调整步长和速度

- [x] Task 2: 实现摄像头前方移动功能
  - [x] 添加 `move_along_camera_axis()` 方法：沿摄像头光轴方向移动
  - [x] 根据末端旋转矩阵计算移动方向

- [x] Task 3: 实现新的抓取流程
  - [x] 修改 `execute_grasp()` 方法：
    1. 设置横向抓取姿态
    2. 视觉伺服居中
    3. Z下降到10mm
    4. 沿摄像头前方移动100mm
    5. 闭合夹爪

- [x] Task 4: 更新主程序
  - [x] 添加新的抓取命令选项
  - [x] 测试完整流程

# Task Dependencies
- Task 2 依赖 Task 1
- Task 3 依赖 Task 1, Task 2
- Task 4 依赖 Task 3
