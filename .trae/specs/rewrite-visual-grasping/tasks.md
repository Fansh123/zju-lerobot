# Tasks

- [x] Task 1: 简化物体检测模块
  - [x] 修改 `wrist_camera.py`：只检测红色正方体，返回像素尺寸
  - [x] 修改 `object_detector.py`：只检测红色正方体

- [x] Task 2: 实现基于尺寸的深度估计
  - [x] 修改 `coordinate_transformer.py`：添加 `estimate_depth()` 方法
  - [x] 使用已知物块边长22mm和相机焦距计算深度

- [x] Task 3: 简化抓取策略
  - [x] 修改 `grasping_strategy.py`：只保留横向抓取
  - [x] 修改关节5旋转方向：从 +π/2 改为 -π/2
  - [x] 删除纵向抓取相关代码

- [x] Task 4: 简化放置策略
  - [x] 修改 `placing_strategy.py`：保留放置到红色方框功能
  - [x] 使用基于尺寸的深度估计

- [x] Task 5: 简化主程序
  - [x] 修改 `visual_grasping.py`：简化交互菜单
  - [x] 移除不需要的选项

# Task Dependencies
- Task 2 依赖 Task 1（需要物块像素尺寸）
- Task 3 依赖 Task 2（需要深度估计）
- Task 4 依赖 Task 2（需要深度估计）
- Task 5 依赖 Task 3, Task 4
