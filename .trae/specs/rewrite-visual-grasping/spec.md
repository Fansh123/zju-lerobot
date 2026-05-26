# 视觉抓取系统重写 Spec

## Why
当前的视觉抓取系统代码复杂且存在多个问题，需要简化并专注于核心功能：抓取22mm红色正方体并放置到红色方框。

## What Changes
- 简化物体检测：只检测红色正方体（边长22mm）
- 简化抓取策略：只保留横向抓取，关节5旋转方向改为 -90°
- 新增深度估计：通过物块尺寸计算深度（已知边长22mm）
- 保留：URDF SDK、手眼标定、放置功能
- 移除：纵向抓取、圆柱体检测、预设深度

## Impact
- Affected code: 
  - `grasping_strategy.py` - 简化为只有横向抓取
  - `placing_strategy.py` - 保留但简化
  - `object_detector.py` - 只检测红色正方体
  - `wrist_camera.py` - 简化检测逻辑
  - `coordinate_transformer.py` - 添加基于尺寸的深度估计
  - `visual_grasping.py` - 简化主程序

## ADDED Requirements

### Requirement: 基于尺寸的深度估计
系统 SHALL 通过已知物块尺寸（22mm）和图像中物块像素尺寸计算深度。

#### Scenario: 深度计算
- **GIVEN** 物块实际边长为22mm
- **WHEN** 检测到物块在图像中的像素尺寸
- **THEN** 系统计算深度 = 实际尺寸 × 焦距 / 像素尺寸

### Requirement: 横向抓取（关节5方向修改）
系统 SHALL 执行横向抓取，关节5旋转 -90°（而非 +90°）。

#### Scenario: 横向抓取执行
- **GIVEN** 检测到红色正方体
- **WHEN** 执行横向抓取
- **THEN** 关节5旋转 -π/2 (-90°)

### Requirement: 红色正方体检测
系统 SHALL 只检测红色正方体，边长22mm。

#### Scenario: 正方体检测
- **GIVEN** 摄像头视野中有红色正方体
- **WHEN** 执行物体检测
- **THEN** 返回正方体的图像坐标和像素尺寸

## MODIFIED Requirements

### Requirement: 抓取策略
系统 SHALL 只提供横向抓取策略。

#### Scenario: 抓取执行
- **GIVEN** 目标位置已计算
- **WHEN** 执行抓取
- **THEN** 
  1. 打开夹爪
  2. 调整关节5为 -90°
  3. 移动到物体上方
  4. 下降到抓取高度
  5. 闭合夹爪
  6. 提升物体

### Requirement: 放置策略
系统 SHALL 将抓取的物体放置到红色方框中。

#### Scenario: 放置执行
- **GIVEN** 物体已被抓取
- **WHEN** 检测到红色方框
- **THEN** 将物体放置到方框中心

## REMOVED Requirements

### Requirement: 纵向抓取
**Reason**: 简化系统，只保留横向抓取
**Migration**: 删除相关代码

### Requirement: 圆柱体检测
**Reason**: 只需要检测正方体
**Migration**: 删除相关代码

### Requirement: 预设深度
**Reason**: 改用基于尺寸的深度估计
**Migration**: 删除 default_depth 参数
