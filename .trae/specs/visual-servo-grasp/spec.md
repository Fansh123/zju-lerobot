# 视觉伺服抓取策略 Spec

## Why
当前的手眼标定结果不准确，导致坐标转换有误差。采用视觉伺服方式可以让机械臂自动调整位置，使物块居中后再抓取，提高抓取成功率。

## What Changes
- 新增视觉伺服抓取策略：自动调整机械臂使物块居中
- 修改抓取流程：居中 → 下降 → 前进 → 抓取
- 保留横向抓取姿态（关节5 = -90°）

## Impact
- Affected code: 
  - `grasping_strategy.py` - 新增视觉伺服抓取方法
  - `visual_grasping.py` - 更新抓取命令

## ADDED Requirements

### Requirement: 视觉伺服居中
系统 SHALL 自动调整机械臂位置，使物块出现在摄像头画面中心。

#### Scenario: 物块居中
- **GIVEN** 机械臂处于横向抓取姿态
- **WHEN** 检测到物块不在画面中心
- **THEN** 系统自动调整机械臂XY位置，直到物块中心与画面中心偏差小于阈值

### Requirement: 居中后抓取流程
系统 SHALL 在物块居中后执行以下步骤：

#### Scenario: 抓取执行
- **GIVEN** 物块已居中
- **WHEN** 执行抓取
- **THEN** 
  1. 控制Z位置下降到10mm
  2. 沿摄像头前方移动100mm
  3. 闭合夹爪抓取

### Requirement: 摄像头前方移动
系统 SHALL 能够沿摄像头光轴方向移动指定距离。

#### Scenario: 前进移动
- **GIVEN** 机械臂当前位置
- **WHEN** 需要沿摄像头前方移动
- **THEN** 根据末端姿态计算移动方向，执行移动

## MODIFIED Requirements

### Requirement: 抓取策略
系统 SHALL 使用视觉伺服方式进行抓取，而非依赖手眼标定坐标转换。

## REMOVED Requirements

### Requirement: 手眼标定坐标转换
**Reason**: 手眼标定结果不准确，改用视觉伺服方式
**Migration**: 保留标定数据用于其他功能，但抓取不再依赖坐标转换
