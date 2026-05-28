"""
逆运动学模块
实现末端位置到关节角度的转换
支持解析解和数值解两种方法
"""

import numpy as np
from typing import Optional, Tuple, List


class SOARM101Kinematics:
    """SO-ARM101 运动学模型"""
    
    def __init__(self):
        # DH参数 (根据实际机械臂调整)
        # 格式: [a, alpha, d, theta_offset]
        self.dh_params = [
            [0, np.pi/2, 0.072, 0],      # 关节1: 底座旋转
            [0.125, 0, 0, 0],             # 关节2: 肩部
            [0.125, 0, 0, 0],             # 关节3: 肘部
            [0.065, np.pi/2, 0, 0],       # 关节4: 腕部俯仰
            [0, 0, 0.08, 0],              # 关节5: 腕部旋转
            [0, 0, 0.05, 0],              # 关节6: 夹爪
        ]
        
        # 关节限制 (弧度)
        self.joint_limits = [
            [-1.92, 1.92],   # 关节1
            [-1.75, 1.75],   # 关节2
            [-1.69, 1.69],   # 关节3
            [-1.66, 1.66],   # 关节4
            [-2.74, 2.84],   # 关节5
            [-0.17, 1.75],   # 关节6
        ]
        
        # 连杆长度
        self.link_lengths = [0.072, 0.125, 0.125, 0.065, 0.08, 0.05]
    
    def dh_transform(self, a: float, alpha: float, d: float, theta: float) -> np.ndarray:
        """DH变换矩阵"""
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
    
    def forward_kinematics(self, joint_angles: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        正向运动学：关节角度 → 末端位置和姿态
        
        Args:
            joint_angles: 6个关节角度（弧度）
        
        Returns:
            position: 末端位置 [x, y, z]
            rotation: 末端旋转矩阵 (3x3)
        """
        T = np.eye(4)
        
        for i, (dh, angle) in enumerate(zip(self.dh_params, joint_angles)):
            a, alpha, d, theta_offset = dh
            T_i = self.dh_transform(a, alpha, d, angle + theta_offset)
            T = T @ T_i
        
        position = T[:3, 3]
        rotation = T[:3, :3]
        
        return position, rotation
    
    def get_end_effector_position(self, joint_angles: np.ndarray) -> np.ndarray:
        """获取末端位置"""
        position, _ = self.forward_kinematics(joint_angles)
        return position
    
    def jacobian(self, joint_angles: np.ndarray) -> np.ndarray:
        """
        计算雅可比矩阵
        
        Args:
            joint_angles: 关节角度
        
        Returns:
            J: 6x6 雅可比矩阵
        """
        n = len(joint_angles)
        J = np.zeros((6, n))
        
        T = np.eye(4)
        T_list = [T.copy()]
        
        for i, (dh, angle) in enumerate(zip(self.dh_params, joint_angles)):
            a, alpha, d, theta_offset = dh
            T_i = self.dh_transform(a, alpha, d, angle + theta_offset)
            T = T @ T_i
            T_list.append(T.copy())
        
        T_end = T_list[-1]
        p_end = T_end[:3, 3]
        
        for i in range(n):
            T_i = T_list[i]
            z_i = T_i[:3, 2]
            p_i = T_i[:3, 3]
            
            J[:3, i] = np.cross(z_i, p_end - p_i)
            J[3:, i] = z_i
        
        return J
    
    def inverse_kinematics_numerical(self, target_position: np.ndarray,
                                      initial_angles: np.ndarray = None,
                                      max_iterations: int = 100,
                                      tolerance: float = 1e-3) -> Optional[np.ndarray]:
        """
        数值法逆运动学求解
        
        Args:
            target_position: 目标位置 [x, y, z]
            initial_angles: 初始关节角度
            max_iterations: 最大迭代次数
            tolerance: 位置误差容限
        
        Returns:
            joint_angles: 求解的关节角度，或 None（无解）
        """
        if initial_angles is None:
            initial_angles = np.zeros(6)
        
        joint_angles = initial_angles.copy()
        
        for iteration in range(max_iterations):
            current_pos = self.get_end_effector_position(joint_angles)
            
            error = target_position - current_pos
            error_norm = np.linalg.norm(error)
            
            if error_norm < tolerance:
                return joint_angles
            
            J = self.jacobian(joint_angles)[:3, :]
            
            JJT = J @ J.T
            damping = 0.01 * np.eye(3)
            JJT_damped = JJT + damping
            
            try:
                delta_angles = J.T @ np.linalg.solve(JJT_damped, error)
            except np.linalg.LinAlgError:
                delta_angles = J.T @ error
            
            step_size = min(0.5, error_norm * 0.5)
            joint_angles += delta_angles * step_size
            
            for i in range(6):
                joint_angles[i] = np.clip(joint_angles[i], 
                                          self.joint_limits[i][0], 
                                          self.joint_limits[i][1])
        
        return None
    
    def inverse_kinematics_analytical(self, target_position: np.ndarray,
                                       approach_angle: float = 0) -> Optional[np.ndarray]:
        """
        解析法逆运动学求解（简化版）
        
        适用于固定末端姿态的情况
        
        Args:
            target_position: 目标位置 [x, y, z]
            approach_angle: 接近角度（弧度）
        
        Returns:
            joint_angles: 求解的关节角度，或 None（无解）
        """
        x, y, z = target_position
        
        # 关节1: 底座旋转
        theta1 = np.arctan2(y, x)
        
        # 在x-y平面的投影距离
        r = np.sqrt(x**2 + y**2)
        
        # 减去末端连杆长度
        r_eff = r - self.link_lengths[5] * np.cos(approach_angle)
        z_eff = z - self.link_lengths[5] * np.sin(approach_angle)
        
        # 关节2和3: 肩部和肘部
        L1 = self.link_lengths[1]  # 大臂
        L2 = self.link_lengths[2]  # 小臂
        
        # 到目标的距离
        d = np.sqrt(r_eff**2 + z_eff**2)
        
        # 检查可达性
        if d > L1 + L2 or d < abs(L1 - L2):
            return None
        
        # 余弦定理求肘部角度
        cos_theta3 = (d**2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos_theta3 = np.clip(cos_theta3, -1, 1)
        theta3 = np.arccos(cos_theta3)
        
        # 求肩部角度
        alpha = np.arctan2(z_eff, r_eff)
        beta = np.arctan2(L2 * np.sin(theta3), L1 + L2 * np.cos(theta3))
        theta2 = alpha - beta
        
        # 关节4: 腕部俯仰
        theta4 = approach_angle - theta2 - theta3
        
        # 关节5: 腕部旋转
        theta5 = 0
        
        # 关节6: 夹爪
        theta6 = 0.87
        
        joint_angles = np.array([theta1, theta2, theta3, theta4, theta5, theta6])
        
        # 检查关节限制
        for i in range(6):
            if joint_angles[i] < self.joint_limits[i][0] or \
               joint_angles[i] > self.joint_limits[i][1]:
                return None
        
        return joint_angles
    
    def inverse_kinematics(self, target_position: np.ndarray,
                          initial_angles: np.ndarray = None,
                          method: str = 'hybrid') -> Optional[np.ndarray]:
        """
        逆运动学求解（主入口）
        
        Args:
            target_position: 目标位置 [x, y, z]
            initial_angles: 初始关节角度
            method: 求解方法 ('analytical', 'numerical', 'hybrid')
        
        Returns:
            joint_angles: 求解的关节角度，或 None（无解）
        """
        if method == 'analytical':
            return self.inverse_kinematics_analytical(target_position)
        
        elif method == 'numerical':
            return self.inverse_kinematics_numerical(target_position, initial_angles)
        
        else:  # hybrid
            # 先尝试解析解
            result = self.inverse_kinematics_analytical(target_position)
            if result is not None:
                return result
            
            # 解析解失败，尝试数值解
            return self.inverse_kinematics_numerical(target_position, initial_angles)
    
    def is_reachable(self, target_position: np.ndarray) -> bool:
        """检查目标位置是否可达"""
        result = self.inverse_kinematics(target_position)
        return result is not None
    
    def get_workspace_radius(self) -> Tuple[float, float]:
        """获取工作空间半径范围"""
        total_length = sum(self.link_lengths)
        min_radius = 0.1
        max_radius = total_length * 0.9
        return min_radius, max_radius


class CoordinateTransformer:
    """坐标变换器 - 图像坐标到世界坐标"""
    
    def __init__(self, kinematics: SOARM101Kinematics = None):
        self.kinematics = kinematics or SOARM101Kinematics()
        
        # 相机内参（需要标定）
        self.camera_matrix = np.array([
            [500, 0, 320],
            [0, 500, 240],
            [0, 0, 1]
        ], dtype=np.float64)
        
        # 手眼变换（需要标定）
        self.hand_eye_rotation = np.eye(3)
        self.hand_eye_translation = np.array([0, 0, 0.05])
        
        # 固定工作高度
        self.work_height = 0.05
    
    def set_camera_params(self, camera_matrix: np.ndarray):
        """设置相机内参"""
        self.camera_matrix = camera_matrix
    
    def set_hand_eye_transform(self, rotation: np.ndarray, translation: np.ndarray):
        """设置手眼变换"""
        self.hand_eye_rotation = rotation
        self.hand_eye_translation = translation
    
    def set_work_height(self, height: float):
        """设置固定工作高度"""
        self.work_height = height
    
    def image_to_camera(self, u: int, v: int, depth: float = None) -> np.ndarray:
        """
        图像坐标 → 相机坐标
        
        Args:
            u, v: 图像像素坐标
            depth: 深度（米），如果为None则使用固定工作高度
        
        Returns:
            point_camera: 相机坐标系下的点 [x, y, z]
        """
        if depth is None:
            depth = self.work_height
        
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        
        return np.array([x, y, z])
    
    def camera_to_end_effector(self, point_camera: np.ndarray) -> np.ndarray:
        """
        相机坐标 → 末端坐标
        
        Args:
            point_camera: 相机坐标系下的点
        
        Returns:
            point_ee: 末端坐标系下的点
        """
        point_ee = self.hand_eye_rotation @ point_camera + self.hand_eye_translation
        return point_ee
    
    def image_to_world(self, u: int, v: int, 
                        current_angles: np.ndarray,
                        depth: float = None) -> Optional[np.ndarray]:
        """
        图像坐标 → 世界坐标（基座坐标系）
        
        Args:
            u, v: 图像像素坐标
            current_angles: 当前关节角度
            depth: 深度（米）
        
        Returns:
            point_world: 世界坐标系下的点 [x, y, z]
        """
        # 图像 → 相机
        point_camera = self.image_to_camera(u, v, depth)
        
        # 相机 → 末端
        point_ee = self.camera_to_end_effector(point_camera)
        
        # 末端 → 世界（需要正向运动学）
        position, rotation = self.kinematics.forward_kinematics(current_angles)
        
        point_world = rotation @ point_ee + position
        
        return point_world
    
    def get_target_joint_angles(self, u: int, v: int,
                                  current_angles: np.ndarray,
                                  depth: float = None) -> Optional[np.ndarray]:
        """
        从图像坐标直接计算目标关节角度
        
        Args:
            u, v: 图像像素坐标
            current_angles: 当前关节角度
            depth: 深度（米）
        
        Returns:
            target_angles: 目标关节角度
        """
        target_world = self.image_to_world(u, v, current_angles, depth)
        
        if target_world is None:
            return None
        
        return self.kinematics.inverse_kinematics(target_world, current_angles)


def test_kinematics():
    """测试运动学"""
    print("="*60)
    print("运动学测试")
    print("="*60)
    
    kinematics = SOARM101Kinematics()
    
    # 测试正向运动学
    print("\n1. 正向运动学测试")
    test_angles = np.array([0, 0, 0, 0, 0, 0.87])
    position, rotation = kinematics.forward_kinematics(test_angles)
    print(f"关节角度: {np.degrees(test_angles)}")
    print(f"末端位置: {position}")
    
    # 测试逆运动学
    print("\n2. 逆运动学测试")
    target_pos = np.array([0.2, 0, 0.15])
    result = kinematics.inverse_kinematics(target_pos)
    if result is not None:
        print(f"目标位置: {target_pos}")
        print(f"求解角度: {np.degrees(result)}")
        
        # 验证
        verify_pos = kinematics.get_end_effector_position(result)
        print(f"验证位置: {verify_pos}")
        print(f"误差: {np.linalg.norm(verify_pos - target_pos)*1000:.2f} mm")
    else:
        print("无解")
    
    # 测试坐标变换
    print("\n3. 坐标变换测试")
    transformer = CoordinateTransformer(kinematics)
    
    u, v = 320, 240  # 图像中心
    current_angles = np.zeros(6)
    
    world_pos = transformer.image_to_world(u, v, current_angles)
    print(f"图像坐标: ({u}, {v})")
    print(f"世界坐标: {world_pos}")
    
    target_angles = transformer.get_target_joint_angles(u, v, current_angles)
    print(f"目标关节角度: {np.degrees(target_angles) if target_angles is not None else '无解'}")


if __name__ == "__main__":
    test_kinematics()
