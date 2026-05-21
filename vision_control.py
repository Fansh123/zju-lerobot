import pinocchio as pin
import numpy as np
from pinocchio.visualize import MeshcatVisualizer
import os
import time
from vision import initialize_camera, detect_red_block
from vision_simulation import VisionSimulator, GripperObject


current_q = None


def animate_movement(viz, q_start, q_target, duration=1.0, n_steps=20):
    global current_q
    for i in range(n_steps):
        t = i / n_steps
        t_smooth = t * t * t * (t * (t * 6 - 15) + 10)
        q_interp = q_start + (q_target - q_start) * t_smooth
        viz.display(q_interp)
        current_q = q_interp.copy()
        time.sleep(duration / n_steps)


def get_gripper_position(model, q):
    data = model.createData()
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    gripper_joint_id = model.getJointId("gripper")
    return data.oMi[gripper_joint_id].translation.copy()


def initialize(use_simulation=True):
    global current_q
    print("=== 视觉识别机械臂控制 ===")
    print(f"运行模式: {'仿真' if use_simulation else '真实摄像头'}")

    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(current_dir, "SO-ARM100", "Simulation", "SO100", "so100.urdf")
    mesh_dir = os.path.join(current_dir, "SO-ARM100", "Simulation", "SO100")

    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        urdf_path,
        package_dirs=[mesh_dir]
    )

    viz = MeshcatVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True)
    viz.loadViewerModel()

    q = pin.neutral(model)
    current_q = q.copy()
    viz.display(q)

    vision_sim = None
    red_block = None
    cap = None

    if use_simulation:
        vision_sim = VisionSimulator(target_position=(0.3, 0.0, 0.15))
        red_block = GripperObject(viz.viewer["objects"])
        red_block.spawn(position=(0.3, 0.0, 0.15), size=0.03)
    else:
        cap = initialize_camera(0)
        if cap is None:
            print("警告: 摄像头初始化失败，切换到仿真模式")
            use_simulation = True
            vision_sim = VisionSimulator(target_position=(0.3, 0.0, 0.15))
            red_block = GripperObject(viz.viewer["objects"])
            red_block.spawn(position=(0.3, 0.0, 0.15), size=0.03)
            cap = None

    print("\n初始化完成!")
    print("关节列表:")
    joint_names = [model.names[i] for i in range(1, len(model.names))]
    for i, name in enumerate(joint_names):
        print(f"  q[{i}]: {name}")

    return viz, model, vision_sim, red_block, cap, use_simulation, joint_names


def move_to_grasp_position(viz, model, red_block, duration=1.5):
    global current_q
    q = current_q.copy()

    q_target = q.copy()
    q_target[0] = 0.0
    q_target[1] = 1.2
    q_target[2] = -1.8
    q_target[3] = 0.3
    q_target[4] = 0.0

    print(f"移动到抓取位置: 肩部抬升={q_target[1]:.2f}, 肘部弯曲={q_target[2]:.2f}, 腕部旋转={q_target[4]:.2f}")
    animate_movement(viz, q, q_target, duration=duration)

    if red_block is not None:
        block_pos = red_block.position
        gripper_pos = get_gripper_position(model, current_q)
        print(f"  夹爪位置: ({gripper_pos[0]:.3f}, {gripper_pos[1]:.3f}, {gripper_pos[2]:.3f})")
        print(f"  方块位置: ({block_pos[0]:.3f}, {block_pos[1]:.3f}, {block_pos[2]:.3f})")


def grasp_object(viz, model, red_block, duration=1.0):
    global current_q
    q_current = current_q.copy()

    q_target = q_current.copy()
    q_target[1] -= 0.15
    q_target[2] += 0.1
    print(f"下降: 肩部 {q_current[1]:.2f} -> {q_target[1]:.2f}")
    animate_movement(viz, q_current, q_target, duration=0.5)

    q_grip = q_target.copy()
    q_grip[5] = 0.5
    print("夹爪半闭")
    animate_movement(viz, q_target, q_grip, duration=0.3)

    if red_block is not None:
        gripper_pos = get_gripper_position(model, current_q)
        red_block.attach_to_gripper(gripper_pos)
        print(f"物块已附着到夹爪: ({gripper_pos[0]:.3f}, {gripper_pos[1]:.3f}, {gripper_pos[2]:.3f})")

    q_grip_closed = q_target.copy()
    q_grip_closed[5] = 0.0
    print("夹爪闭合")
    animate_movement(viz, q_grip, q_grip_closed, duration=0.3)

    q_up = q_grip_closed.copy()
    q_up[1] += 0.15
    q_up[2] -= 0.1
    print(f"提升: 肩部 {q_grip_closed[1]:.2f} -> {q_up[1]:.2f}")
    animate_movement(viz, q_grip_closed, q_up, duration=0.5)


def rotate_shoulder_180(viz, model, red_block, duration=2.0):
    global current_q
    q_start = current_q.copy()
    q_target = current_q.copy()
    q_target[0] += np.pi / 2

    q_target[0] = np.clip(q_target[0], model.lowerPositionLimit[0],
                          model.upperPositionLimit[0])

    print(f"肩部旋转: {np.degrees(q_start[0]):.1f}° -> {np.degrees(q_target[0]):.1f}°")

    n_steps = 20
    for i in range(n_steps):
        t = i / n_steps
        t_smooth = t * t * t * (t * (t * 6 - 15) + 10)
        q_interp = q_start + (q_target - q_start) * t_smooth
        viz.display(q_interp)
        current_q = q_interp.copy()

        if red_block is not None and red_block.attached:
            gripper_pos = get_gripper_position(model, current_q)
            red_block.update_position(gripper_pos)

        time.sleep(duration / n_steps)


def release_object(viz, model, red_block, duration=1.0):
    global current_q
    q_current = current_q.copy()

    q_target = q_current.copy()
    q_target[1] -= 0.15
    q_target[2] += 0.1
    print(f"下降: 肩部 {q_current[1]:.2f} -> {q_target[1]:.2f}")
    animate_movement(viz, q_current, q_target, duration=0.5)

    q_grip = q_target.copy()
    q_grip[5] = 1.0
    print("夹爪打开")
    animate_movement(viz, q_target, q_grip, duration=0.3)

    if red_block is not None and red_block.attached:
        gripper_pos = get_gripper_position(model, current_q)
        drop_pos = gripper_pos.copy()
        drop_pos[2] -= 0.02
        red_block.detach(drop_pos)

    time.sleep(0.5)

    q_up = q_grip.copy()
    q_up[1] += 0.15
    q_up[2] -= 0.1
    print(f"提升: 肩部 {q_grip[1]:.2f} -> {q_up[1]:.2f}")
    animate_movement(viz, q_grip, q_up, duration=0.5)


def return_to_initial(viz, model, duration=2.0):
    global current_q
    q_start = current_q.copy()
    q_target = pin.neutral(model)

    print(f"返回初始位置")
    animate_movement(viz, q_start, q_target, duration=duration)


def detect_and_wait(vision_sim, cap, use_simulation, timeout=15):
    if use_simulation:
        return vision_sim.wait_for_detection(timeout=timeout)
    else:
        if cap is None:
            print("使用模拟检测模式")
            return True

        print("正在检测红色方块...")
        start_time = time.time()

        while True:
            ret, frame = cap.read()
            if not ret:
                print("无法读取摄像头帧")
                return None, None, False

            cx, cy, area, detected = detect_red_block(frame)

            elapsed = time.time() - start_time
            if elapsed > timeout:
                print(f"\n检测超时 ({timeout}s)")
                return None, None, False

            if detected:
                print(f"\n检测到红色方块! 位置: ({cx}, {cy}), 面积: {area}")
                return cx, cy, True

            print(f"\r检测中... ({elapsed:.1f}s)", end='', flush=True)


def main():
    use_simulation = True

    viz, model, vision_sim, red_block, cap, use_simulation, joint_names = initialize(use_simulation=use_simulation)

    print("\n" + "="*60)
    print("开始视觉识别与机械臂控制演示")
    print("="*60)

    print("\n步骤1: 视觉识别红色方块")
    detected = detect_and_wait(vision_sim, cap, use_simulation, timeout=15)
    if not detected:
        print("未检测到红色方块，程序退出")
        return

    print("\n步骤2: 移动到抓取位置（横向抓取）")
    move_to_grasp_position(viz, model, red_block, duration=1.5)

    print("\n步骤3: 夹取物块")
    grasp_object(viz, model, red_block, duration=1.0)

    print("\n步骤4: 肩部旋转90度（带着物块）")
    rotate_shoulder_180(viz, model, red_block, duration=2.0)

    print("\n步骤5: 等待5秒")
    print("保持物块位置...")
    time.sleep(5)

    print("\n步骤6: 放下物块")
    release_object(viz, model, red_block, duration=1.0)

    print("\n步骤7: 返回初始状态")
    return_to_initial(viz, model, duration=2.0)

    print("\n" + "="*60)
    print("演示完成!")
    print("="*60)

    print("\n浏览器查看: http://127.0.0.1:7000/static/")
    input("按 Enter 退出...")


if __name__ == "__main__":
    main()