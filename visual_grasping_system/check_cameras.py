import cv2
import time

print("检测可用摄像头...")
print("="*50)

available = []
for i in range(10):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret and frame is not None:
            h, w = frame.shape[:2]
            available.append((i, w, h))
            print(f"摄像头 {i}: 可用 ({w}x{h})")
        else:
            available.append((i, 0, 0))
            print(f"摄像头 {i}: 可打开但无法读取图像")
        cap.release()
    else:
        pass

print("="*50)

if len(available) == 0:
    print("未检测到可用摄像头!")
    print("请检查:")
    print("  1. USB摄像头是否正确连接")
    print("  2. 是否有其他程序占用了摄像头")
else:
    print(f"检测到 {len(available)} 个可用摄像头")
    print("\n请选择正确的摄像头ID运行标定程序:")
    for cam_id, w, h in available:
        if w > 0:
            print(f"  python hand_eye_calibration.py COM18 --camera {cam_id}")
