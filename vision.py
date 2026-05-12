import numpy as np

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("警告: OpenCV 未安装，视觉识别功能将使用模拟模式")


def initialize_camera(camera_id=0, width=640, height=480):
    if not CV2_AVAILABLE:
        print("OpenCV 不可用，跳过摄像头初始化")
        return None
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"警告: 无法打开摄像头 {camera_id}，将使用模拟模式")
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    return cap


def detect_red_block(frame, min_area=500):
    if not CV2_AVAILABLE:
        return None, None, 0, False

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None, None, 0, False

    largest_contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest_contour)

    if area < min_area:
        return None, None, area, False

    M = cv2.moments(largest_contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        cx, cy = None, None

    return cx, cy, area, True


def get_red_block_center(cap, timeout=10):
    if cap is None:
        print("摄像头未初始化，返回模拟坐标")
        return 320, 240, True

    if not CV2_AVAILABLE:
        return None, None, False

    start_time = cv2.getTickCount()
    timeout_ticks = timeout * cv2.getTickFrequency()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取摄像头帧")
            return None, None, False

        cx, cy, area, detected = detect_red_block(frame)

        elapsed = cv2.getTickCount() - start_time
        if elapsed > timeout_ticks:
            print("检测超时")
            return None, None, False

        if detected:
            return cx, cy, True

        elapsed_sec = elapsed / cv2.getTickFrequency()
        print(f"\r检测中... ({elapsed_sec:.1f}s)", end='', flush=True)


def draw_detection(frame, cx, cy, area, detected):
    if not CV2_AVAILABLE:
        return None

    if detected and cx is not None and cy is not None:
        cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
        cv2.putText(frame, f"Red Block: ({cx}, {cy})", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Area: {area}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "No red block detected", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    return frame


def is_target_centered(cx, cy, frame_width, frame_height, threshold=50):
    frame_center_x = frame_width // 2
    frame_center_y = frame_height // 2
    distance = np.sqrt((cx - frame_center_x)**2 + (cy - frame_center_y)**2)
    return distance < threshold


def calculate_offset(cx, cy, frame_width, frame_height):
    frame_center_x = frame_width // 2
    frame_center_y = frame_height // 2
    offset_x = (cx - frame_center_x) / frame_width
    offset_y = (cy - frame_center_y) / frame_height
    return offset_x, offset_y


if __name__ == "__main__":
    if not CV2_AVAILABLE:
        print("OpenCV 不可用，无法运行视觉测试")
        exit()

    cap = initialize_camera(0)
    if cap is None:
        print("摄像头初始化失败")
        exit()

    print("开始检测红色方块，按 'q' 退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        cx, cy, area, detected = detect_red_block(frame)
        frame = draw_detection(frame, cx, cy, area, detected)

        cv2.imshow("Red Block Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()