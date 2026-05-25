import cv2
import numpy as np

print("="*50)
print("摄像头窗口测试")
print("="*50)
print("\n1. 程序将弹出一个摄像头窗口")
print("2. 请用鼠标点击该窗口（使其成为活动窗口）")
print("3. 然后按键盘上的 'c' 键拍照")
print("4. 按 'q' 键退出")
print("\n正在打开摄像头...")

cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("错误: 无法打开摄像头")
    exit()

print("摄像头已打开！")
print("\n>>> 请查看任务栏，点击弹出的窗口 <<<\n")

count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        continue
    
    display = frame.copy()
    
    cv2.putText(display, f"Photos: {count}", (10, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
    cv2.putText(display, "Press 'c' to capture, 'q' to quit", (10, 80), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    
    cv2.imshow("CAMERA WINDOW - CLICK HERE FIRST!", display)
    
    key = cv2.waitKey(100) & 0xFF
    
    if key == ord('c'):
        count += 1
        filename = f"test_photo_{count}.jpg"
        cv2.imwrite(filename, frame)
        print(f"已保存: {filename}")
    elif key == ord('q'):
        print("退出")
        break

cap.release()
cv2.destroyAllWindows()
print(f"共拍摄 {count} 张照片")
