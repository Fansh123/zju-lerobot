import cv2
import numpy as np

print("="*50)
print("棋盘格检测测试")
print("="*50)
print("\n程序设置: 8x5 内角点 (对应 9x6 格子)")
print("请将标定板放在摄像头前...")
print("\n如果检测成功，画面上会显示彩色角点")
print("按 'q' 退出\n")

cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("错误: 无法打开摄像头")
    exit()

chessboard_size = (8, 5)

while True:
    ret, frame = cap.read()
    if not ret:
        continue
    
    display = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    found, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    
    if found:
        cv2.drawChessboardCorners(display, chessboard_size, corners, found)
        cv2.putText(display, "CHESSBOARD DETECTED!", (10, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
        cv2.putText(display, "The colored dots show detected corners", (10, 80), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        cv2.putText(display, "NO CHESSBOARD DETECTED", (10, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        cv2.putText(display, "Check: 1) Board in view? 2) Correct size (8x5 corners)? 3) Good lighting?", (10, 80), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    
    cv2.imshow("Chessboard Detection Test", display)
    
    if cv2.waitKey(100) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
