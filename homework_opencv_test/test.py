import cv2

# 打开第一个摄像头
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

# 设置摄像头格式
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 使用 MJPG
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

# 设置 30 FPS
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("无法打开摄像头")
    exit()

print("摄像头打开成功")

while True:
    ret, frame = cap.read()

    if not ret:
        print("无法读取摄像头画面")
        break

    cv2.imshow("USB Camera", frame)

    # 按 q 退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()