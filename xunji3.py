import cv2
import numpy as np
import serial
cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
cap.set(3, 640)
cap.set(4, 720)

class PositionPID:
    def __init__(self, kp=23.0, ki=0.5, kd=4.0, max_output=190):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.integral = 0.0
        self.last_error = 0.0

    def calculate(self, error):
        """
        输入：位置误差（float）
        输出：控制量（int），范围 [-max_output, max_output]
        """
        # 比例项
        proportional = self.kp * error


        self.integral += error
        # 积分限幅
        if self.ki != 0:
            max_integral = self.max_output / self.ki
            if self.integral > max_integral:
                self.integral = max_integral
            elif self.integral < -max_integral:
                self.integral = -max_integral
        # 注意：原 C 代码计算了 integral_term 但未使用，这里也保留但不用
        # integral_term = self.ki * self.integral

        # 微分项
        derivative = error - self.last_error
        derivative_term = self.kd * derivative

        # 输出 = P + D（不含 I）
        output = proportional + derivative_term

        # 输出限幅
        if output > self.max_output:
            output = self.max_output
        elif output < -self.max_output:
            output = -self.max_output

        # 保存误差供下次微分
        self.last_error = error

        return int(output)

def divide_nine(img):
    h, w = img.shape[:2]
    side = w // 9  # 每个正方形边长（宽度均分）
    # 垂直方向居中
    y_center = h // 2
    top = y_center - side // 2
    bottom = top + side

    rois = []
    for i in range(9):
        x_start = i * side
        x_end = x_start + side
        roi = img[100:bottom, x_start:x_end]
        rois.append(roi)
    return rois,side
WEIGHTS = [-8, -5, -3, -2, 0, 2, 3, 5, 8]
def compute_sensor_num(flags):
    return sum(flags)   # flags 中元素为 0 或 1
def compute_power(flags):
    total = 0
    for i in range(9):
        total += flags[i] * WEIGHTS[i]
    return total
def compute_error(flags):
    sensor_num = compute_sensor_num(flags)
    if sensor_num != 0:
        power = compute_power(flags)
        error = power / sensor_num
        return error
    else:
        return 0.0
def send_to_stm32(value1):
    global ser
    if ser is None or not ser.is_open:
        print("❌ 串口未连接")
        return False

    try:
        data = f"@{int(value1)}\r\n"
        ser.write(data.encode())
        ser.flush()
        print(f"📤 发送:value1={value1}")
        return True
    except Exception as e:
        print(f"❌ 发送失败: {e}")
        return False
try:
    ser = serial.Serial('COM3', 9600, timeout=1)
    print("✅ 串口已连接")
except:
    ser = None
    print("❌ 串口连接失败")
pid = PositionPID(kp=23.0, ki=0.5, kd=4.0, max_output=190)
while True:
    ret, frame = cap.read()
    if not ret:
        print("无法读取帧")
        break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    rois,side = divide_nine(binary)

    # 水平拼接9个区域
    combined = np.hstack(rois)
    # 在每个区域顶部标注序号
    for i in range(9):
        # 每个区域宽度为 side，文字水平居中
        x_center = i * side + side // 2
        cv2.putText(combined, str(i + 1), (x_center - 10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, 128, 1)
    cv2.imshow('Regions', combined)

    # 分别显示每个区域并计算像素和
    sums = []
    flags = []
    for i, roi in enumerate(rois, 1):
        s = np.sum(roi)
        sums.append(s)
        flag = 1 if s < 1000000 else 0
        flags.append(flag)
        cv2.imshow(f'frame{i}', roi)
    print(f"九个窗口标志：{flags}")
    print(f"九个窗口像素和：{sums}")
    error = compute_error(flags)
    send_to_stm32(error)

    control_output = pid.calculate(error)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()