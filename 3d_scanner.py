import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

# ==========================================
# 1. 設定區 (請修改為你的 COM Port)
# ==========================================
SERIAL_PORT = 'COM5'  # 請確認這是你目前的 COM Port
BAUD_RATE = 115200
MAX_POINTS = 500      

# ==========================================
# 2. 初始化 Serial Port 與資料陣列
# ==========================================
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"成功連線至 {SERIAL_PORT}，等待資料中...")
except Exception as e:
    print(f"無法開啟 Serial Port: {e}")
    print("請確認開發板已接上，且沒有其他軟體 (如 Serial Monitor) 佔用該 Port！")
    exit()

x_data, y_data, z_data = [], [], []

# ==========================================
# 3. 初始化 Matplotlib 3D 畫布與快捷鍵事件
# ==========================================
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.set_title("3D Handheld Scanner (Press 1,2,3,0 to change views)")

scatter = ax.scatter([], [], [], c='c', marker='o', s=10, alpha=0.8)

def init():
    ax.set_xlim(-150, 150)   
    ax.set_ylim(-150, 150)   
    ax.set_zlim(-150, 150)   
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    return scatter,

# --- 新增：鍵盤事件處理函式 ---
def on_key_press(event):
    if event.key == '1':
        ax.view_init(elev=0, azim=0)    # 從 X 軸看過去 -> YZ 平面
        print("[視角切換] YZ 平面 (側視圖)")
    elif event.key == '2':
        ax.view_init(elev=0, azim=-90)  # 從 Y 軸看過去 -> XZ 平面
        print("[視角切換] XZ 平面 (正視圖)")
    elif event.key == '3':
        ax.view_init(elev=90, azim=-90) # 從 Z 軸看過去 -> XY 平面
        print("[視角切換] XY 平面 (俯視圖)")
    elif event.key == '0':
        ax.view_init(elev=30, azim=-60) # 恢復 Matplotlib 預設 3D 視角
        print("[視角切換] 預設 3D 視角")

# 將鍵盤事件綁定到畫布上
fig.canvas.mpl_connect('key_press_event', on_key_press)

# ==========================================
# 4. 更新動畫的迴圈函式
# ==========================================
def update(frame):
    global x_data, y_data, z_data
    
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                parts = line.split(',')
                if len(parts) == 3:
                    x, y, z = map(float, parts)
                    
                    x_data.append(x)
                    y_data.append(y)
                    z_data.append(z)
                    
                    if len(x_data) > MAX_POINTS:
                        x_data.pop(0)
                        y_data.pop(0)
                        z_data.pop(0)
        except Exception as e:
            pass 

    scatter._offsets3d = (x_data, y_data, z_data)
    
    if len(z_data) > 0:
        scatter.set_array(np.array(z_data))
        
    return scatter,

# ==========================================
# 5. 啟動即時動畫
# ==========================================
ani = FuncAnimation(fig, update, init_func=init, interval=50, blit=False)

try:
    # 印出操作說明
    print("\n=== 操作說明 ===")
    print("按下 '1' : 切換至 YZ 平面 (側視)")
    print("按下 '2' : 切換至 XZ 平面 (正視)")
    print("按下 '3' : 切換至 XY 平面 (俯視)")
    print("按下 '0' : 恢復 3D 立體視角")
    print("================\n")
    plt.show()
except KeyboardInterrupt:
    print("程式結束")
finally:
    ser.close()