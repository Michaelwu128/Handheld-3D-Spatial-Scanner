import asyncio
import struct
import threading
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from bleak import BleakClient

# =======================================================
# 1. 設定區
# =======================================================
DEVICE_ADDRESS = "C4:CA:07:9A:93:BA"
#DEVICE_ADDRESS = "FF:6B:06:B4:46:D5"
CHARACTERISTIC_UUID = "d973f2e1-b19e-11e2-9e96-0800200c9a66"
MAX_POINTS = 2000

# 使用 deque(maxlen) 取代 list：超過上限自動丟最舊的點，且 O(1) append
x_data = deque(maxlen=MAX_POINTS)
y_data = deque(maxlen=MAX_POINTS)
z_data = deque(maxlen=MAX_POINTS)

# 保護共用資料：BLE 執行緒（asyncio）與 Matplotlib 主執行緒會同時存取
data_lock = threading.Lock()

# =======================================================
# 2. 藍牙接收與解碼
# =======================================================
def notification_handler(sender, data):
    if len(data) == 12:
        x, y, z = struct.unpack('<fff', data)
        # 移除每點 print：在 10~20Hz 下 print() 的 I/O 會拖慢 asyncio event loop
        print(f"接收座標: X={x:7.1f}, Y={y:7.1f}, Z={z:7.1f}")
        with data_lock:
            x_data.append(x)
            y_data.append(y)
            z_data.append(z)

async def run_ble():
    print(f"🔗 準備使用 MAC 位址 [{DEVICE_ADDRESS}] 強制連線...")
    try:
        async with BleakClient(DEVICE_ADDRESS) as client:
            print("✅ 連線成功！正在開啟資料通道...")
            await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
            print("📡 通道已開啟！請按下開發板的【藍色按鈕】開始掃描！")
            
            while True:
                await asyncio.sleep(1)
    except Exception as e:
        print(f"⚠️ 連線失敗，請確認 MAC 位址正確且開發板已開機: {e}")

def start_ble_thread():
    try:
        asyncio.run(run_ble())
    except Exception as e:
        print(f"💥 藍牙執行緒崩潰: {e}")

threading.Thread(target=start_ble_thread, daemon=True).start()

# =======================================================
# 3. 初始化高效 3D 畫布與快捷鍵
# =======================================================
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.set_title("Wireless 3D Scanner (Press 1,2,3,0 to change views)")

# 🌟 初始化時就畫好空座標軸，之後不再 clear()
ax.set_xlim([-400, 400])
ax.set_ylim([-400, 400])
ax.set_zlim([0, 800])
ax.set_xlabel("X Axis (mm)")
ax.set_ylabel("Y Axis (mm)")
ax.set_zlabel("Z Axis (mm)")

# 建立點雲物件
scatter = ax.scatter([], [], [], c='cyan', marker='o', s=10, alpha=0.8)

def on_key_press(event):
    """恢復你原本超棒的視角切換功能！"""
    if event.key == '1':
        ax.view_init(elev=0, azim=0)
    elif event.key == '2':
        ax.view_init(elev=0, azim=-90)
    elif event.key == '3':
        ax.view_init(elev=90, azim=-90)
    elif event.key == '0':
        ax.view_init(elev=30, azim=-60)

fig.canvas.mpl_connect('key_press_event', on_key_press)

# =======================================================
# 4. 高效畫面更新 (只換資料，不重繪座標軸)
# =======================================================
def update_plot(frame):
    # 在 data_lock 保護下複製一份快照，避免渲染到一半資料被 BLE 執行緒更改
    with data_lock:
        xs = list(x_data)
        ys = list(y_data)
        zs = list(z_data)
    scatter._offsets3d = (xs, ys, zs)
    return scatter,

# interval=50ms (20Hz 渲染) 對應嵌入式端的 20Hz 生產速率
ani = FuncAnimation(fig, update_plot, interval=50, cache_frame_data=False, blit=False)

try:
    print("\n=== 操作說明 ===")
    print("按下 '1' : 切換至 YZ 平面 (側視)")
    print("按下 '2' : 切換至 XZ 平面 (正視)")
    print("按下 '3' : 切換至 XY 平面 (俯視)")
    print("按下 '0' : 恢復 3D 立體視角")
    print("================\n")
    plt.show()
except KeyboardInterrupt:
    print("程式結束")