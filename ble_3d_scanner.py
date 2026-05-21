import asyncio
import struct
import threading
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from bleak import BleakClient

# =======================================================
# 1. 設定區
# =======================================================
DEVICE_ADDRESS = "FF:6B:06:B4:46:D5" 
#DEVICE_ADDRESS = "C2:D4:FE:69:B3:EB"
CHARACTERISTIC_UUID = "d973f2e1-b19e-11e2-9e96-0800200c9a66" 
MAX_POINTS = 500  # 🌟 恢復數量上限，防止系統卡頓

x_data, y_data, z_data = [], [], []

# =======================================================
# 2. 藍牙接收與解碼
# =======================================================
def notification_handler(sender, data):
    if len(data) == 12:
        x, y, z = struct.unpack('<fff', data)
        print(f"接收座標: X={x:7.1f}, Y={y:7.1f}, Z={z:7.1f}")
        
        x_data.append(x)
        y_data.append(y)
        z_data.append(z)
        
        # 🌟 限制最大點數，維持渲染流暢度
        if len(x_data) > MAX_POINTS:
            x_data.pop(0)
            y_data.pop(0)
            z_data.pop(0)

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
    # 🌟 直接替換底層資料，速度極快
    scatter._offsets3d = (x_data, y_data, z_data)
    return scatter,

# interval=30 搭配 blit=False 維持高效刷新
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