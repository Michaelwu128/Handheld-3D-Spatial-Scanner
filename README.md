# Handheld-3D-Spatial-Scanner（手持3D空間掃描儀）

## 專題資訊


| 項目      | 內容                                                           |
| ------- | ------------------------------------------------------------ |
| 平台      | STM32L475E IoT Node（B-L475E-IOT01A1）                         |
| 書面報告    | `[docs/手持3D空間掃描儀專題期末報告.pdf](docs/手持3D空間掃描儀專題期末報告.pdf)`       |
| 簡報      | `[docs/手持3D空間掃描儀專題期末報告.pptx](docs/手持3D空間掃描儀專題期末報告.pptx)`     |
| Demo 影片 | [https://youtu.be/wLbfHWUSziY](https://youtu.be/wLbfHWUSziY) |


---

## 專案概述

本專案在 **STM32 IoT Node** 上整合 ToF 測距、IMU 姿態融合與 BLE 無線通訊，實作一台可手持的 3D 空間掃描原型。使用者手持開發板移動，系統將 **距離 + 板子朝向** 轉換為 3D 座標，經 BLE 傳至 PC，以 Python 即時顯示點雲。

**核心思路：** ToF 直接量距離（非 IMU 加速度雙重積分）+ Madgwick 估算姿態 → 幾何投影得到 (X, Y, Z)。

---

## 實作功能

### 硬體


| 元件  | 型號                          | 用途                     |
| --- | --------------------------- | ---------------------- |
| 主控  | STM32L475 (B-L475E-IOT01A1) | FreeRTOS、感測整合          |
| 距離  | VL53L0X                     | ToF 測距（mm）             |
| 慣性  | LSM6DSL                     | 加速度 + 陀螺儀              |
| 磁力  | LIS3MDL（板載）                 | 程式支援 9 軸；Demo 版以 6 軸運作 |
| 無線  | BlueNRG-MS (BLE)            | GATT Notify 傳送點座標      |
| 操作  | 藍色 User Button              | 開始／停止掃描                |
| 指示  | PA5 綠燈                      | 掃描中亮起                  |


### 軟體重點

- **姿態融合：** Madgwick AHRS（`Core/Src/ahrs.c`），輸出四元數
- **3D 座標：** 四元數旋轉 ToF 方向向量，減去按鍵基準點
- **FreeRTOS 五 Task：** ToF / Logic / IMU / Telemetry / BLE
- **RTOS API：** Message Queue、Mutex（I2C2 互斥）、Task 優先權分工
- **BLE 傳輸：** 自訂 GATT Service，約 **5 Hz** 送點（12 bytes = 3× float）
- **上電行為：** 韌體自動初始化感測器並開始 BLE 廣播

---

## 系統架構（FreeRTOS）

```
ToF Task (High)     → distQueue → Logic Task → global_tof_dmm
IMU Task (Normal)   → Madgwick → global_q0~q3
Telemetry Task      → 按鍵 / LED / 算 XYZ → blePointQueue (~5 Hz)
BLE Task (Normal)   → MX_BlueNRG_MS_Process + GATT Notify
```

ToF 與 IMU 共用 **I2C2**，以 `i2c2Mutex` 保護。

---

## 開發環境

- **IDE：** STM32CubeIDE
- **RTOS：** FreeRTOS（CMSIS-RTOS v2）
- **PC 端：** Python 3.8+、`bleak`、`matplotlib`

主要原始碼：


| 路徑                                | 說明                         |
| --------------------------------- | -------------------------- |
| `Core/Src/main.c`                 | RTOS Task、座標計算、GATT 服務     |
| `Core/Src/ahrs.c`                 | Madgwick 姿態濾波              |
| `BlueNRG_MS/App/app_bluenrg_ms.c` | BlueNRG 協定栈                |
| `BlueNRG_MS/App/sensor.c`         | BLE 連線、Connection Interval |
| `ble_3d_scanner.py`               | PC 端 BLE 接收與 3D 繪圖         |


---

## 如何使用（Demo 流程）

### 1. 燒錄韌體

1. 以 STM32CubeIDE 開啟本專案並 Build
2. 燒錄至 B-L475E-IOT01A1
3. 開發板 **USB 上電** → 韌體自動執行、BLE 開始廣播

### 2. 安裝 PC 端套件

```bash
pip install bleak matplotlib
```

### 3. 設定 BLE MAC 位址

編輯 `ble_3d_scanner.py` 中的 `DEVICE_ADDRESS`：

```python
DEVICE_ADDRESS = "C4:CA:07:9A:93:BA"  # 請改為你的開發板 MAC
```

### 4. 連線與掃描

```bash
python ble_3d_scanner.py
```

1. 確認終端機顯示 `連線成功`
2. 按開發板 **藍色按鈕** → **PA5 綠燈亮** → 開始記錄點雲
3. 手持板子掃描，PC 即時顯示 3D 軌跡
4. 再按藍鍵 → 綠燈滅 → 停止記錄

**注意：**

- 掃描前請確認 **手機藍牙未佔用** 開發板連線
- 小範圍、以感測器為中心旋轉的掃描效果較佳

### 5. 3D 畫布快捷鍵


| 按鍵  | 功能         |
| --- | ---------- |
| `1` | YZ 平面（側視）  |
| `2` | XZ 平面（正視）  |
| `3` | XY 平面（俯視）  |
| `0` | 恢復預設 3D 視角 |


---

## 參考文獻

1. Madgwick, S. O. H. (2010). *An efficient orientation filter for inertial and inertial/magnetic sensor arrays.* [https://x-io.co.uk/downloads/madgwick_internal_report.pdf](https://x-io.co.uk/downloads/madgwick_internal_report.pdf)
2. STMicroelectronics. B-L475E-IOT01A1 Discovery kit User Manual
3. STMicroelectronics. STM32L475VG Datasheet
4. AHRS Python — Madgwick Filter: [https://ahrs.readthedocs.io/en/latest/filters/madgwick.html](https://ahrs.readthedocs.io/en/latest/filters/madgwick.html)

