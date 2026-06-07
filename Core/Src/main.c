/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "app_bluenrg_ms.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "b_l475e_iot01a1.h"   // <--- 換成新的 BSP 標頭檔
#include "vl53l0x_def.h"
#include "vl53l0x_api.h"
#include <stdio.h>
#include <stdlib.h> // <--- 補上這行給 abs() 使用
#include <math.h>   // <--- 新增這行：供姿態運算使用

// === 修改：BLE 相關標頭檔 (補上 bluenrg_def.h 與 bluenrg_gap.h) ===
#include "bluenrg_def.h"
#include "hci.h"
#include "bluenrg_gap.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_gap_aci.h"
#include "sensor.h"  // <--- 新增這行：讓系統認識 setConnectable() 函式
#include "ahrs.h"    // 9 軸 Madgwick AHRS
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 定點運算：將 mm 轉為 0.1mm (deci-millimeter) */
#define MM_TO_DMM(x) ((uint32_t)(x) * 10)
#define FILTER_SIZE 1  // <--- 將這裡改成 1，關閉平滑濾波，保留真實的邊緣跳變
/* 設 1：UART 輸出 GYRO_RAW / MAG_RAW（測量 B+C 用）；正式使用請設 0 */
#define DEBUG_SENSOR_RAW 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ToF_Sensor_Task */
osThreadId_t ToF_Sensor_TaskHandle;
const osThreadAttr_t ToF_Sensor_Task_attributes = {
  .name = "ToF_Sensor_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Logic_Control_T */
osThreadId_t Logic_Control_THandle;
const osThreadAttr_t Logic_Control_T_attributes = {
  .name = "Logic_Control_T",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Telemetry_Task */
osThreadId_t Telemetry_TaskHandle;
const osThreadAttr_t Telemetry_Task_attributes = {
  .name = "Telemetry_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for IMU_Task */
osThreadId_t IMU_TaskHandle;
const osThreadAttr_t IMU_Task_attributes = {
  .name = "IMU_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myBinarySem01 */
osSemaphoreId_t myBinarySem01Handle;
const osSemaphoreAttr_t myBinarySem01_attributes = {
  .name = "myBinarySem01"
};
/* USER CODE BEGIN PV */
osMessageQueueId_t distQueueHandle;
VL53L0X_Dev_t Dev;

volatile uint32_t global_display_dmm = 0;
volatile uint32_t global_tof_dmm = 0;     // ToF 絕對距離（deci-mm），供 Task04 計算 3D 座標
volatile uint32_t idle_tick_count = 0;

// === 新增：I2C2 Mutex 宣告 ===
osMutexId_t i2c2MutexHandle;
const osMutexAttr_t i2c2Mutex_attributes = {
  .name = "i2c2Mutex"
};

// === LSM6DSL I2C 定義 ===
#define LSM6DSL_ADDR         (0x6A << 1) // 8-bit Address: 0xD4
#define LSM6DSL_WHO_AM_I     0x0F
#define LSM6DSL_CTRL1_XL     0x10  // 加速度計控制
#define LSM6DSL_CTRL2_G      0x11  // 陀螺儀控制
#define LSM6DSL_OUTX_L_G     0x22  // 陀螺儀數據起始位址
#define LSM6DSL_OUTX_L_XL    0x28  // 加速度計數據起始位址

// === LIS3MDL 磁力計 I2C 定義 ===
// B-L475E-IOT01A1 板上 SA1 接 VDD → 7-bit 0x1E → 8-bit 0x3C
#define LIS3MDL_ADDR         (0x1E << 1) // 8-bit: 0x3C
#define LIS3MDL_ADDR_ALT     (0x1C << 1) // 8-bit: 0x38（備用，SA1=GND 時）
#define LIS3MDL_WHO_AM_I_REG 0x0F        // 預期回傳 0x3D
#define LIS3MDL_CTRL_REG1    0x20
#define LIS3MDL_CTRL_REG2    0x21
#define LIS3MDL_CTRL_REG3    0x22
#define LIS3MDL_CTRL_REG4    0x23
#define LIS3MDL_OUT_X_L      0x28        // 連續讀 6 bytes 需 OR 0x80 啟用自動遞增

// === 新增：IMU 資料結構與全域變數 ===
typedef struct {
    int16_t ax, ay, az; // 加速度
    int16_t gx, gy, gz; // 陀螺儀
} IMU_Data_t;

volatile IMU_Data_t global_imu_data = {0};

// === 9 軸 AHRS 四元數輸出（取代 roll/pitch，支援完整 Yaw）===
// q0 = 純量部分；q1,q2,q3 = 向量部分
// 由 IMU_Task 寫入，Task04 讀取；使用 volatile 保證可見性
volatile float global_q0 = 1.0f;
volatile float global_q1 = 0.0f;
volatile float global_q2 = 0.0f;
volatile float global_q3 = 0.0f;

// === 新增：掃描狀態旗標 ===
volatile uint8_t is_scanning = 0; // 0: 待機 (Standby), 1: 掃描中 (Scanning)
// === BLE 點雲傳輸 Handles ===
uint16_t ScannerServHandle;
uint16_t PointCharHandle;

// === BLE 點資料結構（取代 ble_send_ready 旗標，用佇列傳遞）===
typedef struct {
    float px, py, pz;
} BlePoint_t;

// 深度為 1 的覆寫佇列：永遠只保留最新的點
osMessageQueueId_t blePointQueueHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartIMUTask(void *argument);

/* USER CODE BEGIN PFP */
void StartIMUTask(void *argument);
void Add_Scanner_Service(void); // <--- 新增這行
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DFSDM1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_BlueNRG_MS_Init();
  /* USER CODE BEGIN 2 */
    // === 修改：喚醒按鍵，但使用「純讀取模式」不要用中斷！ ===
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
    BSP_LED_Init(LED2);
    // ===============================================

    Add_Scanner_Service();
    Set_DeviceConnectable();
    printf("[BLE] 廣播程序已啟動！等待連線...\r\n");

    /* LIS3MDL 初始化改由 StartIMUTask 統一處理（避免重複寫 I2C、位址不一致） */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  i2c2MutexHandle = osMutexNew(&i2c2Mutex_attributes); // 建立 I2C2 互斥鎖
  if (i2c2MutexHandle == NULL) {
      // Mutex 建立失敗的錯誤處理，一般來說不會發生
      Error_Handler();
  }
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of myBinarySem01 */
  myBinarySem01Handle = osSemaphoreNew(1, 1, &myBinarySem01_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  // 建立一個可以容納 10 個 uint32_t 的 Queue
  distQueueHandle = osMessageQueueNew(10, sizeof(uint32_t), NULL);
  // 建立深度 1 的 BLE 點資料佇列（Fire-and-Forget 覆寫語意）
  blePointQueueHandle = osMessageQueueNew(1, sizeof(BlePoint_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ToF_Sensor_Task */
  ToF_Sensor_TaskHandle = osThreadNew(StartTask02, NULL, &ToF_Sensor_Task_attributes);

  /* creation of Logic_Control_T */
  Logic_Control_THandle = osThreadNew(StartTask03, NULL, &Logic_Control_T_attributes);

  /* creation of Telemetry_Task */
  Telemetry_TaskHandle = osThreadNew(StartTask04, NULL, &Telemetry_Task_attributes);

  /* creation of IMU_Task */
  IMU_TaskHandle = osThreadNew(StartIMUTask, NULL, &IMU_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  // --- 加入開機測試字串 ---
  char boot_msg[] = "\r\n--- System Booted Successfully ---\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)boot_msg, sizeof(boot_msg)-1, HAL_MAX_DELAY);
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00F12981;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|GPIO_PIN_5|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|SPSGRF_915_SDN_Pin
                          |ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin PA5 SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|GPIO_PIN_5|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D4_Pin */
  GPIO_InitStruct.Pin = ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin SPSGRF_915_SDN_Pin
                           ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|SPSGRF_915_SDN_Pin
                          |ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
extern void hci_notify_asynch_evt(void* pdata);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // 🌟 刪除按鍵判斷，只處理藍牙晶片 (PE6) 觸發的底層事件
  if (GPIO_Pin == GPIO_PIN_6)
  {
    hci_notify_asynch_evt(NULL);
  }
}
void Add_Scanner_Service(void)
{
    tBleStatus ret;

    // 1. 建立 Custom Service (使用隨機生成的 128-bit UUID)
    // UUID: d973f2e0-b19e-11e2-9e96-0800200c9a66
    uint8_t service_uuid[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe0,0xf2,0x73,0xd9};
    ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid, PRIMARY_SERVICE, 7, &ScannerServHandle);

    if (ret == BLE_STATUS_SUCCESS) {
        // 2. 建立 Characteristic (容量為 12 Bytes，具備 Notify 屬性)
        // UUID: d973f2e1-b19e-11e2-9e96-0800200c9a66
        uint8_t char_uuid[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9};
        aci_gatt_add_char(ScannerServHandle, UUID_TYPE_128, char_uuid, 12,
                          CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE,
                          0, 16, 0, &PointCharHandle);
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  static char dbg_msg[64];
  BlePoint_t point;
  float point_data[3];

  int len = snprintf(dbg_msg, sizeof(dbg_msg), "\r\n[BLE Task] 啟動！\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)dbg_msg, len, 100);

  /* Infinite loop */
  for(;;)
  {
    // 優先處理 BlueNRG 底層 HCI 事件（必須高頻呼叫）
    MX_BlueNRG_MS_Process();

    // 非阻塞地取點（timeout=0：佇列空就直接跳過，不等待）
    if (osMessageQueueGet(blePointQueueHandle, &point, NULL, 0) == osOK) {
        point_data[0] = point.px;
        point_data[1] = point.py;
        point_data[2] = point.pz;

        tBleStatus ret = aci_gatt_update_char_value(
            ScannerServHandle, PointCharHandle, 0, 12, (uint8_t*)point_data);

        if (ret == 0x64) {
            // BLE TX buffer 滿：丟棄這個點，讓 BlueNRG 喘口氣，絕不卡死
            osDelay(10);
        } else if (ret != BLE_STATUS_SUCCESS) {
            len = snprintf(dbg_msg, sizeof(dbg_msg), "[BLE Err] 0x%02X\r\n", ret);
            HAL_UART_Transmit(&huart1, (uint8_t*)dbg_msg, len, 50);
        }
    }

    osDelay(5);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the ToF_Sensor_Task thread.
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  uint32_t filter_buf[FILTER_SIZE] = {0};
  uint8_t buf_idx = 0;
  uint32_t sum = 0;
  VL53L0X_RangingMeasurementData_t RangingData;

  // 延長物理重置時間
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin, GPIO_PIN_RESET);
  osDelay(50);
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin, GPIO_PIN_SET);
  osDelay(200);

  Dev.I2cHandle = &hi2c2;
  Dev.I2cDevAddr = 0x52;

  if (HAL_I2C_IsDeviceReady(&hi2c2, Dev.I2cDevAddr, 2, 100) != HAL_OK) {
      printf("\r\n[ToF Error] Sensor dead! Suspending task...\r\n");
      osThreadSuspend(ToF_Sensor_TaskHandle);
  }

  // 加上 1 秒的 Mutex 超時限制
  if (osMutexAcquire(i2c2MutexHandle, 1000) == osOK) {
      VL53L0X_WaitDeviceBooted(&Dev);
      VL53L0X_DataInit(&Dev);
      VL53L0X_StaticInit(&Dev);
      uint32_t refSpadCount;
      uint8_t isApertureSpads, VhvSettings, PhaseCal;
      VL53L0X_PerformRefSpadManagement(&Dev, &refSpadCount, &isApertureSpads);
      VL53L0X_PerformRefCalibration(&Dev, &VhvSettings, &PhaseCal);
      VL53L0X_SetDeviceMode(&Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
      VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&Dev, 33000);
      osMutexRelease(i2c2MutexHandle);
      printf("\r\n[ToF] Init Complete. Starting Loop...\r\n");
  } else {
      printf("\r\n[ToF Error] Mutex Timeout! Suspending task...\r\n");
      osThreadSuspend(ToF_Sensor_TaskHandle);
  }

  for(;;)
  {
    if (osMutexAcquire(i2c2MutexHandle, 50) == osOK) {
        VL53L0X_PerformSingleRangingMeasurement(&Dev, &RangingData);
        osMutexRelease(i2c2MutexHandle);

        if(RangingData.RangeMilliMeter > 0 && RangingData.RangeMilliMeter < 8000) {
           uint32_t current_dmm = MM_TO_DMM(RangingData.RangeMilliMeter);
           sum -= filter_buf[buf_idx];
           filter_buf[buf_idx] = current_dmm;
           sum += filter_buf[buf_idx];
           buf_idx = (buf_idx + 1) % FILTER_SIZE;
           uint32_t stable_dist = sum / FILTER_SIZE;
           osMessageQueuePut(distQueueHandle, &stable_dist, 0, 10);
        }
    }
    osDelay(20);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Logic_Control_T thread.
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  uint32_t d_now = 0;
  uint32_t d_base = 0;
  uint8_t last_scan_state = 0;

  for(;;)
  {
    // 如果 ToF 當機，這裡會安全地 Block 住，不消耗任何 CPU
    if (osMessageQueueGet(distQueueHandle, &d_now, NULL, osWaitForever) == osOK)
    {
      global_tof_dmm = d_now;  // 始終保持絕對距離給 Task04 使用

      // 偵測掃描狀態的「瞬間開啟」，執行 Auto-Tare 原點歸零
      if (is_scanning == 1 && last_scan_state == 0) {
          d_base = d_now;
      }
      last_scan_state = is_scanning;

      global_display_dmm = (d_now >= d_base) ? (d_now - d_base) : (d_base - d_now);

      // 註：實體 LED 切換已移交給 StartTask04 處理
    }
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the Telemetry_Task thread.
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  static char uart_buf[128];
  uint8_t last_state = 0;
  uint8_t btn_prev = 0;
  int len;
  BlePoint_t point;

  // 1. 把剛剛從 main 刪掉的變數，改在這邊宣告
  uint8_t raw_mag[6];
  int16_t mag_x_raw = 0, mag_y_raw = 0, mag_z_raw = 0;
  extern I2C_HandleTypeDef hi2c2;

  len = snprintf(uart_buf, sizeof(uart_buf), "\r\n[Telemetry Task] 啟動！\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, len, 100);

  float px_base = 0.0f, py_base = 0.0f, pz_base = 0.0f;

  for(;;)
  {
    // --- 這裡放我們新加的：讀取磁力計並列印 ---
    /*if (HAL_I2C_Mem_Read(&hi2c2, (0x1E << 1), 0x28, I2C_MEMADD_SIZE_8BIT, raw_mag, 6, 50) == HAL_OK)
    {
        mag_x_raw = (int16_t)((raw_mag[1] << 8) | raw_mag[0]);
        mag_y_raw = (int16_t)((raw_mag[3] << 8) | raw_mag[2]);
        mag_z_raw = (int16_t)((raw_mag[5] << 8) | raw_mag[4]);

        len = snprintf(uart_buf, sizeof(uart_buf), "MAG_RAW:%d,%d,%d\r\n", mag_x_raw, mag_y_raw, mag_z_raw);
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, len, 10);
    }*/

    // === 2. 按鍵輪詢 ===
    uint8_t btn_now = BSP_PB_GetState(BUTTON_USER);
    if (btn_now == 1 && btn_prev == 0) {
        is_scanning = !is_scanning;
    }
    btn_prev = btn_now;

    // === 3. 狀態切換回饋 ===
    if (is_scanning != last_state) {
        if (is_scanning) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            float d_base_mm = (float)global_tof_dmm / 10.0f;
            float q0_b = global_q0, q1_b = global_q1;
            float q2_b = global_q2, q3_b = global_q3;
            px_base = -2.0f * (q1_b * q3_b + q0_b * q2_b) * d_base_mm;
            py_base = -2.0f * (q2_b * q3_b - q0_b * q1_b) * d_base_mm;
            pz_base = -(1.0f - 2.0f * (q1_b * q1_b + q2_b * q2_b)) * d_base_mm;

            len = snprintf(uart_buf, sizeof(uart_buf), "\r\n>>> 掃描開始 <<< base=(%.1f,%.1f,%.1f)\r\n", px_base, py_base, pz_base);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, len, 50);
        } else {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            len = snprintf(uart_buf, sizeof(uart_buf), "\r\n>>> 掃描停止 <<<\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, len, 50);
        }
        last_state = is_scanning;
    }

    // === 4. 資料打包 ===
    if (is_scanning) {
        float d_now_mm = (float)global_tof_dmm / 10.0f;
        float q0_l = global_q0;
        float q1_l = global_q1;
        float q2_l = global_q2;
        float q3_l = global_q3;

        float px_abs = -2.0f * (q1_l * q3_l + q0_l * q2_l) * d_now_mm;
        float py_abs = -2.0f * (q2_l * q3_l - q0_l * q1_l) * d_now_mm;
        float pz_abs = -(1.0f - 2.0f * (q1_l * q1_l + q2_l * q2_l)) * d_now_mm;

        point.px = px_abs - px_base;
        point.py = py_abs - py_base;
        point.pz = pz_abs - pz_base;

        osMessageQueueReset(blePointQueueHandle);
        osMessageQueuePut(blePointQueueHandle, &point, 0, 0);
    }

    osDelay(50);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartIMUTask */
/**
* @brief Function implementing the IMU_Task thread.
*/
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void *argument)
{
  /* USER CODE BEGIN StartIMUTask */
  uint8_t imu_ok  = 0; // LSM6DSL 是否成功初始化
  uint8_t mag_ok  = 0; // LIS3MDL 是否成功初始化
  uint16_t mag_i2c_addr = LIS3MDL_ADDR;
  uint8_t whoAmI  = 0;
  uint8_t ctrl    = 0;
  uint8_t raw_imu[12];
  uint8_t raw_mag[6];

  // 磁力計原始值與校準後的物理值變數
  int16_t mx_raw = 0, my_raw = 0, mz_raw = 0;
  float mx_cal = 0.0f, my_cal = 0.0f, mz_cal = 0.0f;

  // 本板硬磁校正（測量 C ×2 取 min/max 平均）
  const int16_t mag_x_offset = -1765;
  const int16_t mag_y_offset = -1183;
  const int16_t mag_z_offset = -1475;
  // 陀螺零偏（測量 B：板子平放不動）
  const int16_t gyro_x_bias = 15;
  const int16_t gyro_y_bias = -29;
  const int16_t gyro_z_bias = 6;

  osDelay(100); // 等待其他任務（特別是 ToF）完成開機初始化

  if (osMutexAcquire(i2c2MutexHandle, 200) == osOK) {

      // ----- 初始化 LSM6DSL -----
      HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_ADDR, LSM6DSL_WHO_AM_I, 1, &whoAmI, 1, 100);
      if (whoAmI == 0x6A) {
          ctrl = 0x40; // ODR=104Hz, FS=±2g
          HAL_I2C_Mem_Write(&hi2c2, LSM6DSL_ADDR, LSM6DSL_CTRL1_XL, 1, &ctrl, 1, 100);
          ctrl = 0x4C; // ODR=104Hz, FS=±2000dps
          HAL_I2C_Mem_Write(&hi2c2, LSM6DSL_ADDR, LSM6DSL_CTRL2_G,  1, &ctrl, 1, 100);
          imu_ok = 1;
          printf("[IMU] LSM6DSL OK (WHO_AM_I=0x6A)\r\n");
      } else {
          printf("[IMU] LSM6DSL not found (0x%02X)\r\n", whoAmI);
      }


      // ----- 初始化 LIS3MDL（先試 0x3C，失敗再試 0x38）-----
       mag_i2c_addr = LIS3MDL_ADDR;
       whoAmI = 0;
       HAL_I2C_Mem_Read(&hi2c2, mag_i2c_addr, LIS3MDL_WHO_AM_I_REG,
                        I2C_MEMADD_SIZE_8BIT, &whoAmI, 1, 100);
       if (whoAmI != 0x3D) {
           mag_i2c_addr = LIS3MDL_ADDR_ALT;
           whoAmI = 0;
           HAL_I2C_Mem_Read(&hi2c2, mag_i2c_addr, LIS3MDL_WHO_AM_I_REG,
                            I2C_MEMADD_SIZE_8BIT, &whoAmI, 1, 100);
       }
       if (whoAmI == 0x3D) {
           // CTRL_REG1: XY ultra-high performance, ODR=80Hz
           ctrl = 0x7C;
           HAL_I2C_Mem_Write(&hi2c2, mag_i2c_addr, LIS3MDL_CTRL_REG1,
                             I2C_MEMADD_SIZE_8BIT, &ctrl, 1, 100);
           // CTRL_REG2: Full scale ±4 gauss
           ctrl = 0x00;
           HAL_I2C_Mem_Write(&hi2c2, mag_i2c_addr, LIS3MDL_CTRL_REG2,
                             I2C_MEMADD_SIZE_8BIT, &ctrl, 1, 100);
           // CTRL_REG3: Continuous measurement mode
           ctrl = 0x00;
           HAL_I2C_Mem_Write(&hi2c2, mag_i2c_addr, LIS3MDL_CTRL_REG3,
                             I2C_MEMADD_SIZE_8BIT, &ctrl, 1, 100);
           // CTRL_REG4: Z ultra-high performance
           ctrl = 0x0C;
           HAL_I2C_Mem_Write(&hi2c2, mag_i2c_addr, LIS3MDL_CTRL_REG4,
                             I2C_MEMADD_SIZE_8BIT, &ctrl, 1, 100);
           mag_ok = 1;
           printf("[IMU] LIS3MDL OK (WHO_AM_I=0x3D, addr=0x%02X) → 9-axis mode\r\n",
                  mag_i2c_addr);
       } else {
           printf("[IMU] LIS3MDL not found (0x%02X) → 6-axis fallback\r\n", whoAmI);
       }

       osMutexRelease(i2c2MutexHandle);
   }

   // 【優化點 1】：重新拉高 beta 權重至 0.15f
   // 既然你一定要 9 軸絕對精準不飄移，演算法就必須加大對磁力計的信任度，快速拉回航向角！
   AHRS_Init(0.15f, 50.0f);

   for(;;)
   {
     if (imu_ok) {
         // 【優化點 2】：將 Mutex 等待時間限制在 10ms，避免卡死其他任務
         if (osMutexAcquire(i2c2MutexHandle, 10) == osOK) {

             // 【優化點 3】：讀取超時從 100ms 降到 5ms（極速讀取）
             if (HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_ADDR, LSM6DSL_OUTX_L_G,
                                  I2C_MEMADD_SIZE_8BIT, raw_imu, 12, 15) != HAL_OK) {
                 osMutexRelease(i2c2MutexHandle);
                 osDelay(20);
                 continue;
             }

             // 【優化點 4】：分時分流！在讀取極慢的磁力計前，先短暫釋放 CPU 1 毫秒
             // 這樣可以讓出 I2C 總線給 ToF 任務插隊讀取，座標點掃描速度立刻就飆起來了！
             osMutexRelease(i2c2MutexHandle);
             osDelay(1);

             if (mag_ok) {
                 if (osMutexAcquire(i2c2MutexHandle, 10) == osOK) {
                     // 直接讀取 0x28 暫存器，超時設為 5ms
                     HAL_I2C_Mem_Read(&hi2c2, mag_i2c_addr, LIS3MDL_OUT_X_L | 0x80,
                                      I2C_MEMADD_SIZE_8BIT, raw_mag, 6, 15);
                     osMutexRelease(i2c2MutexHandle);
                 }
             }

             // ----- 所有的數學運算全部移到 Mutex 外面進行，完全不佔用 I2C 資源 -----
             global_imu_data.gx = (int16_t)((raw_imu[1]  << 8) | raw_imu[0]) - gyro_x_bias;
             global_imu_data.gy = (int16_t)((raw_imu[3]  << 8) | raw_imu[2]) - gyro_y_bias;
             global_imu_data.gz = (int16_t)((raw_imu[5]  << 8) | raw_imu[4]) - gyro_z_bias;
             global_imu_data.ax = (int16_t)((raw_imu[7]  << 8) | raw_imu[6]);
             global_imu_data.ay = (int16_t)((raw_imu[9]  << 8) | raw_imu[8]);
             global_imu_data.az = (int16_t)((raw_imu[11] << 8) | raw_imu[10]);

             if (mag_ok) {
                 mx_raw = (int16_t)((raw_mag[1] << 8) | raw_mag[0]);
                 my_raw = (int16_t)((raw_mag[3] << 8) | raw_mag[2]);
                 mz_raw = (int16_t)((raw_mag[5] << 8) | raw_mag[4]);

                 // 扣除硬磁 Offset
                 mx_cal = (float)(mx_raw - mag_x_offset);
                 my_cal = (float)(my_raw - mag_y_offset);
                 mz_cal = (float)(mz_raw - mag_z_offset);
             }

             // 轉換為物理單位
             float ax = (float)global_imu_data.ax / 16384.0f;
             float ay = (float)global_imu_data.ay / 16384.0f;
             float az = (float)global_imu_data.az / 16384.0f;

             float gx = (float)global_imu_data.gx * 70.0e-3f * (3.14159265f / 180.0f);
             float gy = (float)global_imu_data.gy * 70.0e-3f * (3.14159265f / 180.0f);
             float gz = (float)global_imu_data.gz * 70.0e-3f * (3.14159265f / 180.0f);

             // ----- 進行 9 軸感測器官方標準軸向對齊 -----
             AHRS_Update9(gx, gy, gz, ax, ay, az, my_cal, mx_cal, -mz_cal);

             // 輸出四元數存入全域變數
             AHRS_GetQuaternion(
                 (float *)&global_q0, (float *)&global_q1,
                 (float *)&global_q2, (float *)&global_q3
             );

#if DEBUG_SENSOR_RAW
             {
                 static uint8_t dbg_cnt = 0;
                 int16_t gx_raw = (int16_t)((raw_imu[1] << 8) | raw_imu[0]);
                 int16_t gy_raw = (int16_t)((raw_imu[3] << 8) | raw_imu[2]);
                 int16_t gz_raw = (int16_t)((raw_imu[5] << 8) | raw_imu[4]);
                 if (++dbg_cnt >= 5) {
                     dbg_cnt = 0;
                     printf("GYRO_RAW:%d,%d,%d\r\n", gx_raw, gy_raw, gz_raw);
                     if (mag_ok) {
                         printf("MAG_RAW:%d,%d,%d\r\n", mx_raw, my_raw, mz_raw);
                     }
                 }
             }
#endif
         }
     }
     osDelay(20); // 穩定 50Hz 更新率
   }
   /* USER CODE END StartIMUTask */
 }



/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
