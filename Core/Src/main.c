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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 定點運算：將 mm 轉為 0.1mm (deci-millimeter) */
#define MM_TO_DMM(x) ((uint32_t)(x) * 10)
#define FILTER_SIZE 1  // <--- 將這裡改成 1，關閉平滑濾波，保留真實的邊緣跳變
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
  .stack_size = 128 * 4,
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
volatile uint32_t idle_tick_count = 0;

// === 新增：I2C2 Mutex 宣告 ===
osMutexId_t i2c2MutexHandle;
const osMutexAttr_t i2c2Mutex_attributes = {
  .name = "i2c2Mutex"
};

// === 新增：LSM6DSL I2C 定義 (為下一步準備) ===
#define LSM6DSL_ADDR         (0x6A << 1) // 8-bit Address: 0xD4
#define LSM6DSL_WHO_AM_I     0x0F
#define LSM6DSL_CTRL1_XL     0x10  // 加速度計控制
#define LSM6DSL_CTRL2_G      0x11  // 陀螺儀控制
#define LSM6DSL_OUTX_L_G     0x22  // 陀螺儀數據起始位址
#define LSM6DSL_OUTX_L_XL    0x28  // 加速度計數據起始位址

// === 新增：IMU 資料結構與全域變數 ===
typedef struct {
    int16_t ax, ay, az; // 加速度
    int16_t gx, gy, gz; // 陀螺儀
} IMU_Data_t;

volatile IMU_Data_t global_imu_data = {0};

// === 新增：姿態角全域變數 ===
volatile float global_roll = 0.0f;
volatile float global_pitch = 0.0f;

// === 新增：掃描狀態旗標 ===
volatile uint8_t is_scanning = 0; // 0: 待機 (Standby), 1: 掃描中 (Scanning)
// === 新增：BLE 點雲傳輸 Handles ===
uint16_t ScannerServHandle;
uint16_t PointCharHandle;
// === 新增：BLE 執行緒安全傳輸旗標 ===
volatile uint8_t ble_send_ready = 0;
volatile float ble_px = 0, ble_py = 0, ble_pz = 0;
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
  // === 新增：喚醒被 CubeMX 綁架的藍色按鍵與 LED ===
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI); // 初始化按鍵與中斷
  BSP_LED_Init(LED2);                         // 初始化 PB14 的 LED
  // ===============================================

  Add_Scanner_Service();
  // === 新增：叫藍牙晶片開始對外大聲廣播 ===
  Set_DeviceConnectable();
  printf("[BLE] 廣播程序已啟動！等待連線...\r\n");
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
  // 1. 直接在硬體中斷切換狀態，不等待任何感測器！(最暴力的直達車)
  if (GPIO_Pin == GPIO_PIN_13)
  {
    is_scanning = !is_scanning;
  }

  // 2. 處理藍牙晶片 (PE6) 觸發的底層事件
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
  char err_msg[64];

  /* Infinite loop */
  for(;;)
  {
    // === 新增：安全的 BLE 傳輸區塊 ===
    if (ble_send_ready) {
        float point_data[3] = {ble_px, ble_py, ble_pz};
        // 只有這個 Task 可以存取 SPI，絕對不會撞車！
        tBleStatus ret = aci_gatt_update_char_value(ScannerServHandle, PointCharHandle, 0, 12, (uint8_t*)point_data);

        // 加入除錯機制：如果發送失敗，印出錯誤碼
        if (ret != BLE_STATUS_SUCCESS) {
            int err_len = snprintf(err_msg, sizeof(err_msg), "[BLE Error] Code: 0x%02X\r\n", ret);
            HAL_UART_Transmit(&huart1, (uint8_t*)err_msg, err_len, HAL_MAX_DELAY);
        }
        ble_send_ready = 0; // 資料送出，放下旗標
    }

    // 讓藍牙協定疊在背景持續運作
    MX_BlueNRG_MS_Process();
    osDelay(10); // 稍微休息，避免佔用全部 CPU
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
  // 🌟 貶為賤民：把不穩定的感測器降到最低權限
  osThreadSetPriority(ToF_Sensor_TaskHandle, osPriorityLow);

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
  // 🌟 破除飢餓：將按鈕與發送任務提升到「最高優先權」，保證絕對不被卡死！
  osThreadSetPriority(Telemetry_TaskHandle, osPriorityRealtime);

  char uart_buf[128];
  uint8_t last_state = 0;

  for(;;)
  {
    // --- 1. 即時狀態回饋 ---
    if (is_scanning != last_state) {
        if (is_scanning) {
            printf("\r\n>>> 掃描開始 (綠燈) <<<\r\n");
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        } else {
            printf("\r\n>>> 掃描停止 (紅燈) <<<\r\n");
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        }
        last_state = is_scanning;
    }

    // --- 2. 資料打包發送 ---
    if (is_scanning) {
        float r = (float)global_display_dmm / 10.0f;
        float pitch_rad = global_pitch * (3.14159265f / 180.0f);
        float roll_rad  = global_roll  * (3.14159265f / 180.0f);

        float px = r * sinf(pitch_rad) * cosf(roll_rad);
        float py = -r * sinf(roll_rad);
        float pz = r * cosf(pitch_rad) * cosf(roll_rad);

        int len = snprintf(uart_buf, sizeof(uart_buf), "%.1f,%.1f,%.1f\r\n", px, py, pz);
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, len, 100);

        ble_px = px; ble_py = py; ble_pz = pz;
        ble_send_ready = 1;
    }
    osDelay(100);
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
  uint8_t whoAmI = 0;
  uint8_t ctrl[2];
  uint8_t raw_data[12];

  osDelay(100);
  // 🌟 破除死結：不無限死等，最多等 100ms
  if (osMutexAcquire(i2c2MutexHandle, 100) == osOK) {
      HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_ADDR, LSM6DSL_WHO_AM_I, 1, &whoAmI, 1, 100);
      if (whoAmI == 0x6A) {
          ctrl[0] = 0x40;
          HAL_I2C_Mem_Write(&hi2c2, LSM6DSL_ADDR, LSM6DSL_CTRL1_XL, 1, &ctrl[0], 1, 100);
          ctrl[1] = 0x4C;
          HAL_I2C_Mem_Write(&hi2c2, LSM6DSL_ADDR, LSM6DSL_CTRL2_G, 1, &ctrl[1], 1, 100);
      }
      osMutexRelease(i2c2MutexHandle);
  }

  for(;;)
  {
    if (whoAmI == 0x6A) {
        if (osMutexAcquire(i2c2MutexHandle, 50) == osOK) {
            HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_ADDR, LSM6DSL_OUTX_L_G, 1, raw_data, 12, 100);
            osMutexRelease(i2c2MutexHandle);

            global_imu_data.gx = (int16_t)((raw_data[1] << 8) | raw_data[0]);
            global_imu_data.gy = (int16_t)((raw_data[3] << 8) | raw_data[2]);
            global_imu_data.gz = (int16_t)((raw_data[5] << 8) | raw_data[4]);
            global_imu_data.ax = (int16_t)((raw_data[7] << 8) | raw_data[6]);
            global_imu_data.ay = (int16_t)((raw_data[9] << 8) | raw_data[8]);
            global_imu_data.az = (int16_t)((raw_data[11] << 8) | raw_data[10]);

            float ax_g = (float)global_imu_data.ax / 16384.0f;
            float ay_g = (float)global_imu_data.ay / 16384.0f;
            float az_g = (float)global_imu_data.az / 16384.0f;

            global_roll = atan2f(ay_g, az_g) * (180.0f / 3.14159265f);
            global_pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * (180.0f / 3.14159265f);
        }
    }
    osDelay(20);
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
