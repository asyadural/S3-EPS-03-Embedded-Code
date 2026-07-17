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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
    uint8_t priority;
    uint8_t sender;
    uint8_t receiver;
    uint16_t message_id;
    uint8_t seq_type;
    uint8_t seq_count;
} METUCube_FDCAN_ID_t;

typedef struct{
  uint8_t priority;
  uint8_t sender;
  uint8_t receiver;
  uint16_t message_id;
  uint8_t seq_type;
  uint8_t seq_count;
  uint8_t dlc;
  uint8_t payload[8];
}FDCAN_queue_element;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BROADCAST_ID  0x0F
#define MPPT_ID       0x04
#define OBC_ID        0x00

#define MSG_ID_MPPT_HEARTBEAT 0x85
#define MSG_ID_MPPT_HOUSEKEEPING 0x86

#define FDCAN_QUEUE_SIZE 50

#define TELE_TBAT  0x00
#define TELE_POUT  0x02
#define TELE_PIN   0x04
#define TELE_IOUT  0x08
#define TELE_VBAT  0x0C
#define TELE_VIN   0x0E
#define LT8491_CH1_ADDR  (0x10 << 1)
#define LT8491_CH2_ADDR  (0x29 << 1)
#define LT8491_CH3_ADDR  (0x19 << 1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint32_t errorCounter = 0;
uint32_t canRecCounter = 0;
FDCAN_TxHeaderTypeDef txHeader;
FDCAN_RxHeaderTypeDef rxHeader;
uint8_t txData[8];
uint8_t rxData[8];

uint32_t last_heartbeat_time = 0;
uint32_t last_OBC_heartbeat_time = 0;
const uint32_t heartbeat_interval = 1000;
const uint32_t OBC_heartbeat_reset_time = 10000;

volatile uint8_t queue_head = 0;
volatile uint8_t queue_tail = 0;
volatile uint16_t queue_overflow_counter = 0;

FDCAN_queue_element fdcan_queue[FDCAN_QUEUE_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
uint32_t Build_FDCAN_ID_FromStruct(METUCube_FDCAN_ID_t *id);

HAL_StatusTypeDef MPPT_Send_Heartbeat(void);
HAL_StatusTypeDef MPPT_Send_Housekeeping(void);

uint8_t FDCAN_QueuePush(FDCAN_queue_element *queue, volatile uint8_t *head, volatile uint8_t *tail, FDCAN_queue_element *element);
uint8_t FDCAN_QueuePop(FDCAN_queue_element *queue, volatile uint8_t *head, volatile uint8_t *tail, FDCAN_queue_element *element);
HAL_StatusTypeDef FDCAN_ProcessQueue(void);
uint8_t FDCAN_DLC_ToBytes(uint32_t dataLength);
HAL_StatusTypeDef FDCAN_ProcessQueueElement(FDCAN_queue_element *element);
uint32_t FDCAN_Bytes_To_DLC(uint8_t bytes);
HAL_StatusTypeDef LT8491_ReadWord(uint16_t devAddr, uint8_t reg, uint16_t *value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t Build_FDCAN_ID_FromStruct(METUCube_FDCAN_ID_t *id) {
    uint32_t ext_id = 0;
    ext_id |= ((id->priority   & 0x03) << 27);
    ext_id |= ((id->sender     & 0x0F) << 23);
    ext_id |= ((id->receiver   & 0x0F) << 19);
    ext_id |= ((id->message_id & 0x3FF) << 9);
    ext_id |= ((id->seq_type   & 0x03) << 7);
    ext_id |= ((id->seq_count  & 0x7F));

    return ext_id;
}

HAL_StatusTypeDef MPPT_Send_Heartbeat(void){
	METUCube_FDCAN_ID_t fdcan_id;
	fdcan_id.priority = 0x03;
	fdcan_id.sender = MPPT_ID;
	fdcan_id.receiver = OBC_ID;
	fdcan_id.message_id = MSG_ID_MPPT_HEARTBEAT;
	fdcan_id.seq_type = 0x03;
	fdcan_id.seq_count = 0;

	txHeader.Identifier = Build_FDCAN_ID_FromStruct(&fdcan_id);
	txHeader.IdType = FDCAN_EXTENDED_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_1;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader.MessageMarker = 0;

	txData[0] = MSG_ID_MPPT_HEARTBEAT;

	return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);
}

uint8_t FDCAN_QueuePush(FDCAN_queue_element *queue, volatile uint8_t *head, volatile uint8_t *tail, FDCAN_queue_element *element) {
  uint8_t next = (*head + 1) % FDCAN_QUEUE_SIZE;
  if (next == *tail) return 0;   // queue full
  queue[*head] = *element;
  *head = next;
  return 1;
}

uint8_t FDCAN_QueuePop(FDCAN_queue_element *queue, volatile uint8_t *head, volatile uint8_t *tail, FDCAN_queue_element *element){
  __disable_irq();
  if (*tail == *head) {
    __enable_irq();
    return 0;   // empty
  }
  *element = queue[*tail];
  *tail = (*tail + 1) % FDCAN_QUEUE_SIZE;
  __enable_irq();
  return 1;
}

HAL_StatusTypeDef FDCAN_ProcessQueue(void){
  FDCAN_queue_element element;
  if(FDCAN_QueuePop(fdcan_queue, &queue_head, &queue_tail, &element)){
    return FDCAN_ProcessQueueElement(&element);
  }
  return HAL_OK;
}

uint8_t FDCAN_DLC_ToBytes(uint32_t dataLength){
    switch (dataLength){
    	case FDCAN_DLC_BYTES_0: return 0;
        case FDCAN_DLC_BYTES_1: return 1;
        case FDCAN_DLC_BYTES_2: return 2;
        case FDCAN_DLC_BYTES_3: return 3;
        case FDCAN_DLC_BYTES_4: return 4;
        case FDCAN_DLC_BYTES_5: return 5;
        case FDCAN_DLC_BYTES_6: return 6;
        case FDCAN_DLC_BYTES_7: return 7;
        case FDCAN_DLC_BYTES_8: return 8;
        default:
        	return 0;
    }
}

uint32_t FDCAN_Bytes_To_DLC(uint8_t bytes){
    switch(bytes){
        case 0: return FDCAN_DLC_BYTES_0;
        case 1: return FDCAN_DLC_BYTES_1;
        case 2: return FDCAN_DLC_BYTES_2;
        case 3: return FDCAN_DLC_BYTES_3;
        case 4: return FDCAN_DLC_BYTES_4;
        case 5: return FDCAN_DLC_BYTES_5;
        case 6: return FDCAN_DLC_BYTES_6;
        case 7: return FDCAN_DLC_BYTES_7;
        default: return FDCAN_DLC_BYTES_8;
    }
}

HAL_StatusTypeDef FDCAN_ProcessQueueElement(FDCAN_queue_element *element){
    switch(element->message_id){
    	case MSG_ID_MPPT_HOUSEKEEPING: return MPPT_Send_Housekeeping();
        default:
            break;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef LT8491_ReadWord(uint16_t devAddr, uint8_t reg, uint16_t *value){
    uint8_t data[2];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, devAddr, reg, I2C_MEMADD_SIZE_8BIT, data, 2, 100);

    if (status != HAL_OK) return status;

    *value = (uint16_t)(data[0] | (data[1] << 8));   //little endian, yanlışsa düzeltiriz
    return HAL_OK;
}
HAL_StatusTypeDef MPPT_Send_Housekeeping(void){
    uint16_t payload[18] = {0};

    uint16_t ch1_pout, ch1_pin, ch1_iout, ch1_vbat, ch1_vin, ch1_tbat;
    uint16_t ch2_pout, ch2_pin, ch2_iout, ch2_vin, ch2_tbat;
    uint16_t ch3_pout, ch3_pin, ch3_iout, ch3_vin, ch3_tbat;

    if (LT8491_ReadWord(LT8491_CH1_ADDR, TELE_POUT, &ch1_pout) != HAL_OK) return HAL_ERROR;
    if (LT8491_ReadWord(LT8491_CH1_ADDR, TELE_PIN,  &ch1_pin)  != HAL_OK) return HAL_ERROR;
    if (LT8491_ReadWord(LT8491_CH1_ADDR, TELE_IOUT, &ch1_iout) != HAL_OK) return HAL_ERROR;
    if (LT8491_ReadWord(LT8491_CH1_ADDR, TELE_VBAT, &ch1_vbat) != HAL_OK) return HAL_ERROR;
    if (LT8491_ReadWord(LT8491_CH1_ADDR, TELE_VIN,  &ch1_vin)  != HAL_OK) return HAL_ERROR;
    if (LT8491_ReadWord(LT8491_CH1_ADDR, TELE_TBAT, &ch1_tbat) != HAL_OK) return HAL_ERROR;

    if (LT8491_ReadWord(LT8491_CH2_ADDR, TELE_POUT, &ch2_pout) != HAL_OK) return HAL_ERROR;
    if (LT8491_ReadWord(LT8491_CH2_ADDR, TELE_PIN,  &ch2_pin)  != HAL_OK) return HAL_ERROR;
    if (LT8491_ReadWord(LT8491_CH2_ADDR, TELE_IOUT, &ch2_iout) != HAL_OK) return HAL_ERROR;
    if (LT8491_ReadWord(LT8491_CH2_ADDR, TELE_VIN,  &ch2_vin)  != HAL_OK) return HAL_ERROR;
    if (LT8491_ReadWord(LT8491_CH2_ADDR, TELE_TBAT, &ch2_tbat) != HAL_OK) return HAL_ERROR;

    if (LT8491_ReadWord(LT8491_CH3_ADDR, TELE_POUT, &ch3_pout) != HAL_OK) return HAL_ERROR;
    if (LT8491_ReadWord(LT8491_CH3_ADDR, TELE_PIN,  &ch3_pin)  != HAL_OK) return HAL_ERROR;
    if (LT8491_ReadWord(LT8491_CH3_ADDR, TELE_IOUT, &ch3_iout) != HAL_OK) return HAL_ERROR;
    if (LT8491_ReadWord(LT8491_CH3_ADDR, TELE_VIN,  &ch3_vin)  != HAL_OK) return HAL_ERROR;
    if (LT8491_ReadWord(LT8491_CH3_ADDR, TELE_TBAT, &ch3_tbat) != HAL_OK) return HAL_ERROR;

    payload[0]  = ch1_vbat;                         //battery voltage
    payload[1]  = ch1_iout + ch2_iout + ch3_iout;   // battery current
    payload[2]  = ch1_pout + ch2_pout + ch3_pout;   // battery charge
    payload[3]  = ch1_pout;  // p. out
    payload[4]  = ch1_pin;   // p. draw
    payload[5]  = ch1_vin;   // p. voltage
    payload[6]  = 0; // palceholder

    payload[7]  = ch2_pout;
    payload[8]  = ch2_pin;
    payload[9]  = ch2_vin;
    payload[10] = 0;

    payload[11] = ch3_pout;
    payload[12] = ch3_pin;
    payload[13] = ch3_vin;
    payload[14] = 0;

    payload[15] = ch1_tbat; // mppt board temp için ch1 kullandım
    payload[16] = 0;
    payload[17] = 0;

    METUCube_FDCAN_ID_t fdcan_id;
    fdcan_id.priority = 0x03;
    fdcan_id.sender = MPPT_ID;
    fdcan_id.receiver = OBC_ID;
    fdcan_id.message_id = MSG_ID_MPPT_HOUSEKEEPING;

    uint8_t *bytes = (uint8_t *)payload;
    uint8_t total_bytes = 36;

    for (uint8_t segment = 0; segment < 5; segment++){
        uint8_t remaining = total_bytes - segment * 8;
        uint8_t dlc = (remaining >= 8) ? 8 : remaining;

        fdcan_id.seq_count = segment;

        if (segment == 0)
            fdcan_id.seq_type = 0x01;
        else if (segment == 4)
            fdcan_id.seq_type = 0x02;
        else
            fdcan_id.seq_type = 0x00;

        txHeader.Identifier = Build_FDCAN_ID_FromStruct(&fdcan_id);
        txHeader.IdType = FDCAN_EXTENDED_ID;
        txHeader.TxFrameType = FDCAN_DATA_FRAME;
        txHeader.DataLength = FDCAN_Bytes_To_DLC(dlc);
        txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        txHeader.BitRateSwitch = FDCAN_BRS_OFF;
        txHeader.FDFormat = FDCAN_CLASSIC_CAN;
        txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        txHeader.MessageMarker = 0;

        for (uint8_t i = 0; i < dlc; i++) {
            txData[i] = bytes[segment * 8 + i];
        }

        HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);

        if (status != HAL_OK) return status;

        HAL_Delay(2);
    }

    return HAL_OK;
}
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
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK){
      errorCounter++;
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK){
      errorCounter++;
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
	  errorCounter++;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(HAL_GetTick() - last_heartbeat_time >= heartbeat_interval) {
		  last_heartbeat_time = HAL_GetTick();
	  	  if(MPPT_Send_Heartbeat() != HAL_OK) {
	  		  errorCounter++;
	  	  }
	  }

	  if(FDCAN_ProcessQueue() != HAL_OK) errorCounter++;

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV2;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 25;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 1;
  RCC_OscInitStruct.PLL.PLLR = 4;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 10;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00807EBE;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hi2c2.Init.Timing = 0x00807EBE;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi1, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : WDI_Pin */
  GPIO_InitStruct.Pin = WDI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WDI_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {

	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK){
    	errorCounter++;
        return;
    }

    canRecCounter++;

	if (rxHeader.IdType != FDCAN_EXTENDED_ID) return;

	FDCAN_queue_element element;
	element.priority = (rxHeader.Identifier >> 27) & 0x03;
	element.sender = (rxHeader.Identifier >> 23) & 0x0F;
	element.receiver   = (rxHeader.Identifier >> 19) & 0x0F;
	element.message_id = (rxHeader.Identifier >> 9)  & 0x3FF;
	element.seq_type   = (rxHeader.Identifier >> 7)  & 0x03;
	element.seq_count  = rxHeader.Identifier & 0x7F;
	element.dlc = FDCAN_DLC_ToBytes(rxHeader.DataLength);

	if(element.receiver != MPPT_ID && element.receiver != BROADCAST_ID) return;
	for (uint8_t i = 0; i < element.dlc; i++) {
	      element.payload[i] = rxData[i];
	}

	if(!FDCAN_QueuePush(fdcan_queue, &queue_head, &queue_tail, &element)){
	    queue_overflow_counter++;
	}
}
/* USER CODE END 4 */

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
