/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.hpp"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "mbed.h"
#include "EthernetInterface.h"
#include "UDPSocket.h"
#include "Callback.h"
#include "SocketAddress.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_it.h"
#include "nRF24L01P.h"
#include <Semaphore.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BLOCK_SIZE 256

#define BLOCKS_NO_RX 2
#define MAX_WAVE_LEN_RX (BLOCK_SIZE * BLOCKS_NO_RX)

#define MAX_WAVE_LEN_TX ( BLOCK_SIZE * 1024)

#define RF_TRANSFER_SIZE (8)
#define MAX_PKT_LEN_RF RF_TRANSFER_SIZE

#define DATA_PORT 6666
#define CMD_PORT 7777

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

Serial pc(USBTX, USBRX);

char address[14];
uint8_t modifiableNodeAddr = 22;
InterruptIn addrButton(USER_BUTTON);
Timeout selectionTimeout;
bool addressSelected = false;

EthernetInterface eth;
UDPSocket sockData;
UDPSocket sockCmd;
SocketAddress pcCmdAddr, pcDataAddr;

const uint8_t MODE_IDLE = 0;
const uint8_t MODE_TX = 1;
const uint8_t MODE_RX = 2;
const uint8_t MODE_RF = 3;

uint8_t nodeMode = MODE_IDLE;
bool switchedMode = true;

uint32_t lastCmd[2];
const uint8_t IDLE_CMD = 0;
const uint8_t RECEIVING_CMD = 1;
const uint8_t REPLYING_CMD = 2;
const uint8_t BLOCKING_CMD = 3;
const uint8_t ERROR_CMD = 4;
uint8_t cmdState = IDLE_CMD;
Callback<void(void)> sockCmdCb;
bool commandPending = false;

// Reception part

const uint8_t RX_IDLE = 0;
const uint8_t RX_SENDING = 1;

uint8_t rxStatus = RX_IDLE;

bool lowerData = 0;
bool upperData = 0;

uint8_t adcDataL[MAX_WAVE_LEN_RX / 2];
uint8_t adcDataU[MAX_WAVE_LEN_RX / 2];

// Transmission part

uint32_t waveLen = 0;
uint8_t dacData[MAX_WAVE_LEN_TX];

nsapi_size_t receivedData = 0;

const uint8_t TX_IDLE = 0;
const uint8_t TX_BUFFERING = 1;
const uint8_t TX_TRANSMITTING = 2;
const uint8_t TX_TRANSMISSION_DONE = 3;

uint8_t txStatus = TX_IDLE;
uint32_t packet_no = 0;

// RF part

nRF24L01P my_nrf24l01p(PE_6, PE_5, PE_2, PE_3, PE_4, PD_3);    // mosi, miso, sck, csn, ce, irq
uint32_t RFpktLenTx = MAX_PKT_LEN_RF;
uint8_t RFpktDataTx[MAX_PKT_LEN_RF];

uint32_t RFpktLenRx = MAX_PKT_LEN_RF;
uint8_t RFpktDataRx[MAX_PKT_LEN_RF];

uint32_t RFpacket_no = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_UART4_Init(void);

#ifdef __cplusplus
}
#endif

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __cplusplus
extern "C" {
#endif

void startTx(void) {
    txStatus = TX_TRANSMITTING;
    HAL_StatusTypeDef r = HAL_DAC_Start_DMA( &hdac, DAC_CHANNEL_1, (uint32_t *) dacData, waveLen, DAC_ALIGN_8B_R);

    __HAL_TIM_SET_COUNTER( &htim6, 0);
    HAL_TIM_Base_Start( &htim6 );
}

void stopTx(void) {
    txStatus = TX_IDLE;
    HAL_TIM_Base_Stop( &htim6 );
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
}

void startRx(void) {
    rxStatus = RX_IDLE;
    lowerData = false;
    upperData = false;
    __HAL_TIM_SET_COUNTER( &htim4, 0);
    HAL_TIM_Base_Start(&htim4);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adcDataL, (uint32_t) MAX_WAVE_LEN_RX );
}

void stopRx(void) {
    rxStatus = RX_IDLE;
    HAL_TIM_Base_Stop( &htim4 );
    HAL_ADC_Stop_DMA(&hadc1);
}

#ifdef __cplusplus
}
#endif

void ethConnect() {
    int r;

    eth.set_network(address, "255.255.255.0", "192.168.1.1");
    r = eth.connect();

    if (r != 0) {
        pc.printf("Error connecting to ethernet!\n");
        exit(r);
    }

    pc.printf("Success: connected to ethernet!\n");
}

void sockConnect() {
    nsapi_error_t r;

    sockData.open(&eth);
    r = sockData.bind(DATA_PORT);

    if (r != 0) {
        pc.printf("Error: can't open data socket!\n");
        exit(r);
    }

    pc.printf("Success: opened data socket!\n");


    sockCmd.open(&eth);
    r = sockCmd.bind(CMD_PORT);

    if (r != 0) {
        pc.printf("Error: can't open cmd socket!\n");
        exit(r);
    }

    sockCmd.set_blocking(false);
    sockCmd.sigio(sockCmdCb);

    pc.printf("Success: opened cmd socket!\n");
}

void RFInit() {
    my_nrf24l01p.powerUp();

    my_nrf24l01p.setReceiveMode();
    my_nrf24l01p.setTransferSize( RF_TRANSFER_SIZE );
    my_nrf24l01p.setRfOutputPower(NRF24L01P_TX_PWR_MINUS_18_DB);
    my_nrf24l01p.enable();

    // Display the (default) setup of the nRF24L01+ chip
    pc.printf( "nRF24L01+ Frequency    : %d MHz\r\n",  my_nrf24l01p.getRfFrequency() );
    pc.printf( "nRF24L01+ Output power : %d dBm\r\n",  my_nrf24l01p.getRfOutputPower() );
    pc.printf( "nRF24L01+ Data Rate    : %d kbps\r\n", my_nrf24l01p.getAirDataRate() );
    pc.printf( "nRF24L01+ TX Address   : 0x%010llX\r\n", my_nrf24l01p.getTxAddress() );
    pc.printf( "nRF24L01+ RX Address   : 0x%010llX\r\n", my_nrf24l01p.getRxAddress() );



    pc.printf( "(transfers are grouped into %d characters)\r\n", my_nrf24l01p.getTransferSize() );
}

void cmdSocketBeginBlockingOp() {
    sockCmd.set_blocking(true);
    sockCmd.sigio(nullptr);
}

void cmdSocketEndBlockingOp() {
    sockCmd.set_blocking(false);
    sockCmd.sigio(sockCmdCb);
}

nsapi_size_or_error_t setMode( uint8_t mode ) {

    switch (nodeMode) {
    case MODE_IDLE:
        break;
    case MODE_TX:
        stopTx();
        break;
    case MODE_RX:
        stopRx();
        break;
    case MODE_RF:
        break;
    }

    nodeMode = mode;

    switch (mode) {
    case MODE_IDLE:
        break;
    case MODE_TX:
        break;
    case MODE_RX:
        startRx();
        break;
    case MODE_RF:
        break;
    }

    switchedMode = true;

    static uint32_t response[2] = {0, 0};
    return sockCmd.sendto(pcCmdAddr.get_ip_address(), pcCmdAddr.get_port(), response, sizeof(response));
}

void receivePacketData() {
    cmdSocketBeginBlockingOp();

    nsapi_size_or_error_t size = sockData.recvfrom(&pcDataAddr, (void*) (dacData + receivedData), BLOCK_SIZE);

    if (size < 0) {
        pc.printf("recvfrom failed with error code %d\n", size);
    } else {
        receivedData += size;
        //printf("Received %d data\n", receivedData);

        uint32_t response[2] = {0, 0};

        // Signal the client that we're ready to receive the wave
        nsapi_size_or_error_t size = sockCmd.sendto(pcCmdAddr.get_ip_address(), pcCmdAddr.get_port(), response, sizeof(response));

        if ( receivedData == waveLen ) {
            pc.printf("Received all the wave for packet %d \n", packet_no++);
            txStatus = TX_IDLE;

            // for(uint32_t i = 0; i < waveLen;i++){
            //     pc.printf("%d ", dacData[i]);
            // }

            // pc.printf("\n");
        }
    }

    cmdSocketEndBlockingOp();

}

/**
* Receives and executes cmds from the controller. A command is made up of 2 uint32. The first
* specfies the cmd, the second the data.
*
* List of commands:
*
* 0 : set mode
*   0 : idle
*   1 : tx
*   2 : rx
* 0 : set mode reply
*   0 : ok
*   1 : err
*
*
* 1 : buffer wave
*   val : length of the wave in bytes
* 1 : transmit wave reply at each block
*   0   : on buffer_done
*   val : received bytes
*
*
* 2 : transmit rf packet
*   val : length of the packet
* 2 : transmit rf reply
*   0 : ok and on tx_done
*   1 : err
*
*
* 3 : get last rf packet
*   val : don't care
* 3 : get last rf packet reply
*   val : len of pkt
*
*
* 4 : transmit last buffered wave
*   val : don't care
*   
*   
*/

nsapi_size_or_error_t receiveCmd() {
    return sockCmd.recvfrom(&pcCmdAddr, lastCmd, sizeof(lastCmd));
}

nsapi_size_or_error_t executeCmd() {
    pc.printf("Received command from %s value: %d %d!\n", pcCmdAddr.get_ip_address(), lastCmd[0], lastCmd[1]);

    uint32_t op = lastCmd[0];
    uint32_t val = lastCmd[1];

    nsapi_size_or_error_t size;

    switch (op) {
    case 0: {
        size = setMode((uint8_t) val);
        break;
    }
    case 1: {
        receivedData = 0;
        waveLen = val;
        txStatus = TX_BUFFERING;

        memset(dacData, 0, waveLen);
        uint32_t response[2] = {1, 0};

        pc.printf("Preparing to receive data!\n");

        // Signal the client that we're ready to receive the wave
        size = sockCmd.sendto(pcCmdAddr.get_ip_address(), pcCmdAddr.get_port(), response, sizeof(response));

        break;
    }
    case 2: {
        pc.printf("Sending RF packet number %u \n", RFpacket_no++);

        RFpktLenTx = val;
        memset(RFpktDataTx, 0, RFpktLenTx);

        uint32_t response[2] = {2, 0};

        // Signal the client that we're ready to receive the pkt
        size = sockCmd.sendto(pcCmdAddr.get_ip_address(), pcCmdAddr.get_port(), response, sizeof(response));

        nsapi_size_or_error_t size = sockData.recvfrom(&pcDataAddr, (void*) RFpktDataTx, RFpktLenTx);

        my_nrf24l01p.write( NRF24L01P_PIPE_P0, (char*) RFpktDataTx, RFpktLenTx );

        // Signal the client that we're done transmitting the pkt
        size = sockCmd.sendto(pcCmdAddr.get_ip_address(), pcCmdAddr.get_port(), response, sizeof(response));

        break;
    }
    case 3: {
        uint32_t response[2];
        response[0] = 2;

        if ( my_nrf24l01p.readable() ) {
            pc.printf("RF data received! \n");
            memset(RFpktDataRx, 0, RF_TRANSFER_SIZE);

            RFpktLenRx = my_nrf24l01p.read( NRF24L01P_PIPE_P0, (char*) RFpktDataRx, sizeof( RFpktDataRx ) );

            response[1] = RFpktLenRx;

            // Signal the client that there's data
            size = sockCmd.sendto(pcCmdAddr.get_ip_address(), pcCmdAddr.get_port(), response, sizeof(response));

            nsapi_size_or_error_t size = sockData.sendto(pcDataAddr.get_ip_address(), pcDataAddr.get_port(), (void*) RFpktDataRx, RFpktLenRx);


        }else{
            pc.printf("No RF data available! \n");
            response[1] = 0;

            // Signal the client that there's no data
            size = sockCmd.sendto(pcCmdAddr.get_ip_address(), pcCmdAddr.get_port(), response, sizeof(response));
        }

        break;
    }
    case 4: {
        uint32_t response[2];
        response[0] = 4;

        if ( waveLen > 0 ) {
            pc.printf("Transmitting last buffered wave!\n");
            response[1] = 0;

            // Signal the client that there's data
            size = sockCmd.sendto(pcCmdAddr.get_ip_address(), pcCmdAddr.get_port(), response, sizeof(response));

            startTx();

        }else{
            pc.printf("No wave available!\n");
            response[1] = 1;

            // Signal the client that there's no data
            size = sockCmd.sendto(pcCmdAddr.get_ip_address(), pcCmdAddr.get_port(), response, sizeof(response));
        }

        break;
    }
    default:
        break;

    }

    return size;
}

void handleCmdSocketSigio() {
    nsapi_size_or_error_t size;

    size = receiveCmd();

    switch (size) {
    case NSAPI_ERROR_WOULD_BLOCK:
        break;
    default:
        commandPending = true;
        break;
    }

}


void rxLoop() {
    static bool lastSent = false;

    switch (rxStatus) {
    case RX_IDLE: {
        uint8_t _byte;
        nsapi_size_or_error_t size = sockData.recvfrom(&pcDataAddr, &_byte, 1);
        pc.printf("Received start from %s\n", pcDataAddr.get_ip_address());
        rxStatus = RX_SENDING;
    }
    case RX_SENDING:
        if (pcDataAddr) {

            if (lowerData && !lastSent ) {
                lastSent = true;
                lowerData = false;
                nsapi_size_or_error_t size = sockData.sendto(pcDataAddr.get_ip_address(), pcDataAddr.get_port(), (void*) &adcDataL[0], MAX_WAVE_LEN_RX / 2 );

                if ( size < 0 ) {
                    Error_Handler();
                }

            }

            if (upperData && lastSent) {
                lastSent = false;
                upperData = false;
                nsapi_size_or_error_t size = sockData.sendto(pcDataAddr.get_ip_address(), pcDataAddr.get_port(), (void*) &adcDataU[0], MAX_WAVE_LEN_RX / 2 );

                if ( size < 0 ) {
                    Error_Handler();
                }

            }
        }

    }

}


void txLoop() {
    switch (txStatus) {

    case TX_IDLE:
        break;

    case TX_BUFFERING:
        receivePacketData();
        break;

    case TX_TRANSMITTING:
        break;

    case TX_TRANSMISSION_DONE:
        stopTx();
        pc.printf("Transmission done!\n");

        uint32_t response[2] = {1, 0};
        nsapi_size_or_error_t size = sockCmd.sendto(pcCmdAddr.get_ip_address(), pcCmdAddr.get_port(), response, sizeof(response));

        if (size < 0) {
            pc.printf("Error communicating transmission complete, code %d\n", size);
            break;
        }

        txStatus = TX_IDLE;
        break;
    }
}

void mainLoop() {

    static const char* modeNames[4] = {"Idle", "Tx", "Rx", "RF"};

    if ( switchedMode ) {
        pc.printf("Switched mode to %s\n", modeNames[nodeMode]);
        switchedMode = false;
    }

    switch (nodeMode) {
    case MODE_IDLE: {
        break;
    }

    case MODE_TX: {
        txLoop();
        break;
    }

    case MODE_RX:{
        rxLoop();
        break;
    }

    case MODE_RF:{
        break;
    }
    }


    if ( commandPending) {
        cmdSocketBeginBlockingOp();
        executeCmd();
        commandPending = false;
        cmdSocketEndBlockingOp();
    }
}

void addressSelectionTimeout(){
    addressSelected = true;
}

void incrementAddress(){
    modifiableNodeAddr += 1;
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

    pc.baud(2000000);
    pc.printf("TRX pkt Software version: %s,%s\n", __DATE__, __TIME__);

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_DAC_Init();
    MX_TIM4_Init();
    MX_TIM6_Init();
    MX_UART4_Init();
    /* USER CODE BEGIN 2 */

    // Hack to give the node an address

    pc.printf("Starting Timeout\n");
    addrButton.rise(&incrementAddress);
    selectionTimeout.attach(&addressSelectionTimeout, 5.0);
    while(!addressSelected){
        wait(0.2);
    }
    selectionTimeout.detach();
    pc.printf("Selection Time Ended current address\n");

    sprintf(address, "192.168.1.%d", modifiableNodeAddr);
    pc.printf("Board ip: %s\n", address);

    sockCmdCb = Callback<void()>(handleCmdSocketSigio);


    ethConnect();
    sockConnect();
    RFInit();

    handleCmdSocketSigio();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        mainLoop();
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /** Configure LSE Drive Capability
    */
    HAL_PWR_EnableBkUpAccess();
    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 216;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Activate the Over-Drive mode
    */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4;
    PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */
    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_8B;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T4_TRGO;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

    /* USER CODE BEGIN DAC_Init 0 */

    /* USER CODE END DAC_Init 0 */

    DAC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN DAC_Init 1 */

    /* USER CODE END DAC_Init 1 */
    /** DAC Initialization
    */
    hdac.Instance = DAC;
    if (HAL_DAC_Init(&hdac) != HAL_OK)
    {
        Error_Handler();
    }
    /** DAC channel OUT1 config
    */
    sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN DAC_Init 2 */

    /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 2;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 30;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

    /* USER CODE BEGIN TIM6_Init 0 */

    /* USER CODE END TIM6_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM6_Init 1 */

    /* USER CODE END TIM6_Init 1 */
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 8;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 128;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM6_Init 2 */

    /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

    /* USER CODE BEGIN UART4_Init 0 */

    /* USER CODE END UART4_Init 0 */

    /* USER CODE BEGIN UART4_Init 1 */

    /* USER CODE END UART4_Init 1 */
    huart4.Instance = UART4;
    huart4.Init.BaudRate = 115200;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    huart4.AdvancedInit.AutoBaudRateMode = UART_ADVFEATURE_AUTOBAUDRATE_ONSTARTBIT;
    if (HAL_UART_Init(&huart4) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN UART4_Init 2 */

    /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream5_IRQn interrupt configuration */

    NVIC_SetVector( DMA1_Stream5_IRQn, (uint32_t) DMA1_Stream5_IRQHandler);
    NVIC_SetVector( DMA2_Stream0_IRQn, (uint32_t) DMA2_Stream0_IRQHandler);


    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    /* DMA2_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : USER_Btn_Pin */
    GPIO_InitStruct.Pin = USER_Btn_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
    GPIO_InitStruct.Pin = RMII_MDC_Pin | RMII_RXD0_Pin | RMII_RXD1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
    GPIO_InitStruct.Pin = RMII_REF_CLK_Pin | RMII_MDIO_Pin | RMII_CRS_DV_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : RMII_TXD1_Pin */
    GPIO_InitStruct.Pin = RMII_TXD1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : LD3_Pin LD2_Pin */
    GPIO_InitStruct.Pin = LD3_Pin | LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
    GPIO_InitStruct.Pin = STLK_RX_Pin | STLK_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
    GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : USB_OverCurrent_Pin */
    GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
    GPIO_InitStruct.Pin = USB_SOF_Pin | USB_ID_Pin | USB_DM_Pin | USB_DP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : USB_VBUS_Pin */
    GPIO_InitStruct.Pin = USB_VBUS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
    GPIO_InitStruct.Pin = RMII_TX_EN_Pin | RMII_TXD0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
    lowerData = true;
}

void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *hadc) {
    upperData = true;
}

void HAL_DAC_ConvHalfCpltCallbackCh1 (DAC_HandleTypeDef *hdac) {
}

void HAL_DAC_ConvCpltCallbackCh1 (DAC_HandleTypeDef *hdac) {
    txStatus = TX_TRANSMISSION_DONE;
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

    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
