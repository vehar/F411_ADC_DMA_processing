#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

// Global hardware handle declarations
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_adc1;

bool data_rdy_f = false;

// ADC buffer to store conversion results
const int SAMPLES = 1024;
const int CHANNELS = 4;
__attribute__((aligned(4))) uint32_t adc_values[CHANNELS * SAMPLES] = { 0 };

void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void Error_Handler(void);

// GPIO Initialization
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    // Enable GPIO clocks
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    // Configure GPIO pins for LEDs
    GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // Configure other GPIO pins
    GPIO_InitStruct.Pin = DATA_Ready_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DATA_Ready_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
    HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
    HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);
}

// ADC Initialization
void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = { 0 };

    // Configure ADC instance
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 4;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
        Error_Handler();

    // Configure ADC channels
    for (int channel = 0; channel < 4; ++channel)
    {
        sConfig.Channel = ADC_CHANNEL_0 + channel;
        sConfig.Rank = channel + 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            Error_Handler();
    }
}

// USART Initialization
void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE; // UART_PARITY_EVEN;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
        Error_Handler();
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    if (adcHandle->Instance == ADC1)
    {
        __HAL_RCC_ADC1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**ADC1 GPIO Configuration
        PA0-WKUP     ------> ADC1_IN0
        PA1     ------> ADC1_IN1
        PA2     ------> ADC1_IN2
        PA3     ------> ADC1_IN3
        */
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* ADC1 DMA Init */
        /* ADC1 Init */
        hdma_adc1.Instance = DMA2_Stream0;
        hdma_adc1.Init.Channel = DMA_CHANNEL_0;
        hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
        hdma_adc1.Init.Mode = DMA_CIRCULAR;
        hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
        hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
            Error_Handler();

        __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc1);
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle)
{
    if (adcHandle->Instance == ADC1)
    {
        __HAL_RCC_ADC1_CLK_DISABLE();

        /**ADC1 GPIO Configuration
        PA0-WKUP     ------> ADC1_IN0
        PA1     ------> ADC1_IN1
        PA2     ------> ADC1_IN2
        PA3     ------> ADC1_IN3
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
        HAL_DMA_DeInit(adcHandle->DMA_Handle);
    }
}

// DMA Initialization
void MX_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

int _write(int file, char *ptr, int len)
{
    for (int i = 0; i < len; i++)
        ITM_SendChar((*ptr++));
    return len;
}

void Set_LED_State(uint8_t index)
{
    uint16_t pins[] = { LD4_Pin, LD3_Pin, LD5_Pin, LD6_Pin };
    for (int i = 0; i < 4; i++)
        HAL_GPIO_WritePin(GPIOD, pins[i], (i == index) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint32_t Calculate_Max_Amplitude(uint32_t *buffer, uint32_t channel, uint32_t num_samples,
                                 uint32_t channels)
{
    uint32_t max_val = 0, min_val = UINT32_MAX;

    // Iterate over samples for the specified channel
    for (uint32_t i = 0; i < num_samples; ++i)
    {
        uint32_t val = buffer[i * channels + channel]; // Access interleaved data
        if (val > max_val)
            max_val = val;
        if (val < min_val)
            min_val = val;
    }

    return max_val - min_val; // Amplitude
}

// Main application entry
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    PeriphCommonClock_Config();
    // Initialize peripherals
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_USART1_UART_Init();

    if (HAL_ADC_Start_DMA(&hadc1, adc_values, SAMPLES * SAMPLES) != HAL_OK)
        Error_Handler();

    // Main loop
    while (1)
    {
        // Wait for data
        if (data_rdy_f)
        {
            uint32_t amplitude[CHANNELS] = { 0 };

            // Calculate amplitude for each channel
            for (int i = 0; i < CHANNELS; ++i)
                amplitude[i] = Calculate_Max_Amplitude(adc_values, i, SAMPLES, CHANNELS);

            // Find the channel with the highest amplitude
            uint32_t max_amplitude = amplitude[0];
            int max_channel = 0;

            for (int channel = 0; channel < CHANNELS; ++channel)
            {
                if (amplitude[channel] > max_amplitude)
                {
                    max_amplitude = amplitude[channel];
                    max_channel = channel;
                }
            }

            // Output results via UART
            printf("Channel Amplitudes: ");
            for (int channel = 0; channel < CHANNELS; ++channel)
                printf("%lu ", amplitude[channel]);
            printf("\n");

            // Stream ADC data via ITM for plotting
            // for (int i = 0; i < SAMPLES * CHANNELS; ++i)
            //     ITM_SendChar((uint8_t)(adc_values[i] & 0xFF)); // Example: LSB of ADC value
            for (int i = 0; i < SAMPLES; ++i)
            {
                ITM->PORT[0].u8 = (uint8_t)(adc_values[i * CHANNELS + 0] & 0xFF); // Channel 0
                ITM->PORT[1].u8 = (uint8_t)(adc_values[i * CHANNELS + 1] & 0xFF); // Channel 1
                ITM->PORT[2].u8 = (uint8_t)(adc_values[i * CHANNELS + 2] & 0xFF); // Channel 2
                ITM->PORT[3].u8 = (uint8_t)(adc_values[i * CHANNELS + 3] & 0xFF); // Channel 3
            }

            // Control LEDs based on ADC result
            Set_LED_State(max_channel);
            // HAL_Delay(1000);
            data_rdy_f = false; // Processed
        }
        // Perform other tasks here (e.g., debugging or communication)
    }
}

// System Clock Configuration
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 8;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
}

void PeriphCommonClock_Config(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
    PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
    PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        Error_Handler();
}

void DMA2_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_adc1); }

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        data_rdy_f = true; // Handle the data in the adc_values buffer

        // Restart the ADC conversion
        if (HAL_ADC_Start_DMA(&hadc1, adc_values, 4 * 1024) != HAL_OK)
            Error_Handler();
    }
}

// Placeholder for future use for true realtime
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        // Handle first half of the buffer (adc_values[0] to adc_values[2047])
    }
}

// Error handler function
void Error_Handler(void)
{
    while (1)
    {
        HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin); // Toggle an LED
        HAL_Delay(500);                             // 500 ms delay
    }
}
