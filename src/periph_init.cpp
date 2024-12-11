#include "periph_init.h"
#include "main.h"

// Global Hardware Handle Declarations
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;
TIM_HandleTypeDef htim2;

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
#if 1
    // From led prj

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 50;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
        Error_Handler();
#else
    // From adc proj
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;  // Enable HSI
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF; // Ensure HSE is disabled
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;            // Adjust for 16 MHz HSI
    RCC_OscInitStruct.PLL.PLLN = 192;           // Multiplier
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4; // Divider for 48 MHz PLL output
    RCC_OscInitStruct.PLL.PLLQ = 8;             // Divider for USB/SDIO/Random
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    // Configure the clock source and prescalers
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Use PLL as SYSCLK
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        // No division for HCLK
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;         // APB1 (low-speed peripherals)
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;         // APB2 (high-speed peripherals)
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
        Error_Handler();

    // Disable the Window Watchdog Clock (if not used)
    __HAL_RCC_WWDG_CLK_DISABLE();

    // Update the SystemCoreClock variable
    SystemCoreClockUpdate();
#endif
}

void PeriphCommonClock_Config(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;

    // Conditional compilation for microcontroller-specific fields
#if defined(STM32F411xE) || defined(STM32F469xx) // Add families that support PLLI2SM
    PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
#endif

    PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
    PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        Error_Handler();
}

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
}

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
        PA0     ------> ADC1_IN0
        PA1     ------> ADC1_IN1
        PA2     ------> ADC1_IN2
        PA3     ------> ADC1_IN3
        */
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* ADC1 DMA Init */
        hdma_adc1.Instance = DMA2_Stream0;
        hdma_adc1.Init.Channel = DMA_CHANNEL_0;
        hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adc1.Init.MemDataAlignment = DMA_PDATAALIGN_HALFWORD;
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
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
        HAL_DMA_DeInit(adcHandle->DMA_Handle);
    }
}

// ADC Initialization
void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = { 0 };

    // Configure ADC instance
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = CHANNELS; // Sequential scan
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
        Error_Handler();

    // Configure ADC channels
    for (int channel = 0; channel < CHANNELS; ++channel)
    {
        sConfig.Channel = ADC_CHANNEL_0 + channel;
        sConfig.Rank = channel + 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            Error_Handler();
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    if (htim->Instance == TIM2)
    {
        // Enable GPIOB clock
        __HAL_RCC_GPIOB_CLK_ENABLE();

        // Configure PB3 as TIM2_CH2 (Alternate Function AF1)
        GPIO_InitStruct.Pin = GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
    TIM_OC_InitTypeDef sConfigOC = { 0 };
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0; // Updated dynamically by ARGB
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFF; // Updated dynamically by ARGB
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
        Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
        Error_Handler();

    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
        Error_Handler();

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();

    HAL_TIM_MspPostInit(&htim2);
}

void MX_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    // Enable the clock for DMA1
    __HAL_RCC_DMA1_CLK_ENABLE();

    // Configure the DMA stream for TIM2_CH2
    hdma_tim2_ch2_ch4.Instance = DMA1_Stream6;               // TIM2_CH2 mapped to DMA1_Stream6
    hdma_tim2_ch2_ch4.Init.Channel = DMA_CHANNEL_3;          // Channel 3 for TIM2_CH2
    hdma_tim2_ch2_ch4.Init.Direction = DMA_MEMORY_TO_PERIPH; // Memory to peripheral
    hdma_tim2_ch2_ch4.Init.PeriphInc = DMA_PINC_DISABLE;     // Do not increment peripheral
    hdma_tim2_ch2_ch4.Init.MemInc = DMA_MINC_ENABLE;         // Increment memory address
    hdma_tim2_ch2_ch4.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD; // Peripheral data width:
    hdma_tim2_ch2_ch4.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;    // Memory data width:
    hdma_tim2_ch2_ch4.Init.Mode = DMA_CIRCULAR;                       // Normal mode for one-shot
    hdma_tim2_ch2_ch4.Init.Priority = DMA_PRIORITY_HIGH;              // High priority for DMA
    hdma_tim2_ch2_ch4.Init.FIFOMode = DMA_FIFOMODE_DISABLE;           // FIFO not used

    // Initialize the DMA
    if (HAL_DMA_Init(&hdma_tim2_ch2_ch4) != HAL_OK)
        Error_Handler(); // Handle initialization error

    // Link the DMA handle to TIM2
    __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC2], hdma_tim2_ch2_ch4);

    // Configure and enable the DMA interrupt
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}