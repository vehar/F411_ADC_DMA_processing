#include "main.h"
#include "ARGB.h"
#include "periph_init.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include <math.h>
#include <stdio.h>

volatile bool data_rdy_f = false;

float angles[ADC_CHANNELS] = { 0.0, 90.0, 180.0, 270.0 }; // Microphone angles

// ADC buffer to store conversion results
__attribute__((aligned(2))) uint16_t adc_values[ADC_CHANNELS * SAMPLES] = { 0 };

uint16_t Calculate_Max_Amplitude(uint16_t *buffer, uint8_t channel, uint32_t num_samples,
                                 uint8_t channels)
{
    uint16_t max_val = 0, min_val = UINT16_MAX;

    for (uint16_t i = channel; i < num_samples * channels; i += channels)
    {
        uint16_t val = buffer[i];
        max_val = (val > max_val) ? val : max_val;
        min_val = (val < min_val) ? val : min_val;
    }

    return max_val - min_val; // Amplitude
}

// Estimate angle based on N microphone amplitudes
float EstimateAngle(const uint16_t *amplitudes, const float *angles, uint8_t num_mics)
{
    // Ensure num_mics is valid to avoid division by zero
    if (num_mics == 0)
        return 0.0;

    float X = 0.0, Y = 0.0;

    for (uint8_t i = 0; i < num_mics; i++)
    {
        float rad = angles[i] * M_PI / 180.0; // Convert angle to radians
        X += (float)amplitudes[i] * cosf(rad);
        Y += (float)amplitudes[i] * sinf(rad);
    }

    float angle = atan2f(Y, X) * 180.0 / M_PI; // Convert result to degrees

    // Normalize the angle to the range [0, 360)
    if (angle < 0.0)
        angle += 360.0;

    return angle;
}
/**
 * @brief Map an estimated angle to a circular LED array and set a gradient hue.
 * @param[in] angle Estimated angle in degrees (0-360).
 * @param[in] num_leds Total number of LEDs in the circular array.
 */
void MapAngleToLEDs(float angle, uint8_t num_leds, uint8_t intensity)
{
    // Ensure num_leds is valid to prevent division by zero
    if (num_leds == 0)
        return;

    float step = 360.0f / num_leds; // Angle covered by each LED
    uint8_t primary_led = (uint8_t)(fmodf(angle, 360.0f) / step) % num_leds; // Main LED index

    for (uint8_t i = 0; i < num_leds; i++)
    {
        // Calculate distance from the primary LED, considering wraparound
        int16_t distance = (int16_t)(i - primary_led);
        if (distance < -num_leds / 2)
            distance += num_leds;
        else if (distance > num_leds / 2)
            distance -= num_leds;

        // Compute the absolute distance for symmetric hue mapping
        uint16_t abs_distance = abs(distance);

        // Apply a nonlinear mapping for hue based on the distance 0.0 to 1.0
        float normalized_distance = (float)abs_distance / (num_leds / 2);

        // Hue decreases with distance
        uint8_t hue_variation = (uint8_t)((1.0f - normalized_distance) * 255);
        uint8_t hue = hue_variation > 175 ? 175 : hue_variation; // trimm hue a little

        // Set the HSV color for the LED
        ARGB_SetHSV(i, hue, 255, intensity);
    }

    // Apply changes to the LED array
    ARGB_Show();
}

// Main application entry
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    PeriphCommonClock_Config();

    // Initialize peripherals
    MX_DMA_Init();
    MX_GPIO_Init();
    MX_ADC1_Init();
    // MX_USART1_UART_Init();
    MX_TIM2_Init(); //

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_values, ADC_CHANNELS * SAMPLES) != HAL_OK)
        Error_Handler();

    ARGB_Init(); // Initialize the ARGB library
    ARGB_Clear();
    while (!ARGB_Show())
        ;
    ARGB_SetBrightness(100); // Set brightness (0-255)

#ifdef DEBUG
    // Initial test
    for (int i = 0; i < 4; i++)
    {
        Set_LED_State(i);
        HAL_Delay(500);
    }
    for (int i = 0; i < 360; i++)
    {
        ARGB_SetRGB(i, 255, 0, 0);
        MapAngleToLEDs(i, NUM_LEDS, 100);
        HAL_Delay(100);
    }
#endif
    // Main loop
    while (1)
    {
        // Wait for data
        if (data_rdy_f)
        {
            uint16_t amplitude[ADC_CHANNELS] = { 0 };

            // Calculate amplitude for each channel
            for (int i = 0; i < ADC_CHANNELS; ++i)
                amplitude[i] = Calculate_Max_Amplitude(adc_values, i, SAMPLES, ADC_CHANNELS);

            // Find the channel with the highest amplitude
            uint16_t max_amplitude = amplitude[0];
            int max_channel = 0;

            for (int channel = 0; channel < ADC_CHANNELS; ++channel)
            {
                if (amplitude[channel] > max_amplitude)
                {
                    max_amplitude = amplitude[channel];
                    max_channel = channel;
                }
            }

#ifdef DEBUG
            // Output results via UART
            printf("Channel Amplitudes: ");
            for (int channel = 0; channel < ADC_CHANNELS; ++channel)
                printf("%lu ", amplitude[channel]);
            printf("\n");

            // Stream ADC data via ITM for plotting
            // for (int i = 0; i < SAMPLES * ADC_CHANNELS; ++i)
            //     ITM_SendChar((uint8_t)(adc_values[i] & 0xFF)); // Example: LSB of ADC value
            for (int i = 0; i < SAMPLES; ++i)
            {
                // if (i % 10 == 0) // Send every 10th sample to reduce ITM traffic
                {
                    ITM->PORT[0].u8 =
                        (uint8_t)(adc_values[i * ADC_CHANNELS + 0] & 0xFF); // Channel 0
                    ITM->PORT[1].u8 =
                        (uint8_t)(adc_values[i * ADC_CHANNELS + 1] & 0xFF); // Channel 1
                    ITM->PORT[2].u8 =
                        (uint8_t)(adc_values[i * ADC_CHANNELS + 2] & 0xFF); // Channel 2
                    ITM->PORT[3].u8 =
                        (uint8_t)(adc_values[i * ADC_CHANNELS + 3] & 0xFF); // Channel 3
                }
            }
#endif

            if (max_amplitude > 400)
            {
                // HAL_ADC_Stop_DMA(&hadc1);

                float estimated_angle = EstimateAngle(amplitude, angles, ADC_CHANNELS);
                MapAngleToLEDs(estimated_angle + 180, NUM_LEDS, max_amplitude / 40);

                // Control LEDs based on ADC result
                // Set_LED_State(max_channel);
                data_rdy_f = false; // Processed

                // HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_values, ADC_CHANNELS * SAMPLES);
            }
        }
        // Perform other tasks here (e.g., debugging or communication)
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        data_rdy_f = true; // Handle the data in the adc_values buffer

        // Restart the ADC conversion in manual mode
        // This might not be necessary as circular mode automatically restarts
        // if (HAL_ADC_Start_DMA(&hadc1, adc_values, ADC_CHANNELS * SAMPLES) != HAL_OK)
        //     Error_Handler();
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

void Set_LED_State(uint8_t index)
{
    uint16_t pins[] = { LD4_Pin, LD3_Pin, LD5_Pin, LD6_Pin };
    for (int i = 0; i < 4; i++)
    {
        GPIO_PinState state = (i == index) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(GPIOD, pins[i], state);
    }
}

// For all interrupt handlers for proper linking
extern "C"
{
    int _write(int file, char *ptr, int len)
    {
        // for (int i = 0; i < len; i++)
        //     ITM_SendChar((*ptr++));

        HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
        return len;
    }

    void SysTick_Handler(void)
    {
        HAL_IncTick(); // Increment HAL time base
        HAL_SYSTICK_IRQHandler();
    }

    void DMA2_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_adc1); }

    void DMA1_Stream6_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_tim2_ch2_ch4); }
} // extern "C"