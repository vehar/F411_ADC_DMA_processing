### **README: STM32 ADC with DMA, UART Debugging, and ITM Plotting**

This project demonstrates the use of **STM32 ADC with DMA** to collect data from multiple channels, calculate their amplitudes, send debug information over UART, and visualize ADC data via ITM for real-time plotting. It uses **STM32Cube HAL** and is designed for STM32F4 series microcontrollers.

---

### **Features**
1. **ADC with DMA:**
   - Configured for 4 channels.
   - Collects 1024 samples per channel.
   - Data is stored in an interleaved buffer.

2. **Amplitude Calculation:**
   - Calculates the maximum amplitude (difference between max and min) for each channel.

3. **Debug Output:**
   - Channel amplitudes are sent to UART using `printf`.

4. **ITM Streaming for Plotting:**
   - Streams ADC data for all channels through ITM (Instrumentation Trace Macrocell) for real-time plotting.

5. **LED Control:**
   - Indicates the channel with the highest amplitude by lighting up an LED.

---

### **System Configuration**

#### **Hardware Setup**
- **ADC Channels**: Configured for channels `ADC_CHANNEL_0` to `ADC_CHANNEL_3` (`PA0`, `PA1`, `PA2`, `PA3`).
- **DMA**: Configured for circular mode to continuously fetch ADC data.
- **UART**: Debug output is sent via UART at 115200 baud.
- **LEDs**: GPIO pins `LD4`, `LD3`, `LD5`, and `LD6` are used to display the highest amplitude channel.

#### **Software Configuration**
1. **Core Clock**: Configured to 84 MHz.
2. **ADC Clock Prescaler**: `ADC_CLOCK_SYNC_PCLK_DIV4` for efficient conversion.
3. **UART Debugging**: Implemented with `printf` support.
4. **ITM for Real-Time Data**:
   - Data for plotting is streamed via ITM stimulus ports 0–3.
   - Requires an SWO-enabled debugger.

---

### **Key Functions**

#### **1. ADC Initialization**
```c
void MX_ADC1_Init(void)
```
- Configures ADC1 in scan mode with DMA.
- Enables continuous conversion for 4 channels.

#### **2. DMA Initialization**
```c
void MX_DMA_Init(void)
```
- Configures DMA2 for ADC1 in circular mode.
- Enables interrupts for transfer complete and error handling.

#### **3. ITM Debug Output**
```c
int _write(int file, char *ptr, int len)
```
- Redirects `printf` to ITM for real-time data output.

#### **4. Amplitude Calculation**
```c
uint32_t Calculate_Max_Amplitude(uint32_t *buffer, uint32_t channel, uint32_t num_samples, uint32_t channels)
```
- Computes the maximum amplitude of the specified channel from interleaved ADC data.

#### **5. LED Control**
```c
void Set_LED_State(uint8_t index)
```
- Activates an LED corresponding to the channel with the highest amplitude.

---

### **Data Handling**

1. **Interleaved ADC Data Storage**:
   - DMA stores data in an interleaved manner:
     ```
     Buffer Index:  [0] [1] [2] [3] [4] [5] ...
     Data Channel:   0   1   2   3   0   1   ...
     ```
   - Each channel's data is accessed using:
     ```c
     buffer[i * channels + channel];
     ```

2. **Amplitude Calculation**:
   - Extracts max and min values for each channel and computes the difference.

3. **Debugging via UART**:
   - Prints the amplitudes of all channels to the UART terminal.

4. **Plotting via ITM**:
   - Streams ADC data for all channels to ITM stimulus ports (0–3).

---

### **ITM and SWO Setup**

#### **STM32CubeIDE Configuration**:
1. **Enable SWO**:
   - Go to `Run > Debug Configurations > Debugger > Serial Wire Viewer (SWV)`.
   - Enable **"Enable SWV Trace"** and set the core clock (HCLK) to 84 MHz.
   
2. **Open SWV ITM Data Console**:
   - `Window > Show View > SWV > SWV ITM Data Console`.
   - Enable stimulus ports 0–3 for ADC data visualization.

#### **Code for ITM Output**:
```c
for (int i = 0; i < SAMPLES; ++i)
{
    ITM->PORT[0].u8 = (uint8_t)(adc_values[i * CHANNELS + 0] & 0xFF); // Channel 0
    ITM->PORT[1].u8 = (uint8_t)(adc_values[i * CHANNELS + 1] & 0xFF); // Channel 1
    ITM->PORT[2].u8 = (uint8_t)(adc_values[i * CHANNELS + 2] & 0xFF); // Channel 2
    ITM->PORT[3].u8 = (uint8_t)(adc_values[i * CHANNELS + 3] & 0xFF); // Channel 3
}
```

---

### **Building and Running the Project**

1. **Build**:
   - Use STM32CubeIDE or VS Code with PlatformIO to build the project.

2. **Flash**:
   - Flash the firmware to the STM32 board.

3. **Debug**:
   - Open the SWV ITM Data Console in your IDE for real-time plotting.
   - Connect a UART terminal (e.g., PuTTY) at 115200 baud to view debug output.

4. **Visualize**:
   - Enable graphs for stimulus ports 0–3 in the SWV ITM Data Console to plot the ADC data for each channel.

---

### **Expected Behavior**
- ADC continuously samples 4 channels and stores the results in the buffer.
- The program calculates the amplitude of each channel in real-time.
- Debug output shows the amplitudes of all channels.
- ITM streams ADC data for real-time plotting.
- LEDs indicate the channel with the highest amplitude.

---

### **Potential Enhancements**
1. **Data Filtering**:
   - Apply a low-pass or moving average filter to ADC data for noise reduction.

2. **Dynamic Channel Configuration**:
   - Allow the number of channels to be dynamically configurable.

3. **Improved Plotting**:
   - Use tools like STM32CubeMonitor for enhanced visualization.

---
