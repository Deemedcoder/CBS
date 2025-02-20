#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <stm32f1xx_hal_can.h>  // For STM32F1 CAN peripheral

// Create ADS1115 object
Adafruit_ADS1115 ads;

// Define ADC Channels (PA0 to PB1)
#define NUM_ADC_CHANNELS 10
const uint8_t ADC_PINS[NUM_ADC_CHANNELS] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1};

// ADC and DMA handle structures
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

// Number of ADC samples per channel
#define NUM_SAMPLES 100
uint16_t adcBuffer[NUM_ADC_CHANNELS * NUM_SAMPLES]; // Buffer to store ADC values

// Variables to store converted voltage values
float adcVoltages[NUM_ADC_CHANNELS];

HardwareSerial Serial1(PA10, PA9);  // Using Serial1 (PA10, PA9)

// Function prototypes
void MX_ADC1_Init(void);
void MX_DMA_Init(void);

void setup() {
    Serial1.begin(115200); // Initialize Serial1
    delay(100);

    // Initialize DMA and ADC
    MX_DMA_Init();
    MX_ADC1_Init();

    // Start ADC in DMA mode
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, NUM_ADC_CHANNELS * NUM_SAMPLES);

    Wire.begin();

    // Initialize ADS1115
    if (!ads.begin(0x48)) {  // Ensure correct I2C address
        Serial1.println("Failed to initialize ADS1115!");
        while (1);
    }

    // Set ADS1115 Gain (for ±4.096V range)
    ads.setGain(GAIN_ONE);
}

void loop() {
    // Compute the average ADC value and convert to voltage
    for (int ch = 0; ch < NUM_ADC_CHANNELS; ch++) {
        uint32_t sum = 0;
        for (int i = 0; i < NUM_SAMPLES; i++) {
            sum += adcBuffer[ch + (i * NUM_ADC_CHANNELS)];
        }
        float avgADC = sum / NUM_SAMPLES;
        adcVoltages[ch] = (avgADC * 3.3) / 4095.0;  // Convert ADC value to voltage
    }

    // Read ADS1115 Channels (CH11 to CH13)
    float adsVoltages[3];
    adsVoltages[0] = (ads.readADC_SingleEnded(0) * 4.096) / 32768.0; // CH11
    adsVoltages[1] = (ads.readADC_SingleEnded(1) * 4.096) / 32768.0; // CH12
    adsVoltages[2] = (ads.readADC_SingleEnded(2) * 4.096) / 32768.0; // CH13

    // Create a string to store sensor statuses
    String statusString = "";

    // Process ADC Channels (CH1 - CH10)
    for (int ch = 0; ch < NUM_ADC_CHANNELS; ch++) {
        if (adcVoltages[ch] < 2.0) {
            statusString += "0";  // Sensor Disconnected
        } else if (adcVoltages[ch] >= 2.3) {
            statusString += "1";  // Fault
        } else {
            statusString += "0";  // Normal Operation (Default)
        }
    }

    // Process ADS1115 Channels (CH11 - CH13)
    for (int ch = 0; ch < 3; ch++) {
        if (adsVoltages[ch] < 2.0) {
            statusString += "0";  // Sensor Disconnected
        } else if (adsVoltages[ch] >= 2.3) {
            statusString += "1";  // Fault
        } else {
            statusString += "0";  // Normal Operation (Default)
        }
    }

    // Send the final status string over Serial1
   
    Serial1.println(statusString);

    delay(1000); // Print values every second
}

/**
 * Initialize ADC1 with DMA (Multi-channel)
 */
void MX_ADC1_Init(void) {
    __HAL_RCC_ADC1_CLK_ENABLE(); // Enable ADC1 clock

    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE; // Enable scanning for multiple channels
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = NUM_ADC_CHANNELS; // Number of channels to convert
    HAL_ADC_Init(&hadc1);

    // Configure all ADC channels
    ADC_ChannelConfTypeDef sConfig = {0};
    for (int ch = 0; ch < NUM_ADC_CHANNELS; ch++) {
        sConfig.Channel = ch;  // Channels are numbered from 0 to 7 (PA0-PA7)
        sConfig.Rank = ch + 1; // Conversion rank (1-based)
        sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
        HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    }

    // Link ADC with DMA
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
}

/**
 * Initialize DMA for ADC1
 */
void MX_DMA_Init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE(); // Enable DMA1 clock

    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_adc1);
}
