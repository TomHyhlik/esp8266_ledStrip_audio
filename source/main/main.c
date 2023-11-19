/**
  ******************************************************************************
  * @brief   
  ******************************************************************************
**/


/* Includes ------------------------------------------------------------------*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/adc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/rtc.h"

#include "ws2812.h"
#include "fft.h"

/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/



#define PIN_LED_INTEGRATED                   GPIO_NUM_2
#define PIN_DBG1                             GPIO_NUM_5
#define PIN_WS2812                           GPIO_NUM_4

#define LEDSTRIP_LED_CNT                     (2)    // Number of "pixels"

static const char *TAG = "ledStrip_audio";

#define ADC_SAMPLES_CNT                     (20000)


/* Private macro -------------------------------------------------------------*/
#define DELAY_MS(ms)            vTaskDelay((ms) / portTICK_PERIOD_MS)
#define DEBUG_PRINT(...)  printf(__VA_ARGS__)

    


/* Private variables ---------------------------------------------------------*/
QueueHandle_t dataQueue;

/* Private functions ---------------------------------------------------------*/
void spectrScaledLog_band_GetFreqRange(FFT_PRECISION* frequencyScale, int freqBandCnt);







#define BUFFER_EXTENSION_MULTIPLIER     1

#define SPECTR_LOG_SCALE_BAND_cNT        60

void spectrScaledLog_band_GetFreqRange(FFT_PRECISION* frequencyScale, int freqBandCnt);

FFT_PRECISION fft_getMag(FFT_PRECISION* input);




/**************************************************************************//**
* @brief   Function name
* @param   List of parameters and related description
* @details Detailed description of implemented functionality
*******************************************************************************/
static void task_01_adc()
{
    esp_err_t rsp;
    uint16_t adc_data[ADC_SAMPLES_CNT];

    DEBUG_PRINT("TASK %s ENTRY________________________________\n", __func__);

    /* LED Blink */
    // while (1) 
    // {
    //     gpio_set_level(PIN_LED_INTEGRATED, 0);
    //     vTaskDelay(100 / portTICK_RATE_MS);
    //     gpio_set_level(PIN_LED_INTEGRATED, 1);
    //     vTaskDelay(1000 / portTICK_RATE_MS);
    // }







    FFT_PRECISION spectrumScaleLog_freq[SPECTR_LOG_SCALE_BAND_cNT];
    FFT_PRECISION spectrumScaleLog_mag[SPECTR_LOG_SCALE_BAND_cNT];

    // Construct input signal
    FFT_PRECISION sample_rate = 200000; // signal sampling rate
    FFT_PRECISION signal_length_ms = 1;//50;
    FFT_PRECISION f = 1010;    // frequency of the artifical signal




    /* Get logarithmic scale of the spectrum */
    spectrScaledLog_band_GetFreqRange(spectrumScaleLog_freq, SPECTR_LOG_SCALE_BAND_cNT);

    // for (int i = 0; i < SPECTR_LOG_SCALE_BAND_cNT; i++)
    // {
    //     printf("%d\t%.3f\n", (int)i, spectrumScaleLog_freq[i]);
    // }

    int input_cnt = BUFFER_EXTENSION_MULTIPLIER * signal_length_ms / 1000 * sample_rate; // n has to be greater than 1

    FFT_PRECISION input[input_cnt];  








    /* Initialize fourier transformer */
    FFTTransformer* transformer = create_fft_transformer(input_cnt, FFT_SCALED_OUTPUT);



















    while (1) 
    {
        gpio_set_level(PIN_DBG1, 1);
        rsp = adc_read_fast(adc_data, ADC_SAMPLES_CNT);
        gpio_set_level(PIN_DBG1, 0);


        if (ESP_OK != rsp)
        {
            DEBUG_PRINT("ERROR: ADC Data Sampling\n");
            while (1);
        }





        {





            
            memset(input, 0, sizeof(input));



            // FFT_PRECISION * input = myinput;
            // int n = sizeof(myinput)/sizeof(double);

            // printf("\nTime data START =======\n");
            // for(int i = 0; i < n; i ++) 
            //     printf("%lf,",i/sample_rate);
            // printf("\nTime data END =======\n");

            /* Set the input data */
            for (int i = 0; i < sizeof(input)/sizeof(FFT_PRECISION)/BUFFER_EXTENSION_MULTIPLIER; i++) 
            {
                //2*PI*f*t
                // input[i] = 1 * sin(2 * M_PI * f * i/sample_rate);
                input[i] = 1 * sin(2 * M_PI * f * i/sample_rate) +
                            1 * sin(2 * M_PI * f/2 * i/sample_rate) +
                            1 * sin(2 * M_PI * f/10 * i/sample_rate) +
                            1 * sin(2 * M_PI * f*2 * i/sample_rate);
            }

            // printf("\nRaw data START =======\n");
            // for(int i = 0; i < sizeof(input)/sizeof(FFT_PRECISION); i ++) 
            // {
            //     printf("Raw_sample[%d] =\t%lf\n,", i, input[i]);
            // }
            // printf("\nRaw data END =======\n");





            // Transform signal
            // fft_forward(transformer, input);



            /* Scale spectrum to log - use top value of the spectrum */
            // {
            //     FFT_PRECISION cnt = 0;
            //     FFT_PRECISION mag = 0;

            //     for (int band = 0; band < SPECTR_LOG_SCALE_BAND_cNT; band++)
            //     {
            //         for (int i = 0; i < sizeof(input)/sizeof(FFT_PRECISION); i += 2)
            //         {
            //             /* Calc FREQUENCY */
            //             FFT_PRECISION freq = i / 2 * sample_rate / input_cnt;

            //             if (freq < spectrumScaleLog_freq[band])
            //             {
            //                 continue;
            //             }
            //             else if (freq >= spectrumScaleLog_freq[band + 1])
            //             {
            //                 if (cnt == 0)
            //                 {
            //                     if (i >= 2)
            //                     {
            //                         spectrumScaleLog_mag[band] = fft_getMag(&input[i-2]);
            //                     }
            //                     break;
            //                 }
            //                 else
            //                 {
            //                     spectrumScaleLog_mag[band] = mag;
            //                 }
            //                 mag = 0;
            //                 cnt = 0;
            //                 break;
            //             }

            //             FFT_PRECISION mag_current = fft_getMag(&input[i]);

            //             if (mag_current > mag)
            //             {
            //                 mag = mag_current;
            //             }

            //             cnt += 1;

            //             if (band >= SPECTR_LOG_SCALE_BAND_cNT) break; // rm later
            //         }
            //     }
            // }





        }



        vTaskDelay(20 / portTICK_RATE_MS);


        printf("***\n");


        // uint64_t variance = Calc_Variance(adc_data, ADC_SAMPLES_CNT);
        // DEBUG_PRINT("Variance: %u\n", (uint32_t)variance);

        // vTaskDelay(1000 / portTICK_RATE_MS);
    }
}


/**************************************************************************//**
* @brief   fft_getMag
* @param   List of parameters and related description
* @details Calc MAGNITUDE
*******************************************************************************/
FFT_PRECISION fft_getMag(FFT_PRECISION* input)
{
    FFT_PRECISION cos_comp = input[0];
    FFT_PRECISION sin_comp = input[1];
    FFT_PRECISION mag = 1000 * sqrt((cos_comp * cos_comp) + (sin_comp * sin_comp));
    return mag;           
}


/**************************************************************************//**
* @brief   Function name
* @param   List of parameters and related description
* @details Detailed description of implemented functionality
*******************************************************************************/
void spectrScaledLog_band_GetFreqRange(FFT_PRECISION* frequencyScale, int freqBandCnt)
{
    FFT_PRECISION start = 1.8;
    FFT_PRECISION stop = 4.3;

    FFT_PRECISION multiplier = (stop - start) / freqBandCnt;

    for (int i = 0; i < freqBandCnt; i++)
    {        
        frequencyScale[i] = 1 * pow(10, (start + ((FFT_PRECISION)(multiplier * i))));
        // printf("%d\t%.3f\n", (int)i, frequencyScale[i]);
    }
}

/**************************************************************************//**
* @brief   Function name
* @param   List of parameters and related description
* @details Detailed description of implemented functionality
*******************************************************************************/
uint64_t Calc_Variance(uint16_t* adcSamples, uint32_t adcSampleCnt)
{
    uint64_t sum = 0;
    uint64_t mean;
    int64_t sum_squared_deviations = 0;
    uint64_t variance;

    for (uint32_t i = 0; i < adcSampleCnt; i++)
    {
        sum += adcSamples[i];
    }

    mean = sum / adcSampleCnt;

    // Calculate the sum of squared deviations
    for (uint32_t i = 0; i < adcSampleCnt; i++) 
    {
        int32_t deviation = adcSamples[i] - mean;
        sum_squared_deviations += deviation * deviation;
    }

    // Calculate the variance
    variance = sum_squared_deviations / adcSampleCnt;

    return variance;
}



/**************************************************************************//**
* @brief   Function name
* @param   List of parameters and related description
* @details Detailed description of implemented functionality
*******************************************************************************/
static void task_02()
{
    while (1)
    {
        // ESP_LOGI(TAGN, "Task halted");

        DEBUG_PRINT("----------------Task_02\n");

        vTaskDelay(500 / portTICK_RATE_MS);
    }
}






/**************************************************************************//**
* @brief   Function name
* @param   List of parameters and related description
* @details Detailed description of implemented functionality
*******************************************************************************/
void task_ledStrip_write(void *pvParameters)
{
    const uint8_t anim_step = 10;
    const uint8_t anim_max = 250;

    /* Init WS2812 pin */
    gpio_set_direction(PIN_WS2812, GPIO_MODE_OUTPUT);


    ws2812_rgb_t color = WS2812_RGB(anim_max, 0, 0);
    uint8_t step = 0;

    ws2812_rgb_t color2 = WS2812_RGB(anim_max, 0, 0);
    uint8_t step2 = 0;

    while (1)
    {
        color = color2;
        step = step2;

        // Start a data sequence (disables interrupts)
        ws2812_seq_start();

        for (uint8_t i = 0; i < LEDSTRIP_LED_CNT; i++)
        {
            // send a color
            ws2812_seq_rgb(PIN_WS2812, color.num);

            // now we have a few hundred nanoseconds
            // to calculate the next color

            if (i == 1) {
                color2 = color;
                step2 = step;
            }

            switch (step) {
                case 0: color.g += anim_step; if (color.g >= anim_max) step++;  break;
                case 1: color.r -= anim_step; if (color.r == 0) step++; break;
                case 2: color.b += anim_step; if (color.b >= anim_max) step++; break;
                case 3: color.g -= anim_step; if (color.g == 0) step++; break;
                case 4: color.r += anim_step; if (color.r >= anim_max) step++; break;
                case 5: color.b -= anim_step; if (color.b == 0) step = 0; break;
            }
        }

        // End the data sequence, display colors (interrupts are restored)
        ws2812_seq_end();


        DEBUG_PRINT("----------------demo\n");

        // wait a bit
        DELAY_MS(400);
    }
}


/**************************************************************************//**
* @brief   Function name
* @param   List of parameters and related description
* @details Detailed description of implemented functionality
*******************************************************************************/
void app_main()
{
    /* Set MCU CLK to 160 MHz */
    rtc_clk_cpu_freq_set(RTC_CPU_FREQ_160M);

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    DEBUG_PRINT("This is ESP8266 chip with %d CPU cores, WiFi, ",
            chip_info.cores);
    DEBUG_PRINT("silicon revision %d, ", chip_info.revision);
    DEBUG_PRINT("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    rtc_cpu_freq_t rtcCpuFreq = rtc_clk_cpu_freq_get();
    DEBUG_PRINT("CLK: ");
    switch (rtcCpuFreq)
    {
        case RTC_CPU_FREQ_80M:
            DEBUG_PRINT("80");
            break;
        case RTC_CPU_FREQ_160M:
            DEBUG_PRINT("160");
            break;
        default:
            DEBUG_PRINT("Unknown");
    }
    DEBUG_PRINT(" MHz\n");

    /* Init GPIO */
    gpio_set_direction(PIN_LED_INTEGRATED, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_DBG1, GPIO_MODE_OUTPUT);


    /* ADC init */
    adc_config_t adc_config;

    // Depend on menuconfig->Component config->PHY->vdd33_const value
    // When measuring system voltage(ADC_READ_VDD_MODE), vdd33_const must be set to 255.
    adc_config.mode = ADC_READ_TOUT_MODE;
    adc_config.clk_div = 8; // ADC sample collection clock = 80MHz/clk_div = 10MHz
    ESP_ERROR_CHECK(adc_init(&adc_config));

    // 2. Create a adc task to read adc value
    xTaskCreate(task_01_adc, "task_01_adc", (ADC_SAMPLES_CNT*2) + 25000, NULL, 5, NULL);
    // xTaskCreate(task_02, "task_02", 1024, NULL, 5, NULL);


    // xTaskCreate(task_ledStrip_write, "task_ledStrip_write", 1024, NULL, 5, NULL);


    // xTaskCreate(task2, "Task 2", configMINIMAL_STACK_SIZE, NULL, 1, &task2Handle);

}
/**********************************************************/
