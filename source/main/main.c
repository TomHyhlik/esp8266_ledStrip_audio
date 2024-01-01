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
#include <time.h>


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
typedef struct 
{
    uint8_t h;
    uint8_t s;
    uint8_t v;
}
Pixel_hsv_t;

/* Private constants ---------------------------------------------------------*/


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// CONFIGURATION APP BUILD                                                    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
// #define APP_TYPE_RAINBOW
// #define APP_TYPE_AUDIO_DEVIATION_1               // noise turns on the leds and keeps it on for a short time, so it is not so responsive to the noises
#define APP_TYPE_AUDIO_DEVIATION_2                  // leds are on only during the noise occuring, so it is very responsive
// #define APP_TYPE_AUDIO_SPECTRUM

#define DEBUG_ENABLED

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// CONFIGURATION HW                                                           //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
#define PIN_LED_INTEGRATED                   GPIO_NUM_2
#define PIN_DBG1                             GPIO_NUM_16
#define PIN_WS2812                           GPIO_NUM_5
#define LEDSTRIP_LED_CNT                     (60)    // Number of "pixels"

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                                                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#define LEDSTRIP_CNT_MAX_SWITCHED             (5)


#define ADC_SAMPLES_CNT                     (20000)


#define STDEV_THRESHOLD_NOISE                   (100)   // Standard deviation threshold below which the sampled sound data are considered as noise
#define STDEV_THRESHOLD_TOP                     (1500)  // Standard deviation threshold ... todo
#define STDEV_USED_RANGE                        (STDEV_THRESHOLD_TOP - STDEV_THRESHOLD_NOISE)

#define MAX_LED_CNT_SWITCHED                    (10)  // Maximum nuber of LEDs turned on in one sampling cycle
#define LED_TURINING_OFF_SPEED                  (0.9) // 1 is quickest turning the LEDs off, lower number is fastest

#define ADC_SAMPLES_CNT_VARIANCE                (1000) // ADC samples count in one cycle


static const char *TAG = "ledStrip_audio";

/* Private macro -------------------------------------------------------------*/
#define DELAY_MS(ms)            vTaskDelay((ms) / portTICK_PERIOD_MS)
#define DEBUG_PRINT(...)  printf(__VA_ARGS__)

    


/* Private variables ---------------------------------------------------------*/
QueueHandle_t dataQueue;

/* Private functions ---------------------------------------------------------*/
void spectrScaledLog_band_GetFreqRange(FFT_PRECISION* frequencyScale, int freqBandCnt);


void hsv_to_rgb(uint8_t h, uint8_t s, uint8_t v, ws2812_rgb_t *rgb) ;





#define BUFFER_EXTENSION_MULTIPLIER     1

#define SPECTR_LOG_SCALE_BAND_cNT        60

void spectrScaledLog_band_GetFreqRange(FFT_PRECISION* frequencyScale, int freqBandCnt);

FFT_PRECISION fft_getMag(FFT_PRECISION* input);




/**************************************************************************//**
* @brief   Function name
* @param   List of parameters and related description
* @details Detailed description of implemented functionality
*******************************************************************************/
void printDouble(double v, int decimalDigits)
{
    int i = 1;
    int intPart, fractPart;
    for (;decimalDigits!=0; i*=10, decimalDigits--);
    intPart = (int)v;
    fractPart = (int)((v-(double)(int)v)*i);
    if(fractPart < 0) fractPart *= -1;
    printf("%i.%i", intPart, fractPart);
}

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
    FFT_PRECISION signal_length_ms = 2;//50;
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
        printf("Microphone sampling.....................\n");

        /* Sample data via ADC */
        gpio_set_level(PIN_DBG1, 1);
        rsp = adc_read_fast(adc_data, ADC_SAMPLES_CNT);
        gpio_set_level(PIN_DBG1, 0);


        if (ESP_OK != rsp)
        {
            DEBUG_PRINT("ERROR: ADC Data Sampling\n");
            while (1);
        }





    
        /* Set the input data */
        memset(input, 0, sizeof(input));
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
        fft_forward(transformer, input);



        /* Scale spectrum to log - use top value of the spectrum */
        {
            FFT_PRECISION cnt = 0;
            FFT_PRECISION mag = 0;

            for (int band = 0; band < SPECTR_LOG_SCALE_BAND_cNT; band++)
            {
                for (int i = 0; i < sizeof(input)/sizeof(FFT_PRECISION); i += 2)
                {
                    /* Calc FREQUENCY */
                    FFT_PRECISION freq = i / 2 * sample_rate / input_cnt;

                    if (freq < spectrumScaleLog_freq[band])
                    {
                        continue;
                    }
                    else if (freq >= spectrumScaleLog_freq[band + 1])
                    {
                        if (cnt == 0)
                        {
                            if (i >= 2)
                            {
                                spectrumScaleLog_mag[band] = fft_getMag(&input[i-2]);
                            }
                            break;
                        }
                        else
                        {
                            spectrumScaleLog_mag[band] = mag;
                        }
                        mag = 0;
                        cnt = 0;
                        break;
                    }

                    FFT_PRECISION mag_current = fft_getMag(&input[i]);

                    if (mag_current > mag)
                    {
                        mag = mag_current;
                    }

                    cnt += 1;

                    if (band >= SPECTR_LOG_SCALE_BAND_cNT) break; // rm later
                }
            }
        }




        /* Print output */
        for (int i = 0; i < sizeof(input)/sizeof(FFT_PRECISION); i += 2)
        {
            FFT_PRECISION freq = i / 2 * sample_rate / input_cnt;
            FFT_PRECISION cos_comp = input[i];
            FFT_PRECISION sin_comp = input[i+1];
            FFT_PRECISION mag = 1000 * sqrt((cos_comp * cos_comp) + (sin_comp * sin_comp));
            // printf("Freq:%f,COS:%f\n", freq, cos_comp);
            // printf("Freq:%f,SIN:%f\n", freq, sin_comp);

            /* Print */
            if (freq >= 50 && freq <= 2500)
            {
                printf("%d\t%.1f\t%d\n", i, freq, (int)(mag));
            }
        }

        printf("---------- Spectrum Log Scaled ----------\n");
        for (int i = 0; i < SPECTR_LOG_SCALE_BAND_cNT; i++)
        {
            // printf("%d\t%.1f\t%.1f\n", (int)i, spectrumScaleLog_freq[i], spectrumScaleLog_mag[i]);

            printf("%d\t", (int)i); 
            printDouble(spectrumScaleLog_freq[i], 2);
            printf("\t");
            printDouble(spectrumScaleLog_mag[i], 2);
            printf("\n");
        }
        
        free_fft_transformer(transformer);
        printf("---------- wrapped fourier transform example end ----------\n");


            



        vTaskDelay(1000 / portTICK_RATE_MS);
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
uint32_t calculate_standardDeviation(uint16_t* adcSamples, uint32_t adcSampleCnt)
{
    double sum = 0;
    double mean;
    double sum_squared_deviations = 0;
    double variance = 0;
    double standardDeviation = 0;

    for (uint32_t i = 0; i < adcSampleCnt; i += 2)
    {
        sum += adcSamples[i];
    }

    mean = sum / (double)(adcSampleCnt/2);

    // Calculate the sum of squared deviations
    for (uint32_t i = 0; i < adcSampleCnt; i += 2)
    {
        double deviation = (double)(adcSamples[i]) - mean;
        sum_squared_deviations += deviation * deviation;
    }

    // Calculate the variance
    variance = sum_squared_deviations / (double)(adcSampleCnt/2);

    if (variance != 0)
    {
        standardDeviation = sqrt(variance);
    }

    return (uint32_t)(standardDeviation * 100);
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
void task_audioIndicator_deviation_2(void *pvParameters)
{
    esp_err_t rsp;
    uint16_t adc_data[ADC_SAMPLES_CNT_VARIANCE];
    ws2812_rgb_t newPixel;
    ws2812_rgb_t pixels_rgb[LEDSTRIP_LED_CNT];
    Pixel_hsv_t pixels_hsv[LEDSTRIP_LED_CNT];
    double sensitivityMultiplier = 1.0;

    memset(pixels_rgb, 0, LEDSTRIP_LED_CNT*sizeof(ws2812_rgb_t));
    memset(pixels_hsv, 0, LEDSTRIP_LED_CNT*sizeof(Pixel_hsv_t));
    
    /* Init WS2812 pin */
    gpio_set_direction(PIN_WS2812, GPIO_MODE_OUTPUT);

    /* Init random number generation */
    srand(time(NULL));

    while (1)
    {
        uint32_t ledsCntToBeWritten;
        uint8_t ledStrip_currentLedIndex;
        uint8_t h;
        uint8_t s = 0xFF;
        uint8_t v;
        uint32_t stdDev;
        uint32_t stdDev_updated;
        
        /* Sample data via ADC */
        gpio_set_level(PIN_DBG1, 1);
        rsp = adc_read_fast(adc_data, ADC_SAMPLES_CNT_VARIANCE);
        gpio_set_level(PIN_DBG1, 0);
        if (ESP_OK != rsp)
        {
            DEBUG_PRINT("ERROR: ADC Data Sampling\n");
        }

        stdDev = calculate_standardDeviation(adc_data, ADC_SAMPLES_CNT_VARIANCE);

        stdDev_updated = (uint32_t)((double)stdDev * sensitivityMultiplier);

        /* Get v and ledsCntToBeWritten */
        if (stdDev_updated < STDEV_THRESHOLD_NOISE)
        {
            v = 0;
            ledsCntToBeWritten = 0;
        }
        else if (STDEV_THRESHOLD_TOP < stdDev_updated)
        {
                v = 0xFF;
                ledsCntToBeWritten = MAX_LED_CNT_SWITCHED;
        }
        else
        {
            v = (uint8_t)((double)(stdDev_updated - STDEV_THRESHOLD_NOISE) / ((double)(STDEV_USED_RANGE)/(double)0xFF));
            ledsCntToBeWritten = (uint32_t)(MAX_LED_CNT_SWITCHED * ((double)stdDev_updated / (double)STDEV_THRESHOLD_TOP));
        }

        /* Update sensitivity */
        if ((MAX_LED_CNT_SWITCHED / 2) < ledsCntToBeWritten)
        {
            /* Decrease sensitivity */
            sensitivityMultiplier *= (double)MAX_LED_CNT_SWITCHED / (double)(MAX_LED_CNT_SWITCHED + ((double)ledsCntToBeWritten * 0.08));
        }
        else
        {
            /* Increase sensitivity (1 is maximum) */
            if (sensitivityMultiplier < 1)
            {
                if (0 == ledsCntToBeWritten) { sensitivityMultiplier *= 1.002; }
                else                         { sensitivityMultiplier *= 1.0005; }
            }
        }

        /* Generate random led index */
        ledStrip_currentLedIndex = rand() % LEDSTRIP_LED_CNT;
        /* Update the led index based on the leds update count */
        if (ledStrip_currentLedIndex + ledsCntToBeWritten >= LEDSTRIP_LED_CNT)
        {
            ledStrip_currentLedIndex = LEDSTRIP_LED_CNT - ledsCntToBeWritten;
        }

        /* Get current h */
        h = rand() % 70;

        /* Ge RGB values */
        if (0 == v)
        {
            newPixel.r = 0;
            newPixel.g = 0;
            newPixel.b = 0;
        }
        hsv_to_rgb(h, s, v, &newPixel.num);


        /* Updat the pixel array to shut down the lights slowly */
        for (int i = 0; i < LEDSTRIP_LED_CNT; i++)
        {
            if (pixels_hsv[i].v != 0)
            {
                pixels_hsv[i].v = (uint8_t)((double)pixels_hsv[i].v * LED_TURINING_OFF_SPEED);
                hsv_to_rgb(pixels_hsv[i].h, pixels_hsv[i].s, pixels_hsv[i].v, &pixels_rgb[i].num);
            }
            else
            {
                pixels_rgb->num = 0;
            }
        }


        /* Update the led strip array with the new pixels */
        for (int i = ledStrip_currentLedIndex; 
            i < ledStrip_currentLedIndex + ledsCntToBeWritten; 
            i++)
        {
            pixels_rgb[i].num = newPixel.num;

            pixels_hsv[i].h = h;
            pixels_hsv[i].s = s;
            pixels_hsv[i].v = v;
        }

        /* Write the led strip */
        ws2812_seq_start();
        ws2812_set_many(PIN_WS2812, pixels_rgb, LEDSTRIP_LED_CNT);
        ws2812_seq_end();

#if defined(DEBUG_ENABLED)
        /* Print debug message  */
        TickType_t currentTickCount = xTaskGetTickCount();
        uint32_t currentMillis = currentTickCount * portTICK_PERIOD_MS;

        char ledsCntToBeWritten_str[MAX_LED_CNT_SWITCHED+1];
        memset(ledsCntToBeWritten_str, 0, sizeof(ledsCntToBeWritten_str));
        for (int i = 0; i < ledsCntToBeWritten; i++)
        {
            ledsCntToBeWritten_str[i] = '|';
        }

        DEBUG_PRINT("T1: %.6d\t%.2X %.2X %.2X\t%.2x %.2x %.2x\t%u \t%u   \t%u   \t%u   \t%s\n", 
                        currentMillis,
                        newPixel.r, newPixel.g, newPixel.b, 
                        h, s, v,
                        ledStrip_currentLedIndex,
                        stdDev, 
                        stdDev_updated,
                        (uint32_t)(sensitivityMultiplier*1000),
                        ledsCntToBeWritten_str);
#endif

        // wait a bit
        // DELAY_MS(10);
    }
}

/**************************************************************************//**
* @brief   Function name
* @param   List of parameters and related description
* @details Detailed description of implemented functionality
*******************************************************************************/
void task_audioIndicator_deviation_1(void *pvParameters)
{
    esp_err_t rsp;
    uint16_t adc_data[ADC_SAMPLES_CNT_VARIANCE];
    ws2812_rgb_t color;
    ws2812_rgb_t pixels[LEDSTRIP_LED_CNT];

    memset(pixels, 0, LEDSTRIP_LED_CNT*sizeof(uint32_t));
    
    /* Init WS2812 pin */
    gpio_set_direction(PIN_WS2812, GPIO_MODE_OUTPUT);

    /* Init random number generation */
    srand(time(NULL));


    while (1)
    {
        uint32_t ledsCntToBeWritten;
        uint8_t ledStrip_currentLedIndex;
        uint8_t h;
        uint8_t v;
        
        /* Sample data via ADC */
        gpio_set_level(PIN_DBG1, 1);
        rsp = adc_read_fast(adc_data, ADC_SAMPLES_CNT_VARIANCE);
        gpio_set_level(PIN_DBG1, 0);
        if (ESP_OK != rsp)
        {
            DEBUG_PRINT("ERROR: ADC Data Sampling\n");
        }

        uint32_t stdDev = (uint32_t)(1000 * calculate_standardDeviation(adc_data, ADC_SAMPLES_CNT_VARIANCE));

        /* Get number of leds to be updated */
        if (stdDev == 0)
        {
            ledsCntToBeWritten = LEDSTRIP_CNT_MAX_SWITCHED;
        }
        else if (stdDev < 2000)
        {
            ledsCntToBeWritten = 1;
        }
        else if (stdDev < 10000)
        {
            ledsCntToBeWritten = 2;
        }
        else
        {
            ledsCntToBeWritten = LEDSTRIP_CNT_MAX_SWITCHED;
        }

        /* Generate random led index */
        ledStrip_currentLedIndex = rand() % LEDSTRIP_LED_CNT;
        /* Update the led index based on the leds update count */
        if (ledStrip_currentLedIndex + ledsCntToBeWritten >= LEDSTRIP_LED_CNT)
        {
            ledStrip_currentLedIndex = LEDSTRIP_LED_CNT - ledsCntToBeWritten - 1;
        }

        /* Get current h */
        h = rand() % 70;

        /* Get current v */
        if (stdDev > (0xff * 50))
        {
            v = 0xff;
        }
        else
        {
            v = (uint8_t)(stdDev/50);
        }

        /* Ge RGB values */
        if (0 == v)
        {
            color.r = 0;
            color.g = 0;
            color.b = 0;
        }
        hsv_to_rgb(h, 0xff, v, &color.num);

        /* Update the pixel array */
        for (int i = ledStrip_currentLedIndex; 
        i < ledStrip_currentLedIndex + ledsCntToBeWritten; 
        i++)
        {
            pixels[i].num = color.num;
        }

        /* Write the led strip */
        ws2812_seq_start();
        ws2812_set_many(PIN_WS2812, pixels, LEDSTRIP_LED_CNT);
        ws2812_seq_end();

#if defined(DEBUG_ENABLED)
        /* Print debug message  */
        TickType_t currentTickCount = xTaskGetTickCount();
        uint32_t currentMillis = currentTickCount * portTICK_PERIOD_MS;

        // DEBUG_PRINT("T1:\t%X\t%X\t%X\t\t%X\t%u\t%d.%d\t%d\n", 
        //                 color.r, color.g, color.b, color.num, ledStrip_currentLedIndex,
        //                 (int)stdDev, (int)(stdDev*1000), currentMillis);

        DEBUG_PRINT("T1: %d\t\t%.2X %.2X %.2X\t\t%u.%u\t\t%d\n", 
                        currentMillis,
                        color.r, color.g, color.b, 
                        (int)stdDev, (int)((int)(stdDev * 100) % 100), 
                        ledStrip_currentLedIndex);

#endif


        // wait a bit
        // DELAY_MS(10);
    

    }
}

/**************************************************************************//**
* @brief   task_ledStrip_rainbow
* @param   None
* @details Task that shows moving rainbow on the strip
*******************************************************************************/
void task_ledStrip_rainbow(void *pvParameters)
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


        DEBUG_PRINT("----------------task_ledStrip_write\n");

        // wait a bit
        DELAY_MS(10);
    }
}

/**************************************************************************//**
* @brief   Function name
* @param   List of parameters and related description
* @details Detailed description of implemented functionality
*******************************************************************************/
void hsv_to_rgb(uint8_t h, uint8_t s, uint8_t v, ws2812_rgb_t *rgb) 
{
    uint8_t region, remainder, p, q, t;

    if (s == 0) {
        // Achromatic (gray)
        rgb->r = rgb->g = rgb->b = v;
        return;
    }

    region = h / 43;
    remainder = (h - (region * 43)) * 6;

    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch (region) {
        case 0:  // Red
            rgb->r = v;
            rgb->g = t;
            rgb->b = p;
            break;
        case 1:  // Yellow
            rgb->r = q;
            rgb->g = v;
            rgb->b = p;
            break;
        case 2:  // Green
            rgb->r = p;
            rgb->g = v;
            rgb->b = t;
            break;
        case 3:  // Cyan
            rgb->r = p;
            rgb->g = q;
            rgb->b = v;
            break;
        case 4:  // Blue
            rgb->r = t;
            rgb->g = p;
            rgb->b = v;
            break;
        default:  // Magenta
            rgb->r = v;
            rgb->g = p;
            rgb->b = q;
            break;
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


#if defined(APP_TYPE_AUDIO_SPECTRUM)
    xTaskCreate(task_01_adc, "task_01_adc", (ADC_SAMPLES_CNT*2) + 25000, NULL, 5, NULL);
#endif

#if defined(APP_TYPE_AUDIO_DEVIATION_1)
    xTaskCreate(task_audioIndicator_deviation_1, "task_audioIndicator_deviation_1", 20*1024, NULL, 5, NULL);
#endif

#if defined(APP_TYPE_AUDIO_DEVIATION_2)
    xTaskCreate(task_audioIndicator_deviation_2, "task_audioIndicator_deviation_2", 20*1024, NULL, 5, NULL);
#endif

#if defined(APP_TYPE_RAINBOW)
    xTaskCreate(task_ledStrip_rainbow, "task_ledStrip_rainbow", 1024, NULL, 5, NULL);
#endif


    // xTaskCreate(task2, "Task 2", configMINIMAL_STACK_SIZE, NULL, 1, &task2Handle);

}
/**********************************************************/


