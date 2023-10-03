/**
  ******************************************************************************
  * @brief   
  ******************************************************************************
**/


/* Includes ------------------------------------------------------------------*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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


/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/



#define PIN_LED_INTEGRATED                   GPIO_NUM_2
#define PIN_DBG1                             GPIO_NUM_5
#define PIN_WS2812                           GPIO_NUM_4

#define LEDSTRIP_LED_CNT                     (60)    // Number of "pixels"

static const char *TAG = "ledStrip_audio";
static const char *TAGN = "NOTE";


/* Private macro -------------------------------------------------------------*/
#define delay_ms(ms) vTaskDelay((ms) / portTICK_PERIOD_MS)





/* Private variables ---------------------------------------------------------*/
QueueHandle_t dataQueue;

/* Private functions ---------------------------------------------------------*/



/**************************************************************************//**
* @brief   Function name
* @param   List of parameters and related description
* @details Detailed description of implemented functionality
*******************************************************************************/
static void task_01_adc()
{
    int x;
    uint16_t adc_data[1000];

    // while (1) 
    // {
    //     if (ESP_OK == adc_read_fast(adc_data, 1000)) 
    //     {
    //         // for (x = 0; x < 100; x++) 
    //         // {
    //         //     printf("%d\n", adc_data[x]);
    //         // }
    //     }

    //     // if (ESP_OK == adc_read(&adc_data[0])) 
    //     // {
    //     //     // ESP_LOGI(TAG, "adc read: %d\r\n", adc_data[0]);
    //     // }


    //     gpio_set_level(PIN_DBG1, 0);
    //     gpio_set_level(PIN_DBG1, 1);

    //     vTaskDelay(1000 / portTICK_RATE_MS);
    // }


    while (1)
    {
        printf("Task_01\n");

        // if (ESP_OK == adc_read(&adc_sample))
        // {
        //     printf("adc read: %u\n", adc_sample);
        // }
        // // ESP_LOGI(TAGN, "Task halted");

        gpio_set_level(PIN_LED_INTEGRATED, 0);
        vTaskDelay(10 / portTICK_RATE_MS);

        gpio_set_level(PIN_LED_INTEGRATED, 1);

        vTaskDelay(1000 / portTICK_RATE_MS);
    }
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

        printf("----------------Task_02\n");

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


        printf("----------------demo\n");

        // wait a bit
        delay_ms(4);
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
    printf("This is ESP8266 chip with %d CPU cores, WiFi, ",
            chip_info.cores);
    printf("silicon revision %d, ", chip_info.revision);
    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    rtc_cpu_freq_t rtcCpuFreq = rtc_clk_cpu_freq_get();
    printf("CLK: ");
    switch (rtcCpuFreq)
    {
        case RTC_CPU_FREQ_80M:
            printf("80");
            break;
        case RTC_CPU_FREQ_160M:
            printf("160");
            break;
        default:
            printf("Unknown");
    }
    printf(" MHz\n");

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
    // xTaskCreate(task_01_adc, "task_01_adc", 4000, NULL, 5, NULL);
    // xTaskCreate(task_02, "task_02", 1024, NULL, 5, NULL);


    xTaskCreate(task_ledStrip_write, "task_ledStrip_write", 1024, NULL, 5, NULL);


    // xTaskCreate(task2, "Task 2", configMINIMAL_STACK_SIZE, NULL, 1, &task2Handle);

}
/**********************************************************/
