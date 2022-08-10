#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <hx711.h>

const gpio_num_t pinLCClk = (gpio_num_t)27;
const gpio_num_t pinLCDat = (gpio_num_t)14;
const gpio_num_t pinLCClk2 = (gpio_num_t)12;
const gpio_num_t pinLCDat2 = (gpio_num_t)13;

int64_t timeNow;
int64_t timePrint = 0;

void loadCell(void *pvParameters) {
    hx711_t dev = {
        .dout = pinLCDat,
        .pd_sck = pinLCClk,
        .gain = HX711_GAIN_A_64
    };

    // initialize device
    while (1)
    {
        esp_err_t r = hx711_init(&dev);
        if (r == ESP_OK)
            break;
        printf("Could not initialize HX711: %d (%s)\n", r, esp_err_to_name(r));
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // read from device
    while (1)
    {
        esp_err_t r = hx711_wait(&dev, 500);
        if (r != ESP_OK)
        {
            printf("Device not found: %d (%s)\n", r, esp_err_to_name(r));
            continue;
        }

        int32_t data;
        r = hx711_read_data(&dev, &data);
        if (r != ESP_OK)
        {
            printf("Could not read data: %d (%s)\n", r, esp_err_to_name(r));
            continue;
        }

        printf("Raw 1: %d\n", data);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void loadCell2(void *pvParameters) {
    hx711_t dev = {
        .dout = pinLCDat2,
        .pd_sck = pinLCClk2,
        .gain = HX711_GAIN_A_64
    };

    // initialize device
    while (1)
    {
        esp_err_t r = hx711_init(&dev);
        if (r == ESP_OK)
            break;
        printf("Could not initialize HX711: %d (%s)\n", r, esp_err_to_name(r));
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // read from device
    while (1)
    {
        esp_err_t r = hx711_wait(&dev, 500);
        if (r != ESP_OK)
        {
            printf("Device not found: %d (%s)\n", r, esp_err_to_name(r));
            continue;
        }

        int32_t data;
        r = hx711_read_data(&dev, &data);
        if (r != ESP_OK)
        {
            printf("Could not read data: %d (%s)\n", r, esp_err_to_name(r));
            continue;
        }

        printf("Raw 2: %d\n", data);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main(void) {
    //configure outputs
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = 1ULL<<pinLCClk|1ULL<<pinLCClk2;
    //disable pull-down mode
    io_conf.pull_down_en = (gpio_pulldown_t)0;
    //disable pull-up mode
    io_conf.pull_up_en = (gpio_pullup_t)0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //create load cell tasks
    xTaskCreate(loadCell, "loadCell", configMINIMAL_STACK_SIZE * 5, NULL, 2, NULL);
    xTaskCreate(loadCell2, "loadCell2", configMINIMAL_STACK_SIZE * 5, NULL, 1, NULL);

    while(1) {
        vTaskDelay(10/portTICK_RATE_MS);
        timeNow = esp_timer_get_time();

        if(timeNow - timePrint > 1000000) {
            timePrint = timeNow;
        }
    }
}