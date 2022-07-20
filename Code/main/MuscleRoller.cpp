#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>
#include "stepper.h"
#include "driver/timer.h"
#include "driver/spi_master.h"
#include <hx711.h>
#include "stepper.h"

#define WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to s
#define TIMER_INTERVAL0_S    0.0002 // sample test interval for the first timer
#define TIMER_INTERVAL1_S 0.00005
static EventGroupHandle_t wifiEventGroup;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int retryNum = 0;
static const char *TAG = "wifi station";   
static esp_adc_cal_characteristics_t *adcCharsCharge, *adcCharsEStop;
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_11db;

const int pinDirZ = 33;
const int pinStepZ = 25;
const int pinMSZ = 26;
const int pinEn = 32;
const int pinDirX = 39;
const int pinStepX = 34;
const int pinMSX = 35;
const int pinLCClk = 27;
const int pinLCDat = 14;
const int pinLCClk2 = 12;
const int pinLCDat2 = 13;
const int pinHomZ = 19;
const int pinHomX = 21;
const int pinClk = 4;
const int pinMOSI = 5;
const int pinMISO = 16;
const int pinCS0 = 17;
const int pinCS1 = 18;

const int pinCharge = ADC1_CHANNEL_0;
const int pinEStop = ADC2_CHANNEL_3;

//init motors
Stepper motorX(pinStepX, pinDirX, pinHomX, pinCS0, pinMSX);
Stepper motorZ(pinStepZ, pinDirZ, pinHomZ, pinCS1, pinMSZ);

float battery;

struct cycle {
    bool cycling;
    int step;
    float force;
    float startX;
    float endX;
    float startZ;
    float travelSpeed;
    float speed;
};

struct cycle cycle;

int64_t timeNow;
int64_t timePrint = 0;

const float LC1Zero = 39600;
const float LC125lb = 627188;
const float LC2Zero = 44318;
const float LC225lb = 627140;
float forceL, forceR;
float targetForce = 0, currentForce = 0;
const float forceTolerance = 0.2;
float kp = 2;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retryNum < CONFIG_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            retryNum++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(wifiEventGroup, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        retryNum = 0;
        xEventGroupSetBits(wifiEventGroup, WIFI_CONNECTED_BIT);
    }
}

void wifiInit(void) {
    wifiEventGroup = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
         .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    //wait for event connected or failed
    EventBits_t bits = xEventGroupWaitBits(wifiEventGroup,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(wifiEventGroup);
}

static esp_err_t get_handler(httpd_req_t *req) {
    extern const unsigned char index_html_start[] asm("_binary_index_html_start");
    extern const unsigned char index_html_end[]   asm("_binary_index_html_end");
    const size_t index_html_size = (index_html_end - index_html_start);
    httpd_resp_send_chunk(req, (const char *)index_html_start, index_html_size);
    /* Send empty chunk to signal HTTP response completion */
    httpd_resp_sendstr_chunk(req, NULL);

    return ESP_OK;
}

static esp_err_t get_handler_2(httpd_req_t *req) {
    char uri8[9];
    strncpy(uri8, req->uri, 8);
    uri8[8] = 0;

    if (!strcmp(req->uri,"/battery")) {
        //return the battery voltage
        int len = snprintf(NULL, 0, "%.2f", battery);
        char *result = (char *)malloc(len + 1);
        snprintf(result, len + 1, "%.2f", battery);
        httpd_resp_sendstr(req, result);

    } else if (!strcmp(req->uri,"/pos")) {
        int len = snprintf(NULL, 0, "X: %.2f\t Z: %.2f\t F: %.1f", 
            motorX.currentPos, motorZ.currentPos, currentForce);
        char *result = (char *)malloc(len + 1);
        snprintf(result, len + 1, "X: %.2f\t Z: %.2f\t F: %.1f", 
            motorX.currentPos, motorZ.currentPos, currentForce);
        httpd_resp_sendstr(req, result);

    } else if (!strcmp(req->uri,"/stop")) {
        motorX.targetSteps = motorX.currentSteps;
        motorZ.targetSteps = motorZ.currentSteps;
        cycle.cycling = false;

    } else if (!strcmp(uri8, "/moveAbs")) {
        printf("moveAbs\n");
    
    } else {
        printf("URI: ");
        printf(req->uri);
        printf("\n");
    }
    return ESP_OK;
}

static httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {

        httpd_uri_t get_uri_1 = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &get_uri_1);

        httpd_uri_t file_download = {
            .uri       = "/*",
            .method    = HTTP_GET,
            .handler   = get_handler_2,
            .user_ctx  = NULL   // Pass server data as context
        };
        httpd_register_uri_handler(server, &file_download);

        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static void stop_webserver(httpd_handle_t server) {
    // Stop the httpd server
    httpd_stop(server);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base, 
                               int32_t event_id, void* event_data) {
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        stop_webserver(*server);
        *server = NULL;
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base, 
                            int32_t event_id, void* event_data) {
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}

void IRAM_ATTR timer_0_isr(void *para) {
    timer_spinlock_take(TIMER_GROUP_0);
    bool stepX = false;
    bool stepZ = false;
    motorX.intrCount++;
    motorZ.intrCount++;

    //motorX
    if(motorX.intrCount >= motorX.intrInterval) {
        motorX.intrCount = 0;

        if(motorX.error > 0.25) {
            stepX = true;
        } else {
            motorX.complete = true;
        }
    }

    //motorZ
    if(motorZ.intrCount >= motorZ.speedCount) {
        motorZ.intrCount = 0;
        motorZ.currentPos = motorZ.currentSteps/motorZ.stepsPer_mm;

        if(targetForce == 0) { //position mode
            motorZ.stepsToTarget = motorZ.targetSteps - motorZ.currentSteps;

            if(motorZ.stepsToTarget > 0) {
                gpio_set_level(motorZ.pinDir, 1);
                motorZ.currentSteps++;
            } else if (motorZ.stepsToTarget < 0) {
                gpio_set_level(motorZ.pinDir, 0);
                motorZ.currentSteps--;
            }
            if(motorZ.stepsToTarget != 0) {
                stepZ = true;
            } else {
                motorZ.complete = true;
            }

        } else { //force mode
            if(currentForce > targetForce + forceTolerance) {
                gpio_set_level(motorZ.pinDir, 1);
                motorZ.currentSteps++;
                stepZ = true;
            } else if (currentForce < targetForce - forceTolerance) {
                gpio_set_level(motorZ.pinDir, 0);
                motorZ.currentSteps--;
                stepZ = true;
            }
        }

    }

    if(stepX) gpio_set_level(motorX.pinStep, 1);
    if(stepZ) gpio_set_level(motorZ.pinStep, 1);
    if(stepX | stepZ) {
        //set timer 1 to reset step pins
        uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, TIMER_1);
        timer_counter_value += (uint64_t) (TIMER_INTERVAL1_S * TIMER_SCALE);
        timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, TIMER_1, timer_counter_value);
        timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_1);
    }

    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_spinlock_give(TIMER_GROUP_0);
}

void IRAM_ATTR timer_1_isr(void *para) {
    timer_spinlock_take(TIMER_GROUP_0);
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);
    gpio_set_level(motorX.pinStep, 0);
    gpio_set_level(motorZ.pinStep, 0);
    timer_spinlock_give(TIMER_GROUP_0);
}

static void tg0_timer_init(int timer_idx,
                                   bool auto_reload, double timer_interval_ms) {
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_ms * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
}

static void enc_task(void *arg) {
    //set CS pins high to start
    gpio_set_level(pinCS0, 1);
    gpio_set_level(pinCS1, 1);

    //create SPI
    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .miso_io_num=pinMISO,
        .mosi_io_num=pinMOSI,
        .sclk_io_num=pinClk,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    spi_device_interface_config_t devcfg={
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode=1,
        .duty_cycle_pos = 128,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz=5000000,
        .input_delay_ns = 0,
        .spics_io_num=-1,   //CS pins manually implemented
        .flags = 0,
        .queue_size = 7,
        .pre_cb = void,
        .post_cb = void
    };
    esp_err_t ret;
    ret=spi_bus_initialize(SPI2_HOST, &buscfg, 0);
    ESP_ERROR_CHECK(ret);
    ret=spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    while(1) {
        motorX.update(spi);
        motorZ.update(spi);
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}

void loadCell(void *pvParameters) {
    hx711_t dev = {
        .dout = (gpio_num_t)pinLCDat,
        .pd_sck = (gpio_num_t)pinLCClk,
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

        forceL = 25*(data-LC1Zero)/LC125lb;
        currentForce = forceL+forceR;
        // printf("Raw: %d\n", data);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void loadCell2(void *pvParameters) {
    hx711_t dev = {
        .dout = (gpio_num_t)pinLCDat2,
        .pd_sck = (gpio_num_t)pinLCClk2,
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

        forceR =  25*(data-LC2Zero)/LC225lb;
        currentForce = forceL+forceR;
        // printf("Raw: %d\n", data);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool forceOk() {
    if(currentForce > targetForce - forceTolerance) {
        if(currentForce < targetForce + forceTolerance) return true;
    }
    return false;
}

void runCycle() {
    switch(cycle.step) {
        case 0: //move to start x and start z
        targetForce = 0;
        motorX.target = cycle.startX;
        motorX.complete = false;
        motorX.intrInterval = 1.0/(cycle.travelSpeed*motorX.stepsPer_mm*TIMER_INTERVAL0_S);

        targetForce = 0;
        motorZ.target = cycle.startZ;
        motorZ.complete = false;
        motorZ.intrInterval = 1.0/(cycle.travelSpeed*motorZ.stepsPer_mm*TIMER_INTERVAL0_S);

        cycle.step = 10;
        break;

        case 10: //wait for move complete
        if(motorX.complete && motorZ.complete) cycle.step = 20;
        break;

        case 20: //apply force
        targetForce = cycle.force;
        if(forceOk()) cycle.step = 30;
        break;

        case 30: // move to end x
        motorX.intrInterval = 1.0/(cycle.speed*motorX.stepsPer_mm*TIMER_INTERVAL0_S);
        motorX.target = cycle.endX;
        motorX.complete = false;
        cycle.step = 40;
        break;

        case 40: //wait for move complete
        if(motorX.complete) cycle.step = 50;
        break;

        case 50: // set force to 0 and move to start z
        motorZ.intrInterval = 1.0/(cycle.travelSpeed*motorZ.stepsPer_mm*TIMER_INTERVAL0_S);
        targetForce = 0;
        motorZ.target = cycle.startZ;
        motorZ.complete = false;
        cycle.step = 60;
        break;

        case 60: //wait for move complete
        if(motorZ.complete) cycle.step = 0;
        break;
    }
}

void zSpeed() {
    float error = fabs(targetForce - currentForce);
    float speed = kp*error;
    if(speed > 20) speed = 20;
    if(speed < 0.5) speed = 0.5;
    double stepsPer_mm = 200.0/8.0*motorZ.microstep;
    motorZ.intrInterval = 1.0/(speed*stepsPer_mm*TIMER_INTERVAL0_S);
}

extern "C" void app_main(void) {
    //connect to WiFi
    wifiInit();
    static httpd_handle_t server = NULL;

    //Register event handlers to stop the server when Wi-Fi is disconnected
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
    server = start_webserver();

    //configure outputs
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = 1ULL<<pinDirX|1ULL<<pinStepX|1ULL<<pinMSX|1ULL<<pinEn;
    io_conf.pin_bit_mask |= 1ULL<<pinDirZ|1ULL<<pinStepZ|1ULL<<pinMSZ;
    //disable pull-down mode
    io_conf.pull_down_en = (gpio_pulldown_t)0;
    //disable pull-up mode
    io_conf.pull_up_en = (gpio_pullup_t)0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //configure inputs
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL<<pinHomX|1ULL<<pinHomZ;
    io_conf.pull_down_en = (gpio_pulldown_t)0;
    io_conf.pull_up_en = (gpio_pullup_t)1;
    gpio_config(&io_conf);

    //configure ADCs
    adc1_config_channel_atten((adc1_channel_t)pinCharge, atten);
    adcCharsCharge = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, atten, width, DEFAULT_VREF, adcCharsCharge);
    adc2_config_channel_atten((adc2_channel_t)pinEStop, atten);
    adcCharsEStop = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_2, atten, width, DEFAULT_VREF, adcCharsEStop);

    tg0_timer_init(TIMER_0, (timer_idx_t)1, TIMER_INTERVAL0_S);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_0_isr,
       (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TIMER_GROUP_0, TIMER_0);

    tg0_timer_init(TIMER_1, (timer_idx_t)0, TIMER_INTERVAL1_S);
    timer_isr_register(TIMER_GROUP_0, TIMER_1, timer_1_isr,
       (void *) TIMER_1, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TIMER_GROUP_0, TIMER_1);

    //create load cell tasks
    xTaskCreate(loadCell, "loadCell", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    xTaskCreate(loadCell2, "loadCell2", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    //create encoder task
    xTaskCreate(enc_task, "enc_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);

    while(1) {
        vTaskDelay(10/portTICK_RATE_MS);
        timeNow = esp_timer_get_time();

        if(targetForce != 0) zSpeed();
        if(cycle.cycling) runCycle();
        if(timeNow - timePrint > 1000000) {
            uint32_t adc_reading = 0;
            int raw = adc1_get_raw((adc1_channel_t)pinCharge);
            //Convert adc_reading to voltage in mV
            uint32_t voltage = esp_adc_cal_raw_to_voltage(raw, adcCharsCharge);
            battery = (float)voltage*57.0/10000.0;
            //if voltage is too low, deep sleep indefinitely
            if(battery < 9.6) {
                printf("Low battery: %.2f V\n", battery);
                esp_deep_sleep_start();
            }
            adc2_get_raw((adc2_channel_t)pinEStop, width, &raw);
            //Convert adc_reading to voltage in mV
            voltage = esp_adc_cal_raw_to_voltage(adc_reading, adcCharsEStop);
            float eStop = (float)voltage*57.0/10000.0;
            printf("eStop voltage: %.3f\n", eStop);
            // printf("X %.1f\t Z %.1f\n", motorX.currentPos, motorZ.currentPos);
            printf("currectForce %.1f\t FL %.1f\t FR %.1f\n", currentForce, forceL, forceR);
            timePrint = timeNow;
        }
    }
}