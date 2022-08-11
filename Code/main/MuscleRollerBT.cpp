#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "nvs_flash.h"
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
#include <string>
#include <iostream>
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to s
#define TIMER_INTERVAL1_S 0.00005

#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "ESP_SPP_ACCEPTOR"

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static esp_adc_cal_characteristics_t *adcCharsCharge, *adcCharsEStop;
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_11db;

const gpio_num_t pinDirZ = (gpio_num_t)33;
const gpio_num_t pinStepZ = (gpio_num_t)25;
const gpio_num_t pinMSZ = (gpio_num_t)26;
const gpio_num_t pinEn = (gpio_num_t)32;
const gpio_num_t pinDirX = (gpio_num_t)23;
const gpio_num_t pinStepX = (gpio_num_t)22;
const gpio_num_t pinMSX = (gpio_num_t)15;
const gpio_num_t pinLCClk = (gpio_num_t)27;
const gpio_num_t pinLCDat = (gpio_num_t)14;
const gpio_num_t pinLCClk2 = (gpio_num_t)12;
const gpio_num_t pinLCDat2 = (gpio_num_t)13;
const gpio_num_t pinHomZ = (gpio_num_t)19;
const gpio_num_t pinHomX = (gpio_num_t)21;
const gpio_num_t pinClk = (gpio_num_t)4;
const gpio_num_t pinMOSI = (gpio_num_t)5;
const gpio_num_t pinMISO = (gpio_num_t)16;
const gpio_num_t pinCS0 = (gpio_num_t)17;
const gpio_num_t pinCS1 = (gpio_num_t)18;

const adc1_channel_t pinCharge = ADC1_CHANNEL_0;
const adc2_channel_t pinEStop = ADC2_CHANNEL_2;

//init motors
Stepper motorX(pinStepX, pinDirX, pinHomX, pinCS0, pinMSX, true);
Stepper motorZ(pinStepZ, pinDirZ, pinHomZ, pinCS1, pinMSZ, true);

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

const float LC1Zero = 187200; //near side
const float LC125lb = -392000;
const float LC2Zero = -92093; //far side
const float LC225lb = 479727;
float forceL, forceR;

static bool IRAM_ATTR timer_0_isr(void *para) {
    bool stepX = false;
    bool stepZ = false;
    motorX.intrCount++;
    motorZ.intrCount++;

    //motorX
    if(motorX.intrCount >= motorX.intrInterval) {
        motorX.intrCount = 0;

        if(motorX.bPosError) {
            stepX = true;
        }
    }

    //motorZ
    if(motorZ.intrCount >= motorZ.intrInterval) {
        motorZ.intrCount = 0;

        if(motorZ.bPosMode) { //position mode
            if(motorZ.bPosError) {
                stepZ = true;
            }
        } else { //force mode
            if(motorZ.bForceError) {
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

    return pdFALSE;
}

static bool IRAM_ATTR timer_1_isr(void *para) {
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);
    gpio_set_level(motorX.pinStep, 0);
    gpio_set_level(motorZ.pinStep, 0);
    return pdFALSE;
}

static void tg0_timer_init(timer_idx_t timer_idx,
                                   timer_autoreload_t auto_reload, double timer_interval_ms) {
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = auto_reload,
        .divider = TIMER_DIVIDER,
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
    spi_bus_config_t buscfg={};
    buscfg.mosi_io_num=pinMOSI;
    // buscfg.data0_io_num=-1;
    buscfg.miso_io_num=pinMISO;
    // buscfg.data1_io_num=-1;
    buscfg.sclk_io_num=pinClk;
    buscfg.quadwp_io_num=-1;
    // buscfg.data2_io_num=-1;
    buscfg.quadhd_io_num=-1;
    // buscfg.data3_io_num=-1;
    // buscfg.data4_io_num=-1;
    // buscfg.data5_io_num=-1;
    // buscfg.data6_io_num=-1;
    // buscfg.data7_io_num=-1;
    // buscfg.max_transfer_sz=0;
    // buscfg.flags=0;
    // buscfg.intr_flags=0;

    spi_device_interface_config_t devcfg={};
    devcfg.command_bits = 0;
    devcfg.address_bits = 0;
    // devcfg.dummy_bits = 0;
    devcfg.mode=1;
    // devcfg.duty_cycle_pos = 128;
    // devcfg.cs_ena_pretrans = 0;
    // devcfg.cs_ena_posttrans = 0;
    devcfg.clock_speed_hz=5000000;
    // devcfg.input_delay_ns = 0;
    devcfg.spics_io_num=-1;   //CS pins manually implemented
    // devcfg.flags = 0;
    devcfg.queue_size = 7;

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
        motorZ.force = forceL+forceR;
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
        motorZ.force = forceL+forceR;
        // printf("Raw: %d\n", data);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void runCycle() {
    switch(cycle.step) {
        case 0: //move to start x and start z
        motorX.target = cycle.startX;
        motorX.complete = false;
        motorX.speed = cycle.travelSpeed;

        motorZ.targetForce = 0;
        motorZ.target = cycle.startZ;
        motorZ.complete = false;
        motorZ.speed = cycle.travelSpeed;
        cycle.step = 10;
        break;

        case 10: //wait for move complete
        if(motorX.complete && motorZ.complete) cycle.step = 20;
        break;

        case 20: //apply force
        motorZ.targetForce = cycle.force;
        motorZ.bForceError = true;
        cycle.step = 25;
        break;

        case 25:
        if(!motorZ.bForceError) cycle.step = 30;
        break;

        case 30: // move to end x
        motorX.speed = cycle.speed;
        motorX.target = cycle.endX;
        motorX.complete = false;
        cycle.step = 40;
        break;

        case 40: //wait for move complete
        if(motorX.complete) cycle.step = 50;
        break;

        case 50: // set force to 0 and move to start z
        motorZ.speed = cycle.travelSpeed;
        motorZ.targetForce = 0;
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
    float error = fabs(motorZ.targetForce - motorZ.force);
    float kp = 1.5;
    float speed = kp*error;
    if(speed > 20) speed = 20;
    if(speed < 0.5) speed = 0.5;
    motorZ.speed = speed;
}

static char *bda2str(uint8_t *bda, char *str, size_t size) {
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

void parseData(esp_spp_cb_param_t *param) {
    //create string from data
    std::string data = (char*)param->data_ind.data;

    if (data.substr(0,5) == "movex") {
        motorX.target = std::stof(data.substr(5,3));
        printf("motorX.target = %.0f\n", motorX.target);

    // } else if (uri.substr(0,7) == "/speeds") {
    //     if (buf_len > 1) {
    //         buf = (char*)malloc(buf_len);
    //         if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
    //             char param[32];
    //             /* Get value of expected key from query string */
    //             if (httpd_query_key_value(buf, "x", param, sizeof(param)) == ESP_OK) {
    //                 ESP_LOGI(TAG, "Found URL query parameter => x=%s", param);
    //                 float tempSpeed = atof(param);
    //                 if(tempSpeed < 40) motorX.speed = tempSpeed;
    //                 else motorX.speed = 40;
    //             }
    //             if (httpd_query_key_value(buf, "z", param, sizeof(param)) == ESP_OK) {
    //                 ESP_LOGI(TAG, "Found URL query parameter => z=%s", param);
    //                 motorZ.speed = atof(param);
    //             }
    //         }
    //         free(buf);
    //     }

    // } else if (uri.substr(0,5) == "/home") {
    //     if (buf_len > 1) {
    //         buf = (char*)malloc(buf_len);
    //         if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
    //             char param[32];
    //             /* Get value of expected key from query string */
    //             if (httpd_query_key_value(buf, "axis", param, sizeof(param)) == ESP_OK) {
    //                 ESP_LOGI(TAG, "Found URL query parameter => axis=%s", param);
    //                 if(*param == 'x') motorX.home();
    //                 else if(*param == 'z') {
    //                     motorZ.home();
    //                     motorZ.force = 0;
    //                 }
    //             }
    //         }
    //         free(buf);
    //     }

    // } else if (uri.substr(0,6) == "/force") {
    //     if (buf_len > 1) {
    //         buf = (char*)malloc(buf_len);
    //         if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
    //             char param[32];
    //             /* Get value of expected key from query string */
    //             if (httpd_query_key_value(buf, "f", param, sizeof(param)) == ESP_OK) {
    //                 ESP_LOGI(TAG, "Found URL query parameter => f=%s", param);
    //                 motorZ.targetForce = atof(param);
    //             }
    //         }
    //         free(buf);
    //     }
        
    // } else if (uri.substr(0,6) == "/cycle") {
    //     if (buf_len > 1) {
    //         buf = (char*)malloc(buf_len);
    //         if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
    //             char param[32];
    //             if (httpd_query_key_value(buf, "startX", param, sizeof(param)) == ESP_OK) {
    //                 ESP_LOGI(TAG, "Found URL query parameter => startX=%s", param);
    //                 cycle.startX = atof(param);
    //             }
    //             if (httpd_query_key_value(buf, "startZ", param, sizeof(param)) == ESP_OK) {
    //                 ESP_LOGI(TAG, "Found URL query parameter => startZ=%s", param);
    //                 cycle.startZ = atof(param);
    //             }
    //             if (httpd_query_key_value(buf, "force", param, sizeof(param)) == ESP_OK) {
    //                 ESP_LOGI(TAG, "Found URL query parameter => force=%s", param);
    //                 cycle.force = atof(param);
    //             }
    //             if (httpd_query_key_value(buf, "endX", param, sizeof(param)) == ESP_OK) {
    //                 ESP_LOGI(TAG, "Found URL query parameter => endX=%s", param);
    //                 cycle.endX = atof(param);
    //             }
    //             if (httpd_query_key_value(buf, "travelSpeed", param, sizeof(param)) == ESP_OK) {
    //                 ESP_LOGI(TAG, "Found URL query parameter => travelSpeed=%s", param);
    //                 cycle.travelSpeed = atof(param);
    //             }
    //             if (httpd_query_key_value(buf, "speed", param, sizeof(param)) == ESP_OK) {
    //                 ESP_LOGI(TAG, "Found URL query parameter => speed=%s", param);
    //                 cycle.speed = atof(param);
    //             }
    //             cycle.cycling = true;
    //         }
    //         free(buf);
    //     }

    } else {
        printf("unknown command: ");
        std::cout << data << "\n";
    }
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    char bda_str[18] = {0};

    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
            esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
        }
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%d close_by_remote:%d", param->close.status,
                 param->close.handle, param->close.async);
        break;
    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT handle:%d sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
                     param->start.scn);
            esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
        }
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        /*
         * We only show the data in which the data length is less than 128 here. If you want to print the data and
         * the data rate is high, it is strongly recommended to process them in other lower priority application task
         * rather than in this callback directly. Since the printing takes too much time, it may stuck the Bluetooth
         * stack and also have a effect on the throughput!
         */
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len:%d handle:%d",
                 param->data_ind.len, param->data_ind.handle);
        if (param->data_ind.len < 128) {
            esp_log_buffer_char("", param->data_ind.data, param->data_ind.len);
        }
        parseData(param);
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT status:%d handle:%d, rem_bda:[%s]", param->srv_open.status,
                 param->srv_open.handle, bda2str(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));
        break;
    case ESP_SPP_SRV_STOP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_STOP_EVT");
        break;
    case ESP_SPP_UNINIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
        break;
    default:
        break;
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    char bda_str[18] = {0};

    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s bda:[%s]", param->auth_cmpl.device_name,
                     bda2str(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
        } else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }
    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d bda:[%s]", param->mode_chg.mode,
                 bda2str(param->mode_chg.bda, bda_str, sizeof(bda_str)));
        break;

    default: {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}

void btInit() {
    esp_err_t ret;
    char bda_str[18] = {0};
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    ESP_LOGI(SPP_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
}

extern "C" void app_main(void) {
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    //init bluetooth
    btInit();

    //configure outputs
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pin_bit_mask = 1ULL<<pinDirX|1ULL<<pinStepX|1ULL<<pinMSX|1ULL<<pinEn;
    io_conf.pin_bit_mask |= 1ULL<<pinDirZ|1ULL<<pinStepZ|1ULL<<pinMSZ;
    io_conf.pin_bit_mask |= 1ULL<<pinCS0|1ULL<<pinCS1|1ULL<<pinLCClk|1ULL<<pinLCClk2;
    io_conf.pull_down_en = (gpio_pulldown_t)0;
    io_conf.pull_up_en = (gpio_pullup_t)0;
    gpio_config(&io_conf);

    //configure inputs
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL<<pinHomX|1ULL<<pinHomZ;
    io_conf.pull_down_en = (gpio_pulldown_t)0;
    io_conf.pull_up_en = (gpio_pullup_t)1;
    gpio_config(&io_conf);

    //configure ADCs
    adc1_config_channel_atten(pinCharge, atten);
    adcCharsCharge = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, atten, width, DEFAULT_VREF, adcCharsCharge);
    adc2_config_channel_atten(pinEStop, atten);
    adcCharsEStop = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_2, atten, width, DEFAULT_VREF, adcCharsEStop);

    tg0_timer_init(TIMER_0, TIMER_AUTORELOAD_EN, TIMER_INTERVAL0_S);
    int *arg = 0;
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_0_isr, arg, 0);
    timer_start(TIMER_GROUP_0, TIMER_0);

    tg0_timer_init(TIMER_1, TIMER_AUTORELOAD_DIS, TIMER_INTERVAL1_S);
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_1, timer_1_isr, arg, 0);
    timer_start(TIMER_GROUP_0, TIMER_1);

    xTaskCreate(loadCell, "loadCell", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    xTaskCreate(loadCell2, "loadCell2", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    xTaskCreate(enc_task, "enc_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);

    while(1) {
        vTaskDelay(10/portTICK_RATE_MS);
        timeNow = esp_timer_get_time();

        if(motorZ.targetForce != 0) zSpeed();
        if(cycle.cycling) runCycle();
        if(timeNow - timePrint > 1000000) {
            int raw = adc1_get_raw(pinCharge);
            //Convert adc_reading to voltage in mV
            uint32_t voltage = esp_adc_cal_raw_to_voltage(raw, adcCharsCharge);
            battery = (float)voltage*57.0/10000.0;
            //if voltage is too low, deep sleep indefinitely
            if(battery < 9.6) {
                printf("Low battery: %.2f V\n", battery);
                esp_deep_sleep_start();
            }
            adc2_get_raw(pinEStop, width, &raw);
            // raw = adc1_get_raw(pinEStop);
            //Convert adc_reading to voltage in mV
            voltage = esp_adc_cal_raw_to_voltage(raw, adcCharsEStop);
            float eStop = (float)voltage*57.0/10000.0;
            if(eStop < 5) printf("estopped\n");
            // printf("pinHomX: %i\n", gpio_get_level(motorX.pinHome));
            printf("X %.1f\t Z %.1f\t step %i\n", motorX.position, motorZ.position, cycle.step);
            printf("force %.1f\t FL %.1f\t FR %.1f\n", motorZ.force, forceL, forceR);
            timePrint = timeNow;
        }
    }
}