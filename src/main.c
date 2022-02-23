/****************************************************************************/
/** 
* SenseGuard gmbh
* @file main.c
* @{
* @details
* 
* This file contains the \e main function of the <b> IDEFIX  </b>().
*
* <b> Major Tasks </b> of this file includes

* -# Initialization of the controller UART (UART 0 and UART 1)
* -# Create two rtos task (one for sending the uart data packet to totalizer
*  and second task is used to read the data packet from the totalizer board)
*
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who    Date        Changes
* ----- ------ --------    ---------------------------------------------------------------------
* 1.0   sk    10-02-2022   First version
*
* </pre>
*****************************************************************************/
/** @} */

/***************************** Include Files ********************************/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "main.h"
#include "driver/mcpwm.h"
#include "pressure.h"
#include "Totalizer.h"

/**************************************** macro defination *************************/
#define CONFIG_WIFI_SSID     "WIFI_NAMe"
#define CONFIG_WIFI_PASSWORD "ROUTER_PASSWORD"
#define CONFIG_BROKER_URL    "MQTT_URL"                   //"mqtt://test.mosquitto.org/"
#define I2C_MASTER_SCL_IO           22                        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21                        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_FREQ_HZ          90000                     /*!< I2C master clock frequency */
#define I2C_MASTER_NUM              0  
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

 
/*****************************variable defination****************************/ 
static const char *TAG = "MQTT_EXAMPLE";
static const char *TAG1 = "Sensor_value";
char *motor_status=NULL;
static EventGroupHandle_t wifi_event_group;
const static int CONNECTED_BIT = BIT0;
char data[1], i =0,f=33;
esp_mqtt_client_handle_t client;
uint32_t rawsensordata = 0;

#define DO_MOTOR_DRIVER_ENABLE GPIO_NUM_2
#define DO_MOTOR_DRIVER_FORWARD_PWM GPIO_NUM_26    // 22
#define DO_MOTOR_DRIVER_REVERSE_PWM GPIO_NUM_25    //21

#define DO_RGB_LED_RED  GPIO_NUM_0
#define DO_RGB_LED_GREEN    GPIO_NUM_2
#define DO_RGB_LED_BLUE GPIO_NUM_4
// #define AI_VALVE_CURRENT

#define DEFAULT_VALVE_MOTOR_DUTY 75

uint8_t Moving_A_count =0;

/* valve datatype and sub datatypes */
typedef struct
{
    uint8_t torque_out;
    uint8_t rpm_out;
}gear_data_t;

typedef struct
{
    uint16_t electric_current[20];
    uint8_t buffer_index;
    uint16_t electric_current_avg;
}motor_data_t;

typedef struct
{   
    uint16_t position_current[10];   // in degrees: 0° closed1, 90° open1, 180° closed2, 270° open2
    uint16_t  position_current_average;
    uint16_t position_current_Temp_average;
    char position_requested;  // same as position current
}valve_data_t;

typedef struct
{   
    valve_data_t valve;
    motor_data_t motor;
    gear_data_t gear;
}data_t;

data_t g_data;

typedef struct
{   
    uint8_t data_len;
    uint8_t *data;
}mqtt;

mqtt mqtt_receive;

void L1_ValveInit(void){
    
    /* initialize all valve related functions */
   // g_data.valve.position_current[3] = {0,0,0};
   g_data.valve.position_current[0]=0;
   g_data.valve.position_current[1]=0;
   g_data.valve.position_current[2]=0;
   g_data.valve.position_requested = 0;
    

    /* set valve motor driver enable pin to GPIO output with pull down activated*/
  //  gpio_config_t pValveEnablePin = {
  //      .pin_bit_mask = DO_MOTOR_DRIVER_ENABLE,            // map corresponding pin
  //      .intr_type = GPIO_INTR_DISABLE,             // no interrupt for this pin
  //      .mode = GPIO_MODE_OUTPUT,                   // configured as output
  //      .pull_down_en = GPIO_PULLDOWN_ONLY          // pull-down active
  //  };

  //  gpio_config(&pValveEnablePin);

    /* connect the clock wise line to motor control unit A */
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, DO_MOTOR_DRIVER_FORWARD_PWM);

    /* connect the counter clock wise line to motor control unit B */
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, DO_MOTOR_DRIVER_REVERSE_PWM);

    /* init the motor control driver unit 0 */
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 30000;                           //frequency = 1kHz,
    pwm_config.cmpr_a = 0;                                  //initial duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;                                  //initial duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;             //up counting mode
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings


}

void L1_LedInit(void){
    /* set the on board LEDs of ESP Wrover kit*/
    gpio_config_t pLedRedPin = {
        .pin_bit_mask = DO_RGB_LED_RED,            // map corresponding pin
        .intr_type = GPIO_INTR_DISABLE,             // no interrupt for this pin
        .mode = GPIO_MODE_OUTPUT,                   // configured as output
        .pull_down_en = GPIO_PULLDOWN_ONLY          // pull-down active
    };

    gpio_config(&pLedRedPin);

        gpio_config_t pLedGreenPin = {
        .pin_bit_mask = DO_RGB_LED_GREEN,            // map corresponding pin
        .intr_type = GPIO_INTR_DISABLE,             // no interrupt for this pin
        .mode = GPIO_MODE_OUTPUT,                   // configured as output
        .pull_down_en = GPIO_PULLDOWN_ONLY          // pull-down active
    };

    gpio_config(&pLedGreenPin);

        gpio_config_t pLedBluePin = {
        .pin_bit_mask = DO_RGB_LED_BLUE,            // map corresponding pin
        .intr_type = GPIO_INTR_DISABLE,             // no interrupt for this pin
        .mode = GPIO_MODE_OUTPUT,                   // configured as output
        .pull_down_en = GPIO_PULLDOWN_ONLY          // pull-down active
    };

    gpio_config(&pLedBluePin);
}

void L1_AdcInit(){

    #if CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
    #elif CONFIG_IDF_TARGET_ESP32S2
        if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
            printf("eFuse Two Point: Supported\n");
        } else {
            printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
        }
    #else
    #error "This example is configured for ESP32/ESP32S2."
    #endif

    /* full range conversion of 12 bit */
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));

    /* set attenuation to range of 100mV to 1250mV at GPIO36*/
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_2_5));

}


/*****************************************************************************/
/**
* \brief MQTT event handler function
*
* \description
*   This function handels the MQTT command request such as : 
    1.  MQTT event connected (subscription command is handeled)
    2.  MQTT event data (data from the subscribed topic receives in this section)
*   The subscribed topic is /topic/motorcontrol
* @param      symbol and state
*  
* @return     totalizer error state
*
* @author     SK 
*
* @date       10/02/2022
*
* @note
********************************************************************************/
   
static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    
    static int motor_msgid=0;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            //motor_msgid = esp_mqtt_client_subscribe(client, "/topic/motorcontrol", 0);
            motor_msgid = esp_mqtt_client_subscribe(client, "/topic/motorposition", 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", motor_msgid);
            break;

        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        
            g_data.valve.position_requested = *(event->data);
            g_data.valve.position_requested =  (g_data.valve.position_requested -48)*10;
         
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;

        default:
          //  ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

/*****************************************************************************/
/**
* \brief wifi handler (this function is refered from the ESP official website) 
*
* \description
*   This function handels the wifi event
*
* @param      symbol and state
*  
* @return     totalizer error state
*
* @author     SK 
*
* @date       02/11/2021
*
* @note
********************************************************************************/
static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);

            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

/*****************************************************************************/
/**
* \brief Wifi Initialization function (referred from the ESP official website)
*
* \description
*   1.  This function sets the microcontroller to the station mode
*   2.  configures the harddcode ssid name and password of the router 
*
* @param      None 
*  
* @return     None 
*
* @author     SK 
*
* @date       10/02//2022
*
* @note
********************************************************************************/
static void wifi_init(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_LOGI(TAG, "start the WIFI SSID:[%s]", CONFIG_WIFI_SSID);
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Waiting for wifi");
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
}

/*****************************************************************************/
/**
* \brief I2C Initialization (refered from the ESP official website)
*
* \description
*   The controller is configured as a spi master
*   GPIO 21 is configured as SDA 
*   GPIO 22 is configured as SCL 
*   spi bus clock is configured to 90 kHz

* @param      None
*  
* @return    esp_err_t 
*
* @author     SK 
*
* @date       10/02/2022
*
* @note
********************************************************************************/
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/*****************************************************************************/
/**
* \brief  MQTT initialization (function is refereed from the ESP official website)
*
* \description
*   The function configures the mqtt mosquitto broker URL. 
*   The defualt communicaation port 1883 is configured.
*   MQTT event handler function is configured.
*
* @param      None
*  
* @return     None
*
* @author     SK 
*
* @date       10/02/2022
*
* @note
********************************************************************************/
static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .host = CONFIG_BROKER_URL,
        .port = 1883,
        .event_handle = mqtt_event_handler,
        // .user_context = (void *)your_context
    };

#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();        
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
}


void ADC_Read(void)
{
    
        char  *valveposition;
         valveposition = (char *)malloc(10);
       
        //g_data.valve.position_current_average = (g_data.valve.position_current[0]+g_data.valve.position_current[1]+g_data.valve.position_current[2])/3;
        
        for(Moving_A_count=0;Moving_A_count<3;Moving_A_count++)
        {
            g_data.valve.position_current[Moving_A_count]  = adc1_get_raw(ADC1_CHANNEL_0);
            g_data.valve.position_current_Temp_average += g_data.valve.position_current[Moving_A_count];
            
        }
        g_data.valve.position_current_Temp_average = g_data.valve.position_current_Temp_average /4;
        g_data.valve.position_current_average = (g_data.valve.position_current_Temp_average/ 3000.00)*100; //
        sprintf(valveposition,"%d", g_data.valve.position_current_average);
        esp_mqtt_client_publish(client, "/topic/valveposition", valveposition, 0, 1, 0); 
        free(valveposition);

        #if 0
        if(Moving_A_count > 2)
        {
            g_data.valve.position_current_average = g_data.valve.position_current_Temp_average / 3; 
            g_data.valve.position_current_Temp_average = 0;
            Moving_A_count =0;   
            g_data.valve.position_current_average = (g_data.valve.position_current_average / 3000.00)*100; // convert to percentage
            sprintf(valveposition,"%d", g_data.valve.position_current_average);
           esp_mqtt_client_publish(client, "/topic/valveposition", valveposition, 0, 1, 0); 
           free(valveposition);
        }
        else
        {
            g_data.valve.position_current_Temp_average += g_data.valve.position_current[Moving_A_count];
            Moving_A_count++;
        }
        #endif
}


/*****************************************************************************/
/**
* \brief This is the main funtion of the project
*
* \description
*This function does driver controller Initializations, Initialization Includes:
     1. Wifi
     2. MQTT
     3. I2C
     4. Initialization sequence for NAU7802 external 24 bit ADC.  
*If the External ADC initialization is succesfull then the ADC startconversion 
 command is transmitted to the ADC. Once the conversion complete status is received
 to the controller, the 24 bit data register of ASIC is read and  transmitted to the 
 raspberry pi via MQTT(publish topic "/topic/pressure").
* @param      None
*  
* @return     None 
*
* @author     SK 
*
* @date       10/02/2022
*
* @note
********************************************************************************/
void app_main()
{
   
    char  *pressure, *volume,*subvolume, *flow; 
    uint8_t power_reg_read = 0;
    TOTALIZER_ERROR_STATUS result=ERROR;  // ERROR
    uint32_t count=0;

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    nvs_flash_init();
    uartinit();
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    L1_AdcInit();
    wifi_init();
    mqtt_app_start();
    L1_ValveInit();
    L1_LedInit();

    result = nau7802_Initialzation(); 
    if(result==PASS)
    {
        adc_start_conversion();     // testing
        while(1)
    {         
        pressure = (char *)malloc(10); 
        subvolume = (char *)malloc(10);
        volume = (char *)malloc(10);
        flow = (char *)malloc(10);

        sensor_data.rawpressuresensor = nau7802_loadsensor_value();
        sensor_data.pressure_millibar = pressure_adc_milli_bar_from_raw(sensor_data.rawpressuresensor);
        sprintf(pressure,"%d", sensor_data.pressure_millibar);
        esp_mqtt_client_publish(client, "/topic/pressure", pressure, 0, 1, 0); 

         L2_totalizer_get_data(&sensor_data.volume, 0xB0);        
         sprintf(volume,"%d", sensor_data.volume);
         esp_mqtt_client_publish(client, "/topic/volume", volume, 0, 1, 0); 

        L2_totalizer_get_data(&sensor_data.subvolume, 0xB1);
        sprintf(subvolume,"%d", sensor_data.subvolume);
        esp_mqtt_client_publish(client, "/topic/subvolume", subvolume, 0, 1, 0); 

        L2_totalizer_get_data(&sensor_data.flow, 0xB2);
        sprintf(flow,"%d", sensor_data.flow);
        esp_mqtt_client_publish(client, "/topic/flow", flow, 0, 1, 0); 

        free(pressure);
        free(subvolume);
        free(volume);
        free(flow);

        motor_valve_control();        
    }

    }
    else
    {
        //no operation
    }
    
    #if 0
    while(1)
    {
        result = nau7802_Initialzation();  //testing
        
        if (result == PASS)
        {
           // ADC_Read();
            adc_start_conversion();     // testing
            while(1)
            {
                motor_valve_control();

           
            nau7802_register_read(NAU7802_PU_CTRL,&power_reg_read,1);
            if((power_reg_read & 0b00100000)>>NAU7802_PU_CTRL_CR == 1)
            {
               
           sensor_data.rawpressuresensor = nau7802_loadsensor_value();
           sensor_data.pressure_millibar = pressure_adc_milli_bar_from_raw(sensor_data.rawpressuresensor);
           //ESP_LOGI(TAG1, "pressure value %d ", sensor_data.pressure_millibar);   
          // if (count >= )
          // {
          //    count =0;
                sprintf(pressure,"%d", sensor_data.pressure_millibar);
                esp_mqtt_client_publish(client, "/topic/pressure", pressure, 0, 1, 0); 
                L2_totalizer_get_data(&sensor_data.volume, 0xB0);
                
                sprintf(volume,"%d", sensor_data.volume);
                esp_mqtt_client_publish(client, "/topic/volume", volume, 0, 1, 0); 

                L2_totalizer_get_data(&sensor_data.subvolume, 0xB1);
           
               sprintf(subvolume,"%d", sensor_data.subvolume);
               esp_mqtt_client_publish(client, "/topic/subvolume", subvolume, 0, 1, 0); 

                L2_totalizer_get_data(&sensor_data.flow, 0xB2);
                sprintf(flow,"%d", sensor_data.flow);
                esp_mqtt_client_publish(client, "/topic/flow", flow, 0, 1, 0); 
           }
          // else
          // {
          //      count++;
          // }
               
            //}
            else 
            {
             //ESP_LOGI(TAG1, "ADC converted value is not ready");
            }
                free(pressure);
                free(volume);
                free(subvolume);
                free(flow);

              
            }   
        }
        else
        {
            ESP_LOGI(TAG1, "Initialization error");
        }
    }    
    #endif
}

void motor_valve_control(void)
{
    ADC_Read();
    //gpio_set_level(DO_RGB_LED_GREEN, 1);

    /* check if valve position to be changed */
        if (g_data.valve.position_requested == g_data.valve.position_current_average)
        {
            /* disable pwm generation of motor controller */
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
            
            /* deactivate motor driver */
            gpio_set_level(DO_RGB_LED_GREEN, 0);
            
            /* check again in 100 milli seconds if still blocked */
            //vTaskDelay(pdMS_TO_TICKS(100));
        }

        /* check if the valve shall rotate clock wise */
        if (g_data.valve.position_requested < g_data.valve.position_current_average)
        {
            /* enable motor driver */
            gpio_set_level(DO_RGB_LED_GREEN, 1);

            /* motor turns in forward direction */
            mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, DEFAULT_VALVE_MOTOR_DUTY);
            mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
           // vTaskDelay(pdMS_TO_TICKS(10));
        }

        /* check if the valve shall be tured counter clock wise */
        if (g_data.valve.position_requested > g_data.valve.position_current_average)
        {
            /* enable motor driver */
            gpio_set_level(DO_RGB_LED_GREEN, 1);

            /* motor turns in reverse direction */
            mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, DEFAULT_VALVE_MOTOR_DUTY);
            mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
           // vTaskDelay(pdMS_TO_TICKS(10));
        }

}


/*****************************************************************************/
/**
* \brief This function initialize the UART.
*
* \description
* Two UART's of the ESP32 are initialized UART 0 is for debug console and UART 1
  is used for the communication between Totalizer board and esp32
* UART 0 baud rate : 115200
* UART 1 baud rate : 38400
* @param    None
*  
* @return   None
*
* @author   SK
*
* @date     02-11-2021
*
* @note
********************************************************************************/


void uartinit(void)
{
    uint8_t intr_alloc_flags = 0;

    /* UART0 */
    uart_config_t debug_uart_config = {
        .baud_rate = DEBUG_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(DEBUG_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(DEBUG_UART_PORT_NUM, &debug_uart_config));
    ESP_ERROR_CHECK(uart_set_pin(DEBUG_UART_PORT_NUM, 1, 3, ECHO_TEST_RTS, ECHO_TEST_CTS));  

   // printf("UART0 initialized");

    /* UART1 */
    uart_config_t mid_uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &mid_uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, 19, 18, ECHO_TEST_RTS, ECHO_TEST_CTS));  //10,9

    //printf("UART1 initialized");
    
    #if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
    #endif
}
