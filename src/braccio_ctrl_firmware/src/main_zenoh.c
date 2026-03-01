//
// Copyright (c) 2022 ZettaScale Technology
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   ZettaScale Zenoh Team, <zenoh@zettascale.tech>

#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "esp_err.h"

//////////////////////////////////////
/*  PCA9685 components  */
#include "pca9685_i2c.h"
#include "pca9685_i2c_hal.h"



#define PCA9685_OSC_CLK         25000000
#define SERVO_PWM_FREQ          50

#define SERVO_OUTPUT_PIN_0      0
#define SERVO_OUTPUT_PIN_1      1
#define SERVO_OUTPUT_PIN_2      2
#define SERVO_OUTPUT_PIN_3      3
#define SERVO_OUTPUT_PIN_4      4
#define SERVO_OUTPUT_PIN_5      5
#define SERVO_OUTPUT_PIN_6      6
#define SERVO_OUTPUT_PIN_7      7
#define SERVO_OUTPUT_PIN_8      8
#define SERVO_OUTPUT_PIN_9      9
#define SERVO_OUTPUT_PIN_10     10
#define SERVO_OUTPUT_PIN_11     11
#define SERVO_OUTPUT_PIN_12     12
#define SERVO_OUTPUT_PIN_13     13
#define SERVO_OUTPUT_PIN_14     14
#define SERVO_OUTPUT_PIN_15     15



/*  Select I2C address to be used. ()
    I2C Address options:
    - Default address - 0x40
    - Sub addresses 1-3
    - All call address
*/
#define I2C_DEFAULT_ADDRESS     0
#define I2C_SUB_ADDRESS_1       1
#define I2C_SUB_ADDRESS_2       2
#define I2C_SUB_ADDRESS_3       3
#define I2C_ALLCALL_ADDRESS     4

#define USE_I2C_ADDRESS         I2C_DEFAULT_ADDRESS 

pca9685_dev_t pca9685_1;

//////////////////////////////////////
/*  Zenoh components  */


#include <wifi_connection.h>
#include <zenoh-pico.h>

#define Z_FEATURE_SUBSCRIPTION 1


#define USE_SERIAL 0
#if USE_SERIAL == 1
    #define MODE "peer"
    #define LOCATOR "serial/UART_0#baudrate=115200"
#else
    // #define MODE "client"
    // #define LOCATOR ""  // If empty, it will scout
    #define MODE "client"
    #define LOCATOR "tcp/192.168.1.7:7447"
#endif



#define KEYEXPR "braccio_control/positions"

typedef struct position_motor{
    float rot1;
    float arm1_up;
    float arm2_up;
    float arm3_up;
    float rot_grasp;
    float grasp;
} position_motor;


position_motor ctrl_position_motor;
uint8_t booting = 0;
unsigned long last_time_event = 0;

void data_handler(z_loaned_sample_t* sample, void* arg) {
    booting = 1;
    z_view_string_t keystr;
    z_keyexpr_as_view_string(z_sample_keyexpr(sample), &keystr);
    const z_loaned_bytes_t * payload = z_sample_payload(sample);
    // float ctrl_position_motor;
    // ze_deserialize_float(payload, &ctrl_position_motor);
    // printf("%f\n",ctrl_position_motor);
    ze_deserializer_t data = ze_deserializer_from_bytes(payload);
    ze_deserializer_deserialize_float(&data, &ctrl_position_motor.rot1);
    ze_deserializer_deserialize_float(&data, &ctrl_position_motor.arm1_up);
    ze_deserializer_deserialize_float(&data, &ctrl_position_motor.arm2_up);
    ze_deserializer_deserialize_float(&data, &ctrl_position_motor.arm3_up);
    ze_deserializer_deserialize_float(&data, &ctrl_position_motor.rot_grasp);
    ze_deserializer_deserialize_float(&data, &ctrl_position_motor.grasp);
    printf("msg: %f,%f,%f,%f,%f,%f\n", ctrl_position_motor.rot1,
                                     ctrl_position_motor.arm1_up,
                                     ctrl_position_motor.arm2_up,
                                     ctrl_position_motor.arm3_up,
                                     ctrl_position_motor.rot_grasp,
                                     ctrl_position_motor.grasp);

}

//////////////////////////////////////

/*control function motor*/

static const char *TAG = "control motor";

void servo_pwm_drive(pca9685_dev_t dev, uint8_t servo_no, float d_cycle_ms)
{
    if(pca9685_i2c_led_pwm_set(dev, servo_no, (d_cycle_ms/(1000/(float)SERVO_PWM_FREQ))*100, 0) == PCA9685_OK)
        ESP_LOGI(TAG, "Servo drive: Duty cycle -> %.01fms, Duration -> %.01fms", d_cycle_ms, (1000/(float)SERVO_PWM_FREQ));
    else
        ESP_LOGE(TAG, "Servo drive: ERROR!");
}

//////////////////////////////////////
/* conversion position motor to signal to the driver*/


float conv_ctrl_rot1(float angle){
    const float max_val_in = 90.0;
    const float min_val_in = -90.0;
    if (angle > max_val_in) angle = max_val_in;
    if (angle < min_val_in) angle = min_val_in;
    const float min_val = 0.6;
    const float max_val = 2.2;
    return min_val+angle*(max_val-min_val);
}
float conv_ctrl_arm1_up(float angle){
    const float max_val_in = 90.0;
    const float min_val_in = -38.07;
    if (angle > max_val_in) angle = max_val_in;
    if (angle < min_val_in) angle = min_val_in;
    const float min_val = 0.55;
    const float max_val = 1.4;
    return min_val+angle*(max_val-min_val);
}
float conv_ctrl_arm2_up(float angle){
    const float max_val_in = 90.0;
    const float min_val_in = -90.0;
    if (angle > max_val_in) angle = max_val_in;
    if (angle < min_val_in) angle = min_val_in;
    const float min_val = 0.55;
    const float max_val = 2.0;
    return min_val+angle*(max_val-min_val);
}
float conv_ctrl_arm3_up(float angle){
    const float max_val_in = 90.0;
    const float min_val_in = -90.0;
    if (angle > max_val_in) angle = max_val_in;
    if (angle < min_val_in) angle = min_val_in;
    const float min_val = 0.6;
    const float max_val = 2.2;
    return min_val+angle*(max_val-min_val);
}
float conv_ctrl_rot_grasp(float angle){
    const float max_val_in = 180.0;
    const float min_val_in = 0.0;
    if (angle > max_val_in) angle = max_val_in;
    if (angle < min_val_in) angle = min_val_in;
    const float min_val = 0.6;
    const float max_val = 2.1;
    return min_val+angle*(max_val-min_val);
}
float conv_ctrl_grasp(float open_graps){
    const float max_val_in = 1.0;
    const float min_val_in = 0.0;
    if (open_graps > max_val_in) open_graps = max_val_in;
    if (open_graps < min_val_in) open_graps = min_val_in;
    const float min_val = 0.6;
    const float max_val = 1.2;
    return min_val+open_graps*(max_val-min_val);
}

//////////////////////////////////////

void app_main() {

    // -----------------
    // init communication protocol
    // -----------------
    
#if USE_SERIAL == 1

#else

    printf("Connecting to device...");
    if (init_communication_wifi()==1){
        printf("communication extabilished!\n");
    }else{
        printf("havent init wifi comm!\n");
        exit(-1);
    }
#endif

    // -----------------
    // init hardware control
    // -----------------
    

    esp_err_t err = ESP_OK;

    pca9685_i2c_hal_init(I2C_ADDRESS_PCA9685);

    err += pca9685_i2c_reset(); /* Perform all device reset */
    ESP_LOGI(TAG, "Device reset: %s", err == PCA9685_OK ? "Successful" : "Failed");

    /* Register i2c device*/
    pca9685_i2c_register( &pca9685_1, 
                          I2C_ADDRESS_PCA9685,
                          I2C_ALL_CALL_ADDRESS_PCA9685,
                          I2C_SUB_ADDRESS_1_PCA9685,
                          I2C_SUB_ADDRESS_2_PCA9685,
                          I2C_SUB_ADDRESS_3_PCA9685); /* Use default values */

#if USE_I2C_ADDRESS == I2C_DEFAULT_ADDRESS

    //Already in default I2C address

#elif USE_I2C_ADDRESS == I2C_SUB_ADDRESS_1

    err += pca9685_i2c_sub_addr_resp(pca9685_1, PCA9685_SUB_ADDR_1, PCA9685_ADDR_RESPOND); /* Enable subaddres 1 response*/
    ESP_LOGI(TAG, "Enable subaddress 1: %s", err == PCA9685_OK ? "Successful" : "Failed");
    pca9685_1.i2c_addr =  pca9685_1.sub_addr_1; /* Assign i2c_addr to subaddress 1 */

#elif USE_I2C_ADDRESS == I2C_SUB_ADDRESS_2

    err += pca9685_i2c_sub_addr_resp(pca9685_1, PCA9685_SUB_ADDR_2, PCA9685_ADDR_RESPOND); /* Enable subaddres 2 response*/
    ESP_LOGI(TAG, "Enable subaddress 2: %s", err == PCA9685_OK ? "Successful" : "Failed");
    pca9685_1.i2c_addr =  pca9685_1.sub_addr_2; /* Assign i2c_addr to subaddress 2 */

#elif USE_I2C_ADDRESS == I2C_SUB_ADDRESS_3

    err += pca9685_i2c_sub_addr_resp(pca9685_1, PCA9685_SUB_ADDR_3, PCA9685_ADDR_RESPOND); /* Enable subaddres 3 response*/
    ESP_LOGI(TAG, "Enable subaddress 3: %s", err == PCA9685_OK ? "Successful" : "Failed");
    pca9685_1.i2c_addr =  pca9685_1.sub_addr_3; /* Assign i2c_addr to subaddress 3 */

#elif USE_I2C_ADDRESS == I2C_ALLCALL_ADDRESS

    /* All call address response is already enabled by default */
    pca9685_1.i2c_addr =  pca9685_1.allcall_addr;

#endif

    ESP_LOGI(TAG, "i2c_addr: 0x%02x", pca9685_1.i2c_addr);

    err += pca9685_i2c_write_pre_scale(pca9685_1, SERVO_PWM_FREQ, PCA9685_OSC_CLK); /* Setting frequency to 50 Hz (200ms) */
    ESP_LOGI(TAG, "Frequency Setting: %s", err == PCA9685_OK ? "Successful" : "Failed");

    pca9685_i2c_sleep_mode(pca9685_1, PCA9685_MODE_NORMAL);
    pca9685_i2c_autoincrement(pca9685_1, PCA9685_AUTOINCR_ON); /* Register increment every read/write */

    if (err == ESP_OK){
        ESP_LOGI(TAG, "PCA9685 initialization successful");
    }else{
        ESP_LOGE(TAG, "PCA9685 initialization failed!");
        exit(-1);
    }

    // -----------------
    // Zenoh part
    // -----------------

    // Initialize Zenoh Session and other parameters
    z_owned_config_t config;
    z_config_default(&config);
    zp_config_insert(z_loan_mut(config), Z_CONFIG_MODE_KEY, MODE);
    if (strcmp(LOCATOR, "") != 0) {
        if (strcmp(MODE, "client") == 0) {
            zp_config_insert(z_loan_mut(config), Z_CONFIG_CONNECT_KEY, LOCATOR);
        } else {
            zp_config_insert(z_loan_mut(config), Z_CONFIG_LISTEN_KEY, LOCATOR);
        }
    }

    z_owned_session_t s;

    // Open Zenoh session
    printf("Opening Zenoh Session...");
    if (z_open(&s, z_move(config), NULL) < 0) {
        printf("Unable to open session!\n");
        exit(-1);
    }
    printf("OK\n");

    // Start the receive and the session lease loop for zenoh-pico
    zp_start_read_task(z_loan_mut(s), NULL);
    zp_start_lease_task(z_loan_mut(s), NULL);

    printf("Declaring Subscriber on '%s'...", KEYEXPR);
    z_owned_closure_sample_t callback;
    z_closure(&callback, data_handler, NULL, NULL);
    z_owned_subscriber_t sub;
    z_view_keyexpr_t ke;
    z_view_keyexpr_from_str_unchecked(&ke, KEYEXPR);
    if (z_declare_subscriber(z_loan(s), &sub, z_loan(ke), z_move(callback), NULL) < 0) {
        printf("Unable to declare subscriber.\n");
        exit(-1);
    }
    printf("OK!\n");

    int cnt = 0;

    while (1) {
        if (booting==0){
            printf("waiting at least publisher\n");
            vTaskDelay( 500/*ms*/ / portTICK_PERIOD_MS);
        }else if (booting==3){
            printf("no new data received from a lot of time\n");
            pca9685_i2c_all_led_pwm_set(pca9685_1, 0, 0);
        }
        else{
            servo_pwm_drive(pca9685_1, SERVO_OUTPUT_PIN_0, conv_ctrl_rot1(ctrl_position_motor.rot1));
            servo_pwm_drive(pca9685_1, SERVO_OUTPUT_PIN_1, conv_ctrl_arm1_up(ctrl_position_motor.arm1_up));
            servo_pwm_drive(pca9685_1, SERVO_OUTPUT_PIN_2, conv_ctrl_arm2_up(ctrl_position_motor.arm2_up));
            servo_pwm_drive(pca9685_1, SERVO_OUTPUT_PIN_4, conv_ctrl_arm3_up(ctrl_position_motor.arm3_up));
            servo_pwm_drive(pca9685_1, SERVO_OUTPUT_PIN_5, conv_ctrl_rot_grasp(ctrl_position_motor.rot_grasp));
            servo_pwm_drive(pca9685_1, SERVO_OUTPUT_PIN_6, conv_ctrl_grasp(ctrl_position_motor.grasp));
            pca9685_i2c_hal_ms_delay(200);
            if (booting == 1){
                booting = 2;
                cnt=0;
            }else if (booting == 2){
                cnt+=1;
            }
            if (cnt>10){
                booting = 3;
            }
            
        }
    }

    printf("Closing Zenoh Session...");
    z_drop(z_move(sub));

    z_drop(z_move(s));
    printf("OK!\n");
}