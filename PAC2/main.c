/*
 * Copyright (C) 2017 Universitat Oberta de Catalunya - http://www.uoc.edu/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Universitat Oberta de Catalunya nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*----------------------------------------------------------------------------*/

/* Standard includes */
#include <stdlib.h>
#include <stdio.h>


/* Free-RTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "portmacro.h"


/* MSP432 drivers includes */
#include "msp432_launchpad_board.h"
#include "edu_boosterpack_accelerometer.h"
#include "uart_driver.h"


/*----------------------------------------------------------------------------*/

#define TASK_PRIORITY      ( tskIDLE_PRIORITY + 2 )
#define HEARTBEAT_TASK_PRIORITY     ( tskIDLE_PRIORITY + 1 )

#define TASK_STACK_SIZE         ( 1024 )
#define HEARTBEAT_STACK_SIZE        ( 128 )

#define HEART_BEAT_ON_MS            ( 10 )
#define HEART_BEAT_OFF_MS           ( 990 )
#define TASK_DELAY_MS               ( 100 )

#define QUEUE_SIZE                  ( 15 )
#define MEAN_VALUES                 ( 10.0 )
#define TX_UART_MESSAGE_LENGTH      ( 50 )

/*----------------------------------------------------------------------------*/

// struct to store acceleration values
typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} acc_t;


// Tasks
static void HeartBeatTask(void *pvParameters);
static void ADCReadTask(void *pvParameters);
static void ProcessingTask(void *pvParameters);

//Task sync tools and variables
SemaphoreHandle_t xButtonSemaphore;
SemaphoreHandle_t xADCReadingAvailableSemaphore;
QueueHandle_t xQueue;

acc_t accel_values;



/*----------------------------------------------------------------------------*/

static void HeartBeatTask(void *pvParameters){
    for(;;){
        led_toggle(MSP432_LAUNCHPAD_LED_RED);
        vTaskDelay( pdMS_TO_TICKS(HEART_BEAT_ON_MS) );
        led_toggle(MSP432_LAUNCHPAD_LED_RED);
        vTaskDelay( pdMS_TO_TICKS(HEART_BEAT_OFF_MS) );
    }
}

static void ADCReadTask(void *pvParameters) {
    for(;;){
        if( xSemaphoreTake( xButtonSemaphore, portMAX_DELAY ) == pdTRUE ){
            edu_boosterpack_accelerometer_read();
            if( xSemaphoreTake( xADCReadingAvailableSemaphore, portMAX_DELAY ) == pdTRUE ){
                xQueueSend(xQueue, &accel_values, portMAX_DELAY);
            }
        }
    }
}

static void ProcessingTask(void *pvParameters) {
    acc_t new_accel_value;
    float mean_acc_x = 0;
    float mean_acc_y = 0;
    float mean_acc_z = 0;
    char message[TX_UART_MESSAGE_LENGTH];

    for(;;){
        if( xQueueReceive( xQueue, &new_accel_value, portMAX_DELAY ) == pdPASS){
            mean_acc_x = (float)new_accel_value.x * (1/MEAN_VALUES) + mean_acc_x * ((MEAN_VALUES-1)/MEAN_VALUES);
            mean_acc_y = (float)new_accel_value.y * (1/MEAN_VALUES) + mean_acc_y * ((MEAN_VALUES-1)/MEAN_VALUES);
            mean_acc_z = (float)new_accel_value.z * (1/MEAN_VALUES) + mean_acc_z * ((MEAN_VALUES-1)/MEAN_VALUES);
            /* send via UART */
            sprintf(message, "X-accel: %.1f, Y-accel: %.1f, Z-accel: %.1f \n\r", mean_acc_x, mean_acc_y, mean_acc_z);
            uart_print(message);
        }
    }
}

void button_S1_callback(void) {
    xSemaphoreGiveFromISR( xButtonSemaphore, NULL);
}

void get_adc_results (adc_result results_buffer ){
    accel_values.x = results_buffer[0];
    accel_values.y = results_buffer[1];
    accel_values.z = results_buffer[2];
    xSemaphoreGiveFromISR( xADCReadingAvailableSemaphore, NULL);
}

/*----------------------------------------------------------------------------*/

int main(int argc, char** argv)
{
    int32_t retVal = -1;

    // Initialize semaphores and queue
    xButtonSemaphore = xSemaphoreCreateBinary ();
    xADCReadingAvailableSemaphore =  xSemaphoreCreateBinary ();
    xQueue = xQueueCreate( QUEUE_SIZE, sizeof( acc_t ) );

    /* Initialize the board */
    board_init();

    /* Initialize accelerometer-ADC */
    edu_boosterpack_accelerometer_init();

    /* Initialize UART */
    uart_init(NULL);

    /* Configure interrupt callbacks*/
    board_buttons_set_callback(MSP432_LAUNCHPAD_BUTTON_S1, button_S1_callback);
    edu_boosterpack_accelerometer_set_callback(get_adc_results);


    if ( (xButtonSemaphore != NULL) && (xADCReadingAvailableSemaphore != NULL) && (xQueue != NULL)) {

        /* Create HeartBeat task */
        retVal = xTaskCreate(HeartBeatTask, "HeartBeatTask", HEARTBEAT_STACK_SIZE, NULL, HEARTBEAT_TASK_PRIORITY, NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        /* Create ADCread task */
        retVal = xTaskCreate(ADCReadTask, "ADCReadTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        /* Create processing task */
        retVal = xTaskCreate(ProcessingTask, "ProcessingTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        /* Start the task scheduler */
        vTaskStartScheduler();
    }

    return 0;
}

/*----------------------------------------------------------------------------*/

