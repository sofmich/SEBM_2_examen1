/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    MK66F_examen1.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK66F18.h"
#include "fsl_debug_console.h"





/* TODO: insert other include files here. */
#include "semphr.h"
#include "task.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"
/* TODO: insert other definitions and declarations here. */
void initial_task(void *pvParameters);
void first_task(void *pvParameters);
void second_task(void *pvParameters);
void third_task(void *pvParameters);

#define NUM_TASKS 	4
#define FIRST_TASK_EVENT 1<<0
#define SECOND_TASK_EVENT 1<<1
#define SENDER_ONE	1
#define SENDER_TWO 	2

/*Semaphores mutex for give and take the buffer space message*/
SemaphoreHandle_t send_msg_mutex;
SemaphoreHandle_t take_msg_mutex;
QueueHandle_t msg_buffer;
EventGroupHandle_t msg_event;
typedef uint8_t sender;
sender sender_number;

typedef struct {
	const char *msg;
} message_t;


/*
 * @brief   Application entry point.
 */
int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    PRINTF("Examen 1\n\r ");


    while(!xTaskCreate(initial_task, "initial_task", configMINIMAL_STACK_SIZE+100, NULL, NUM_TASKS, NULL))
    {
    	PRINTF("Failed to create Task \n\r");
    }

    vTaskStartScheduler();
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {

    }
    return 0 ;
}

void initial_task(void *pvParameters)
{
	msg_buffer = xQueueCreate(2, sizeof(message_t));
	msg_event = xEventGroupCreate();
	send_msg_mutex = xSemaphoreCreateMutex();
	take_msg_mutex = xSemaphoreCreateMutex();
	while(!xTaskCreate(first_task, "send_msg1", configMINIMAL_STACK_SIZE+100, NULL, NUM_TASKS -1, NULL))
	{
		PRINTF("Failed to create Task 1 \n\r");
	}
	while(!xTaskCreate(second_task, "send_msg2", configMINIMAL_STACK_SIZE+100, NULL, NUM_TASKS -1 , NULL))
	{
		PRINTF("Failed to create Task 2\n\r");
	}
	while(!xTaskCreate(third_task, "receive_msg", configMINIMAL_STACK_SIZE+100, NULL, NUM_TASKS -1, NULL))
	{
		PRINTF("Failed to create Task 3\n\r");
	}
	vTaskSuspend(NULL);
}

void first_task(void *pvParameters)
{
	/*Send message*/
	const char msg_1[] = "First task\n\r";
	message_t buffer_msg;
	buffer_msg.msg = msg_1;
	/*Block status*/
	const TickType_t delay = pdMS_TO_TICKS(150U);
	sender_number = SENDER_ONE;
	for(;;)
	{
		/*Take the mutex to send the message*/
		xSemaphoreTake(send_msg_mutex,portMAX_DELAY);

		xQueueSend(msg_buffer,&buffer_msg,portMAX_DELAY);
		/*Free the mutex to be used by other task*/
		xSemaphoreGive(send_msg_mutex);

		/*Set the event bit event for this task */
		xEventGroupSetBits(msg_event, FIRST_TASK_EVENT);
		vTaskDelay(delay);
	}

}
void second_task(void *pvParameters)
{
	/*Send message*/
	const char msg_2[] = "Second task\n\r";
	message_t buffer_msg;
	buffer_msg.msg = msg_2;

	/*Block status*/
	const TickType_t delay = pdMS_TO_TICKS(200U);
	sender_number = SENDER_ONE;

	for(;;)
	{
		/*Take the mutex to send the message*/
		xSemaphoreTake(send_msg_mutex,portMAX_DELAY);

		xQueueSend(msg_buffer,&buffer_msg,portMAX_DELAY);
		/*Free the mutex to be used by other task*/
		xSemaphoreGive(send_msg_mutex);

		/*Set the event bit event for this task */
		xEventGroupSetBits(msg_event, SECOND_TASK_EVENT);
		vTaskDelay(delay);
	}
}

void third_task(void *pvParameters)
{
	message_t buffer_msg;

	/*Receive message*/
	for(;;)
	{
		/*Wait for message*/
		xEventGroupWaitBits(msg_event, (SECOND_TASK_EVENT | FIRST_TASK_EVENT), NULL, NULL, portMAX_DELAY) ;
		xQueueReceive(msg_buffer, &buffer_msg, portMAX_DELAY);
		/*Send and take message are bussy*/
		xSemaphoreTake(send_msg_mutex,portMAX_DELAY);
		xSemaphoreTake(take_msg_mutex,portMAX_DELAY);
		/*The message is completed so lets print it*/
		PRINTF("%s \n\r", buffer_msg.msg);

		/*Free the mutex to be used by other task*/
		xSemaphoreGive(take_msg_mutex);
		/*Clear manually the even group bits for each task*/
		switch(sender_number)
		{
		case(FIRST_TASK_EVENT):
				xEventGroupClearBits(msg_event, FIRST_TASK_EVENT);
				break;
		case(SECOND_TASK_EVENT):
				xEventGroupClearBits(msg_event, SECOND_TASK_EVENT);
				break;
		}
		/*Free the mutex to be used by other task*/
		xSemaphoreGive(send_msg_mutex);


	}

}
