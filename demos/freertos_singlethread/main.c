#define USE_STDPERIPH_DRIVER
#include "stm32f10x.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

static void setup_hardware( void );

void delay(uint32_t ms) {                                                                                                                                    
	ms *= 1440; // 3360=168MHz, 1440=72MHz
	while(ms--) {
		__NOP();
	}
}


void led_flash_task( void *pvParameters )
{
	while(1) {
		/* Toggle the LED. */
		GPIOB->ODR = GPIOB->ODR ^ GPIO_Pin_1;

		/* Wait one second. */
		vTaskDelay(500);
	}
}

void led_flash2_task( void *pvParameters )
{
	while(1) {
		/* Toggle the LED. */
		GPIOB->ODR = GPIOB->ODR ^ GPIO_Pin_2;

		/* Wait one second. */
		vTaskDelay(500);
	}
}

const char data[] = "Daniel Trnila\r\n";
void uart_task(void *param) {
	init_rs232();
	USART_Cmd(USART2, ENABLE);
	int size = strlen(data);
	int i = 1;
	for(;;) {
		send_byte(data[i % size]);
		i++;

//		if(i % 100 == 0) {
//			vTaskDelay(10);		
//		}
	}
}
void uart_1_task(void *param) {
	init_rs232_1();
	USART_Cmd(USART1, ENABLE);
	int size = strlen(data);
	int i = 1;
	for(;;) {
		send_byte_1(data[i % size]);
		i++;

//		if(i % 100 == 0) {
//			vTaskDelay(1);		
//		}
	}
}

int main(void)
{
	init_led();
/*	while(1) {
		GPIOB->ODR = GPIOB->ODR ^ GPIO_Pin_1;
		delay(100);
	}
	*/


//	xTaskCreate( led_flash_task, ( signed portCHAR * ) "LED Flash", 512 /* stack size */, NULL, tskIDLE_PRIORITY + 5, NULL );
//	xTaskCreate( led_flash2_task, ( signed portCHAR * ) "LED Flash", 512 /* stack size */, NULL, tskIDLE_PRIORITY + 5, NULL );
//	xTaskCreate(uart_task, "UART", 128, NULL, tskIDLE_PRIORITY + 5, NULL);
	xTaskCreate(uart_1_task, "UART", 128, NULL, tskIDLE_PRIORITY + 5, NULL);

	/* Start running the task. */
	vTaskStartScheduler();

	return 0;
}

void vApplicationTickHook( void )
{
}
