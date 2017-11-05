#define USE_STDPERIPH_DRIVER
#include "stm32f10x.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


int printf(const char *format, ...);

#define panic(...) printf(__VA_ARGS__); for(;;);

xQueueHandle rxQueue;

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

const char data[] = "Hello world\r\n";
void uart_task(void *param) {
	init_rs232();
	USART_Cmd(USART2, ENABLE);
	int size = strlen(data);
	int i = 1;
	for(;;) {
		send_byte(data[i % size]);
		i++;
	}
}

void uart_1_task(void *param) {
	for(;;) {
		char input;
		xQueueReceive(rxQueue, (void*) &input, portMAX_DELAY);
		if(input >= 'a' && input <= 'z') {
			input -= 'a' - 'A';
		}
		send_byte_1(input);
	}
}

void USART1_IRQHandler(void) {
	static char input;
	static portBASE_TYPE wake;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		input = USART_ReceiveData(USART1);
		if(rxQueue != NULL) {
			xQueueSendFromISR(rxQueue, &input, &wake);
		} else {
			panic("rxQueue NULL");
		}
	} else {
		panic("Unhandled uart event");
	}
}

void init() {
	rxQueue = xQueueCreate(10, sizeof(char));
	if(rxQueue == NULL) {
		//TODO: uart not available yet!
		panic("Could not allocate queue");
	}
	init_rs232_1();
	enable_rs232_interrupts_1();
	USART_Cmd(USART1, ENABLE);
}

int main(void)
{
	init();
	printf("Initialized\r\n", 0);
	init_led();

	/*	while(1) {
		GPIOB->ODR = GPIOB->ODR ^ GPIO_Pin_1;
		delay(100);
		}
		*/


	xTaskCreate( led_flash_task, ( signed portCHAR * ) "LED Flash", 512 /* stack size */, NULL, tskIDLE_PRIORITY + 5, NULL );
	xTaskCreate( led_flash2_task, ( signed portCHAR * ) "LED Flash", 512 /* stack size */, NULL, tskIDLE_PRIORITY + 5, NULL );
	//	xTaskCreate(uart_task, "UART", 128, NULL, tskIDLE_PRIORITY + 5, NULL);
	xTaskCreate(uart_1_task, "UART", 128, NULL, tskIDLE_PRIORITY + 5, NULL);

	/* Start running the task. */
	vTaskStartScheduler();

	return 0;
}

void vApplicationTickHook( void )
{
}
