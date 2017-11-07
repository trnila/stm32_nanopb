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
		send_byte_1('x');
		send_byte_1(input);
	}
}

const int STATE_TYPE = 0;
const int STATE_SIZE = 1;
const int STATE_CONTENT = 2;
void uart_processor(void* param) {
	static char input[10];
	static char inp;
	static char pos = 0;
	static int val;
	static int type, size;
	static char state = 0;
	for(;;) {
		xQueueReceive(rxQueue, (void*) &inp, portMAX_DELAY);

		if(state == STATE_TYPE || state == STATE_SIZE) {
			input[pos] = inp;
			val |= inp << (pos * 8);
			pos = (pos + 1) % 4;
			if(pos == 0) {
				printf("RCV: %d %x %x %x %x\r\n", val, input[0], input[1], input[2], input[3]);

				if(state == STATE_TYPE) {
					state = STATE_SIZE;
					type = val;
				} else if (state == STATE_SIZE) {
					state = STATE_CONTENT;
					size = val;
				}
				val = 0;
				pos = 0;
			}
		} else if (state == STATE_CONTENT) {
			input[pos] = inp;
			pos++;
			if(pos == size) {
				printf("content: %s\r\n", input);
				state = STATE_TYPE;
				val = 0;
				pos = 0;
			}
			
		} else {
			printf("fail %d\r\n", "");
		}
	}
}


void USART1_IRQHandler(void) {
	static char inp;
	static int wake;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		inp = USART_ReceiveData(USART1);
		xQueueSendFromISR(rxQueue, &inp, &wake);
	} else {
		panic("err");
	}
}

void init() {
	rxQueue = xQueueCreate(255, sizeof(char));
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
	//xTaskCreate(uart_1_task, "UART", 128, NULL, tskIDLE_PRIORITY + 5, NULL);
	xTaskCreate(uart_processor, "UART", 128, NULL, tskIDLE_PRIORITY + 5, NULL);

	/* Start running the task. */
	vTaskStartScheduler();

	return 0;
}

void vApplicationTickHook( void )
{
}
