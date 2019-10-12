#include "stdio.h"
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"

#include "my_RTOS_UART_driver.h"
#include "my_RTOS_I2C_driver.h"


/////////////////////////////////////////
/* Macros */
/////////////////////////////////////////
#define ECHO_TASK_NAME				"echo_task"
#define I2C_TEST_TASK_NAME			"i2c_test_task"

#define ECHO_TASK_PRIORITY  		(configMAX_PRIORITIES - 1U)
#define I2C_TEST_TASK_PRIORITY		(configMAX_PRIORITIES - 1U)

#define M24LC256_ADDRESS			(0x50)
#define TEST_TEXT_LENGTH			(5U)

#define CREATE_TASK(a,b,c,d,e,f)	{												\
										if( pdPASS != xTaskCreate(a,b,c,d,e,f) )	\
										{											\
											PRINTF("%s creation failed.\r\n",#b);	\
											for(;;);								\
										}											\
									}
#define CREATE_SEMAPHORE(r)			{												\
										r = xSemaphoreCreateBinary();				\
										if(NULL == r)								\
										{											\
											PRINTF("%s creation failed.\r\n",#r);	\
											for(;;);								\
										}											\
									}
#define CREATE_MUTEX(r)				{												\
										r = xSemaphoreCreateMutex();				\
										if(NULL == r)								\
										{											\
											PRINTF("%s creation failed.\r\n",#r);	\
											for(;;);								\
										}											\
									}
#define CREATE_EVENTGROUP(r)		{												\
										r = xEventGroupCreate();					\
										if(NULL == r)								\
										{											\
											PRINTF("%s creation failed.\r\n",#r);	\
											for(;;);								\
										}											\
									}
#define CREATE_QUEUE(r,a,b)			{												\
										r = xQueueCreate(a,b);						\
										if(NULL == r)								\
										{											\
											PRINTF("%s creation failed.\r\n",#r);	\
											for(;;);								\
										}											\
									}


TaskHandle_t echo_task_handle	   = 0;
TaskHandle_t i2c_test_task_handle  = 0;


void echo_task(void* args)
{
	/* Set UART channel configuration */
	rtos_uart_config_t uart_config;
	uart_config.baudrate     = 115200;
	uart_config.rx_pin       = 16;
	uart_config.tx_pin       = 17;
	uart_config.pin_mux      = kPORT_MuxAlt3;
	uart_config.uart_channel = UART0;
	uart_config.port         = PORTB;

	/* Initialize UART channel */
	rtos_uart_init(&uart_config);


	/* Constant echoing */
	uint8_t data;
	for(;;)
	{
		rtos_uart_receive(UART0, &data, 1);
		rtos_uart_send(UART0, &data, 1);
	}

}
void i2c_test_task(void* args)
{

/* Task local variables */

	i2c_master_transfer_t xfer = {0};

	uint8_t write_buffer[TEST_TEXT_LENGTH] = "Tufak";
	uint8_t read_buffer[TEST_TEXT_LENGTH]  = "-----";

	uint8_t success_message[] = "I2C success :)";
	uint8_t failure_message[] = "I2C failure :(";
	uint8_t* test_result      = success_message;

	rtos_i2c_config_t i2c_config;
	rtos_uart_config_t uart_config;





/* Initialize I2C channel */
	/* Define channel configuration */

	i2c_config.baudrate    = 10000U;
	i2c_config.i2c_channel = I2C0;
	i2c_config.pin_mux     = 5U;
	i2c_config.port        = PORTE;
	i2c_config.scl_pin     = 24U;
	i2c_config.sda_pin     = 25U;

	/* Initialize channel */
	rtos_i2c_init(&i2c_config);





/* Writing into memory */

	/* Define write transfer */
	xfer.data           = ((uint8_t* volatile) (write_buffer));
	xfer.dataSize       = TEST_TEXT_LENGTH;
	xfer.direction      = kI2C_Write;
	xfer.flags          = kI2C_TransferDefaultFlag;
	xfer.slaveAddress   = M24LC256_ADDRESS;
	xfer.subaddress     = 0U;
	xfer.subaddressSize = 2U;

	/* Write into memory */
	rtos_i2c_transfer(i2c_config.i2c_channel, &xfer);





/* Reading from memory */

	/* Define read transfer */
	xfer.data           = ((uint8_t* volatile) (read_buffer));
	xfer.dataSize       = TEST_TEXT_LENGTH;
	xfer.direction      = kI2C_Read;
	xfer.flags          = kI2C_TransferDefaultFlag;
	xfer.slaveAddress   = M24LC256_ADDRESS;
	xfer.subaddress     = 0U;
	xfer.subaddressSize = 2U;

	/* Read from memory */
	rtos_i2c_transfer(i2c_config.i2c_channel, &xfer);





/* Evaluate test result */
	for( uint8_t index = 0; index < TEST_TEXT_LENGTH; index++ )
	{
		if( write_buffer[index] != read_buffer[index] )
		{
			test_result = failure_message;
			break;
		}
	}





/* Print result */
	/* Set UART channel configuration */
	uart_config.baudrate      = 115200U;
	uart_config.rx_pin        = 16U;
	uart_config.tx_pin        = 17U;
	uart_config.pin_mux       = kPORT_MuxAlt3;
	uart_config.uart_channel  = UART0;
	uart_config.port          = PORTB;

	/* Initialize UART channel */
	rtos_uart_init(&uart_config);

	/* Send result message to terminal */
	rtos_uart_send(UART0, read_buffer, TEST_TEXT_LENGTH);





/* End task */

	/* Suspend itself */
	vTaskSuspend(i2c_test_task_handle);

	/* Infinite loop */
	for(;;);


}







int main(void) {
  	/* Init board hardware. */
//    BOARD_InitBootPins();
    BOARD_InitBootClocks();
//    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
//    BOARD_InitDebugConsole();





    /* Create tasks. */
	CREATE_TASK(echo_task, ECHO_TASK_NAME, configMINIMAL_STACK_SIZE, NULL, ECHO_TASK_PRIORITY, &echo_task_handle);
	CREATE_TASK(i2c_test_task, I2C_TEST_TASK_NAME, 500U, NULL, I2C_TEST_TASK_PRIORITY, &i2c_test_task_handle);





	/* Start scheduler*/
    vTaskStartScheduler();




    /* Delete Tasks*/
	vTaskDelete(echo_task_handle);
	vTaskDelete(i2c_test_task_handle);



	for(;;);
    return 0 ;
}
