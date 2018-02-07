/**
  ******************************************************************************
  * @file:		main.c
  * $Author:	YangXueGuang
  * $Revision:	V2.0
  * $Date:	2016-11-05 00:00:00
  * @brief		Main
  ******************************************************************************
  * @attention
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "usb_lib.h"
#include "hw_config.h"
#include "usbio.h"
#include "us_can_zyt.h"
#include "us_mcu_transfer.h"
#include <string.h>

UART_INFO send_buf;
extern unsigned char US_MCU_STATUS;
extern unsigned int ACC_PWR;
void GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

extern unsigned int Usart_BaudRate;
extern unsigned char US_MCU_HW_STATUS;

int main(void)
{
	UART_INFO *send_ptr = &send_buf;
	uint8_t *send = (uint8_t *)send_ptr;
	int send_size = 0;

	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x3400);	//Set Flash Start Address

	Set_System();													//System && Clock Init
	USB_Interrupts_Config();							//USB Interrupt Config
	us_ext_nvic_init();										//NVIC Init
	
	USART_Configuration(Usart_BaudRate);				//Usart Init
	
	Set_USBClock();										//Set USB Clock
	USB_Init();												//USB Init	

	us_mcu_id_get();									//US Get MCU ID

//	if(MCU_TEST){
//		GPIO_Config(); 								//For ZhiQian Developer Edition
//	}else{
		cp_gpio_init();								//US GPIO Init
//	}
	us_dev_init();									//US Device Init
	//us_mcu_hungry_dog_init();						//Hungry Dog Init

	if(1){
		us_init_timer2();								//TIM2 Init
		/*us_init_timer3();*/							//TIM3 Init
		US_MCU_STATUS = MCU_DEV_INIT;
		US_MCU_HW_STATUS = MCU_HW_WAIT;
	}else{
		US_MCU_STATUS = MCU_DEV_DECODE;
	}

	while(1){
		if(GetEPTxStatus(ENDP2) == EP_TX_NAK){
			if(us_mcu_rc_buff_delete(send_ptr) == OK){
				if((send_size = us_mcu_uart_coder(send_ptr)) < 0){
					us_dev_error(MCU_CONFIG, (unsigned char *)__func__, strlen(__func__)+1, send_size);
				}else{
					USB_SendData(send, send_size);
				}
			}
		}
	}
}

#ifdef  USE_FULL_ASSERT
/**
  * @说明   assert_param 用于参数检查.
  * @参数  expr: 假如expr是flase, 将会调用 assert_param
  *   报告错误发生所在的源文件名和所在的行数
  *   假如expr 是 true, 将步返回值.
  * @返回值 无
  */
void assert_failed(uint8_t* file, uint32_t line)
{		
	us_dev_uart_send("\nASSERT_FAILED\n", 16);
	
	sleep(1);
	us_mcu_reset();
	
}
#endif

/*********************************END OF FILE**********************************/
