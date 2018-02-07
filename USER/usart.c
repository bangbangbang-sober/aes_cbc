/**
  ******************************************************************************
  * @file    usart.c
  * $Author: wdluo $
  * $Revision: 67 $
  * $Date:: 2012-08-15 19:00:29 +0800 #$
  * @brief   串口相关函数。
  ******************************************************************************
  * @attention
  *
  *<h3><center>&copy; Copyright 2009-2012, ViewTool</center>
  *<center><a href="http:\\www.viewtool.com">http://www.viewtool.com</a></center>
  *<center>All Rights Reserved</center></h3>
  * 
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "us_can_zyt.h"
#include "us_mcu_transfer.h"
//#define USART_MAX_BUFF_SEND	128
unsigned char USART_SEND_BUFF[USART_MAX_BUFF_SEND] = {0};
int send_index = 0;

extern unsigned char US_MCU_STATUS;

/** @addtogroup USART
  * @brief 串口模块
  * @{
  */

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


/**
  * @brief  开启GPIOA,串口1时钟 
  * @param  None
  * @retval None
  * @note  对于某些GPIO上的默认复用功能可以不开启服用时钟，如果用到复用功能的重
           映射，则需要开启复用时钟
  */
void USART_RCC_Configuration(void)
{

	if(MCU_TEST){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);//开复用时钟
		
	}else{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//开复用时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_GPIOB|RCC_APB2Periph_USART1,ENABLE);
		GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE);
	}
}

/**
  * @brief  设置串口1发送与接收引脚的模式 
  * @param  None
  * @retval None
  */
void USART_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;


	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	
	GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void USART_TEST_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;


	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}


void us_ext_nvic_gpio(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);  
}
/**
  * @brief  配置串口1，并使能串口1
  * @param  None
  * @retval None
  */
void USART_Configuration(unsigned int BaudRate)
{
	USART_InitTypeDef USART_InitStruct;

	USART_RCC_Configuration();
	us_ext_nvic_gpio();

	USART_InitStruct.USART_BaudRate = BaudRate;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	
	USART_Init(USART1, &USART_InitStruct);
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//使能接收中断
	USART_ITConfig(USART1,USART_IT_TXE,ENABLE);//使能发送中断
	USART_ITConfig(USART1, USART_IT_TC, ENABLE);
	USART_Cmd(USART1, ENABLE);//使能串口1
	
	USART_ClearITPendingBit(USART1, USART_IT_TC);//清除中断TC位

	if(MCU_TEST){
		USART_TEST_GPIO_Configuration();
	}else{
		USART_GPIO_Configuration();
	}
}

extern unsigned char DEV_STATUS_CTRL[US_DEV_NUM];
extern int u_front;
extern int u_rear;


//unsigned int  test_num = 0;
UART_RC_BUFF buff;
unsigned int send_num = 0;
void USART1_IRQHandler(void) 
{
	unsigned char temp = 0, s_data = 0; 
	//if(USART_GetFlagStatus(USART1, USART_FLAG_ORE)!= RESET)
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		if(DEV_STATUS_CTRL[USART_0] == DEV_ON && US_MCU_STATUS == MCU_DEV_LINK){
			us_usart_receivedata(USART1->DR);				//get Usart data		
			send_num++;
		}else{
			temp = USART1->DR;
		}
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}

	if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) 
	{
		USART_ClearITPendingBit(USART1, USART_IT_TXE);			 /* Clear the USART transmit interrupt					*/
		if(u_front != u_rear){
			//USART_SendData(USART1, USART_SEND_BUFF[send_index++]);

			#if 1
			us_usart_rc_buff_delete(&s_data);
			#if 0
			if(s_data != test_num){
				s_data = 0xdd;
			}
			#endif
			#if 0
			{
				unsigned char i = 0
				for(i = 0; i < buff.length; i++){
					USART_SendData(USART1, buff.data[i]);
				}
			}
			#endif

			USART_SendData(USART1, s_data);
			#if 0
			test_num++;
			if(test_num == 30){
				test_num = 0;
			}
			#endif
			#endif
			
		}else{
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
			USART_ITConfig(USART1, USART_IT_TC, ENABLE);
		}	
    	} else if (USART_GetITStatus(USART1, USART_IT_TC) != RESET){

		USART_ClearITPendingBit(USART1, USART_IT_TC);

		USART_ITConfig(USART1, USART_IT_TC, DISABLE);

    	}
	
}



PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	USART_SendData(USART1,(u8)ch);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);

	return ch;
}
/**
  * @}
  */

/*********************************END OF FILE**********************************/

