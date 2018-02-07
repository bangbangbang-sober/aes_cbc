/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
* File Name          : hw_config.c
* Author             : MCD Application Team
* Version            : V3.2.1
* Date               : 07/05/2010
* Description        : Hardware Configuration & Setup
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "platform_config.h"
#include "usb_pwr.h"
#include "usb_lib.h"

#include "us_mcu_transfer.h"
#include "us_can_zyt.h"
#include<string.h>
#include<stdio.h>
#define ACC_TIMER		2400*5
#define SIG_TIMER		2400*10
#define ACC_MSG_TIMER	2400*6

extern unsigned int send_num;
extern unsigned char US_MCU_HW_STATUS;
extern int Hungry_ch;
extern int Hungry_Dog;
extern int Hungry_timer;
extern int Hungry_Dog_Lock;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures Main system clocks & power.
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_System(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/   
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if (HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
 
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

#ifdef STM32F10X_CL
    /* Configure PLLs *********************************************************/
    /* PLL2 configuration: PLL2CLK = (HSE / 2) * 10 = 40 MHz */
    RCC_PREDIV2Config(RCC_PREDIV2_Div2);
    RCC_PLL2Config(RCC_PLL2Mul_10);

    /* Enable PLL2 */
    RCC_PLL2Cmd(ENABLE);

    /* Wait till PLL2 is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
    {}

    /* PLL configuration: PLLCLK = (PLL2 / 5) * 9 = 72 MHz */ 
    RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div5);
    RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_9);
#else
    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
#endif

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock configuration.
       User can add here some code to deal with this error */    

    /* Go to infinite loop */
    while (1)
    {
    }
  }
  
  /* enable the PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* Set all the GPIOs to AIN */
  GPIO_AINConfig();

#ifndef STM32F10X_CL
  /* Enable the USB disconnect GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_DISCONNECT, ENABLE);

  /* USB_DISCONNECT used as USB pull-up */
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);
#endif /* STM32F10X_CL */

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE); 	
  //GPIO_PinRemapConfig(GPIO_Remap2_CAN1, ENABLE);//使能改变串口一管脚的映射

  /* Configure the LED IOs */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_12 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz).
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
#ifdef STM32F10X_CL
  /* Select USBCLK source */
  RCC_OTGFSCLKConfig(RCC_OTGFSCLKSource_PLLVCO_Div3);

  /* Enable the USB clock */ 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_OTG_FS, ENABLE) ;
#else
  /* Select USBCLK source */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  
  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
#endif /* STM32F10X_CL */
}

/*******************************************************************************
* Function Name  : GPIO_AINConfig
* Description    : Configures all IOs as AIN to reduce the power consumption.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void GPIO_AINConfig(void)
{

}
/*
IN :defalut High  LOW is OK
Signal IN
Left_IN-----------------PB0
Right_IN---------------PB1
Reverse_IN------------PA4

USB Power OUT
USB PW1--------------PA14
USB PW2--------------PA15
USB PW3--------------PB3
USB PW4--------------PB4
USB PW5--------------PB5
USB PW6--------------PB8 out

812_IR_IN
812_IR_IN--------------PA8

ACC_IN
ACC_IN-----------------PA0

PW_ON (IN)
PW_ON----------------PA1

REVERSE_OUT_POWER High is OK
REVERSE_OUT_POWER-----------------PB12

CVBS 
CVBS_OUT1--------------
CVBS_OUT2--------------
CVBS_IN1--------------
CVBS_IN2--------------

CMMB IR
CMMB_IR-------------PB9   in

RED zhuanma----------PB15

AR IN
AR+ -------------------
AR- -------------------



IO
IO1_OUT-----------------PA6
IO2_OUT-----------------PA7
IO3_OUT-----------------PA5

TXD_MCU----------------PB9
RXD_MCU----------------PB8

USART2RX---------------PA3
USART2TX---------------PA2

IIC-------------PB10 clock
IIC-------------PB11 data
812daiji------------PB13 default H   ( low-500ms---H)
*/
extern unsigned char DEV_STATUS_CTRL[US_DEV_NUM];
extern unsigned char US_MCU_STATUS;
extern unsigned int ACC_PWR;
extern int LCD_MODE;
int USART_DEBUG_TRAS = 0;

int cp_gpio_init(void)
{
	int ret = 0;
	unsigned int temp = 0;
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB |RCC_APB2Periph_AFIO , ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

	/*GPIO ACC PA0*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*GPIO A IN !*/
	//Reverse_IN------------PA4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/*GPIO A OUT !*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	/*GPIO B IN*/
	//Left_IN------PB0  Right_IN-----PB1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1|GPIO_Pin_8 |GPIO_Pin_9;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/*GPIO B OUT*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 |GPIO_Pin_4|GPIO_Pin_5 |GPIO_Pin_10 | GPIO_Pin_11 |GPIO_Pin_12|GPIO_Pin_14;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);	//ACC
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);	//
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);	//Reverse_IN

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);	//Left_IN
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);	//Right_IN

	EXTI_InitStructure.EXTI_Line = GPIO_Pin_0|GPIO_Pin_1| EXTI_Line4 | EXTI_Line8 ;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	
	temp = *(unsigned int*)US_ACC_CONFIG;						//ACC CONFIG GET

	if((temp == ACC_CONF_DIS) || (temp == ACC_CONF_EN)){
		ACC_PWR = temp;
	}else{
		ACC_PWR = ACC_CONF_EN;
	}
	
	/* -----V3.8-----ACC && LCD MODE*/
	LCD_MODE = US_MANU;
	ACC_PWR = ACC_CONF_DIS;
	/*******************************/
	
	if((ACC_PWR == ACC_CONF_EN) && (SIG_PIN5 == 1)){
		USART_DEBUG_TRAS = 1;
		GPIO_SetBits(GPIOB, GPIO_Pin_14);						//S812_POWER_EN
		GPIO_ResetBits(GPIOB, GPIO_Pin_11);						//CVBS2
		GPIO_ResetBits(GPIOA, GPIO_Pin_2);						//CVBS1
		GPIO_SetBits(GPIOB, GPIO_Pin_10);						//LCD ON
	}else{
		USART_DEBUG_TRAS = 0;
		GPIO_SetBits(GPIOB, GPIO_Pin_14);						//S812_POWER_EN
		GPIO_SetBits(GPIOB, GPIO_Pin_11);						//CVBS2
		GPIO_SetBits(GPIOA, GPIO_Pin_2);						//CVBS1
		GPIO_SetBits(GPIOB, GPIO_Pin_10);						//LCD ON
	}

	if(ACC_PWR == ACC_CONF_EN){
		if(SIG_PIN5 == 1){
			GPIO_ResetBits(GPIOB, GPIO_Pin_10); 				//LCD OFF
		}else if(SIG_PIN5 == 0){
			GPIO_SetBits(GPIOB, GPIO_Pin_10); 					//LCD ON
		}
	}else if(ACC_PWR == ACC_CONF_DIS){
		GPIO_SetBits(GPIOB, GPIO_Pin_10);						//LCD ON
	}
	
	//GPIO_SetBits(GPIOA, GPIO_Pin_6);							//IO1_OUT
	//GPIO_SetBits(GPIOA, GPIO_Pin_7);							//IO2_OUT
	//GPIO_SetBits(GPIOA, GPIO_Pin_5);							//IO3_OUT
	
	GPIO_SetBits(GPIOB, GPIO_Pin_12);

	//USB DP ------Low
	GPIO_ResetBits(GPIOA, GPIO_Pin_13);
	
	//USB Power ON
	GPIO_SetBits(GPIOA, GPIO_Pin_14);
	GPIO_SetBits(GPIOA, GPIO_Pin_15);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_3);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_4);
	GPIO_SetBits(GPIOB, GPIO_Pin_5);

	return ret;
}

int us_ext_nvic_init()
{
	int error = 0;

  	NVIC_InitTypeDef   NVIC_InitStructure;

  	/* Set the Vector Table base location at 0x08000000 */
  	//NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* 2 bit for pre-emption priority, 2 bits for subpriority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

#if 0
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);  
#endif

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);   

#if 0
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);   
#endif

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;				//响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;				//响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );

	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;				//响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init( &NVIC_InitStructure );
#if 1	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; //响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init( &NVIC_InitStructure );
#endif

	send_num = 0;

	return error;
}

int us_init_timer2(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

    	TIM_DeInit(TIM2);										//复位定时器
    	TIM_TimeBaseStructure.TIM_Period=50;							//定时器初始值	 2000 == 1s
    	TIM_TimeBaseStructure.TIM_Prescaler=(36000 - 1);				//时钟预分频
    	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;			// 时钟分割
    	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	//向上计数模式
    	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);				//初始化定时器的值
    	TIM_ClearFlag(TIM2,TIM_FLAG_Update);						//清除定时器中断标志 
    	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);					//使能中断
    	TIM_Cmd(TIM2,ENABLE); 									//开启时钟
	
	return 0;  
}
#if 0
int us_init_timer3()
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	TIM_DeInit(TIM3);											//复位定时器
	TIM_TimeBaseStructure.TIM_Period=100;						//定时器初始值
	TIM_TimeBaseStructure.TIM_Prescaler=(36000 - 1);				//时钟预分频
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;			//时钟分割
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	//向上计数模式
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);				//初始化定时器的值
	TIM_ClearFlag(TIM3,TIM_FLAG_Update); 						//清除定时器中断标志 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);					//使能中断
	TIM_Cmd(TIM3,ENABLE);									//开启时钟  

	return 0;
}
#endif 


void us_mcu_signal_send(unsigned char dev, unsigned char data)
{
	US_DEV_TRANS trans_send;

	trans_send.cmd_id = 0;
	trans_send.status = 0;
	trans_send.dev = dev;
	trans_send.data[0] = data;
	trans_send.length = 1;
	us_mcu_rc_buff_enter(US_SINGNAL_DATA, (unsigned char*)&trans_send, sizeof(trans_send));

	return;
}

unsigned long acc_pwr_on_timer = 0, acc_pwr_off_timer = 0, on_timer = 0, off_timer = 0;
extern int Light_WakeUp;

int LCD_MODE = US_MANU;


int us_mcu_pwr_set(unsigned char sig, unsigned char data)
{
	int ret = 0;

	switch( US_MCU_HW_STATUS ){
		case MCU_HW_WAIT:
			if(sig == SIGNAL_IO_5 ){
				if(data == 0 ){
					GPIO_SetBits(GPIOB, GPIO_Pin_11);			//CVBS2 ON
					GPIO_SetBits(GPIOA, GPIO_Pin_2);			//CVBS1 ON
					USART_DEBUG_TRAS = 0;
					GPIO_SetBits(GPIOB, GPIO_Pin_14);			//S812 ON
					if(LCD_MODE == US_AUTO){
						GPIO_SetBits(GPIOB, GPIO_Pin_10);	
					}

					if(Hungry_ch == 0){
						Hungry_ch = 1;
						Hungry_timer = 0;					//Start Hungry Dog
					}
					US_MCU_HW_STATUS = MCU_HW_WORK;
				}
			}else if(Light_WakeUp != TAG_OFF){
				if(data == 0 ){
					GPIO_SetBits(GPIOB, GPIO_Pin_11);			//CVBS2 ON
					GPIO_SetBits(GPIOA, GPIO_Pin_2);			//CVBS1 ON
					USART_DEBUG_TRAS = 0;
					GPIO_SetBits(GPIOB, GPIO_Pin_14);			//S812 ON

					if(Hungry_ch == 0){
						Hungry_ch = 1;
						Hungry_timer = 0;					//Start Hungry Dog
					}
					US_MCU_HW_STATUS = MCU_HW_START;
					acc_pwr_off_timer 	= SIG_TIMER; 
					off_timer = 0;
				}
			}
			break;
		case MCU_HW_START:
			if(sig == SIGNAL_IO_5 ){
				if(data == 0 ){
					if(LCD_MODE == US_AUTO){
						GPIO_SetBits(GPIOB, GPIO_Pin_10);				//LCD OFF
					}
					US_MCU_HW_STATUS = MCU_HW_WORK;
					acc_pwr_off_timer = 0;
				}else{
					//acc_pwr_off_timer 	= ACC_TIMER; 
					//off_timer = 0;
				}
				
			}else{
				if(data == 0 ){
					acc_pwr_off_timer 	= SIG_TIMER; 
					off_timer = 0;
				}
			}
			break;
		case MCU_HW_WORK:
			if(sig == SIGNAL_IO_5 ){
				if(data == 0 ){
					if(LCD_MODE == US_AUTO){
						GPIO_SetBits(GPIOB, GPIO_Pin_10);	   		//LCD ON
					}
					acc_pwr_off_timer = 0;
				}else{
				
					if(LCD_MODE == US_AUTO){
						GPIO_ResetBits(GPIOB, GPIO_Pin_10);  			//LCD OFF		
					}
					acc_pwr_off_timer 	= ACC_TIMER; 
					off_timer = 0;
					US_MCU_HW_STATUS = MCU_HW_START;
				}
			}else{
			}
			break;
	}

	return ret;
}

unsigned char sig_now[8], sig_last[8] = {2,2,2,2,2,2,2,2};

int us_mcu_signal_check(unsigned char sig, unsigned char data)
{
	int ret = 0, num = 0;
	
	switch(sig){
		case SIGNAL_IO_1:
			num = 0;
			if(data == 0){
				if(ACC_PWR == ACC_CONF_DIS){
					ACC_PWR = ACC_CONF_EN;
					LCD_MODE = US_AUTO;
				}
			}
			break;
		case SIGNAL_IO_2:
			num = 1;
			if(data == 0){
				if(ACC_PWR == ACC_CONF_DIS){
					ACC_PWR = ACC_CONF_EN;
					LCD_MODE = US_AUTO;
				}
			}
			break;
		case SIGNAL_IO_3:
			num = 2;
			if(data == 0){
				if(ACC_PWR == ACC_CONF_DIS){
					ACC_PWR = ACC_CONF_EN;
					LCD_MODE = US_AUTO;
				}
			}
			break;
		case SIGNAL_IO_4:
			num = 3;
			break;
		case SIGNAL_IO_5:
			num = 4;
			if(data == 0){
				if(ACC_PWR == ACC_CONF_DIS){
					ACC_PWR = ACC_CONF_EN;
					LCD_MODE = US_AUTO;
				}
			}
			break;
		case SIGNAL_IO_6:
			num = 5;
			break;
		case SIGNAL_IO_7:
			num = 6;
			break;
		case SIGNAL_IO_8:
			num = 7;
			break;			
	}

	if(ACC_PWR == ACC_CONF_EN){
		us_mcu_pwr_set(sig, data);
	}

	sig_now[num] = data;

	if(sig_now[num]!= sig_last[num]){
	
		

		if(US_MCU_STATUS == MCU_DEV_LINK){
			us_mcu_signal_send(sig, data);
		}
		sig_last[num] = sig_now[num];
	}

	return ret;
}


extern US_HOST_PING ping_s;
extern unsigned long long ping_time;

unsigned long long timer = 0;

int usart_ch = 0;
extern unsigned long long usart_rece_time;

void us_mcu_link_check(void)
{
	if((timer - ping_time < 80) && ping_s.link == LINK_OK){
		US_MCU_STATUS = MCU_DEV_LINK;
	}else{
		//ping_time = timer;
		US_MCU_STATUS = MCU_DEV_UNLINK;
	}
	return ;
}

void us_mcu_ping(void)
{
	if(timer % 40 == 0){
		
		ping_s.mcu_status = US_MCU_STATUS;

		us_mcu_rc_buff_enter(US_MCU_PING, (unsigned char*)&ping_s, sizeof(US_HOST_PING));
	}	
	return;
}

void us_mcu_signal_check_fun(void)
{
	if(DEV_STATUS_CTRL[SIGNAL_IO_1] == DEV_ON){
		us_mcu_signal_check(SIGNAL_IO_1, SIG_PIN1);
	}
	if(DEV_STATUS_CTRL[SIGNAL_IO_2] == DEV_ON){
		us_mcu_signal_check(SIGNAL_IO_2, SIG_PIN2);
	}
	if(DEV_STATUS_CTRL[SIGNAL_IO_3] == DEV_ON){
		us_mcu_signal_check(SIGNAL_IO_3, SIG_PIN3);
	}
	if(DEV_STATUS_CTRL[SIGNAL_IO_4] == DEV_ON){
		us_mcu_signal_check(SIGNAL_IO_4, SIG_PIN4);//PW_ON
	}
	
	us_mcu_signal_check(SIGNAL_IO_5, SIG_PIN5); //ACC Check
	
	return;
}



void us_mcu_usart_timeout(void)
{
	if(timer - usart_rece_time >= 1){
		
		//while(usart_ch != 0);
		//usart_ch = 1;
		if(DEV_STATUS_CTRL[USART_0] == DEV_ON && US_MCU_STATUS == MCU_DEV_LINK){
			us_usart_trans();
		}
		//usart_ch = 0;
	}	
}

void us_mcu_acc_power_on_timer(void)
{
	if(acc_pwr_on_timer > 0){
		acc_pwr_on_timer--;
		if(acc_pwr_on_timer == 0){
			GPIO_SetBits(GPIOB, GPIO_Pin_11);	//CVBS2 ON
			GPIO_SetBits(GPIOA, GPIO_Pin_2);	//CVBS1 ON
			USART_DEBUG_TRAS = 0;
			GPIO_SetBits(GPIOB, GPIO_Pin_14);	//S812 ON
		}
	}
}

int LAST_ANDROID_STA = ANDROID_INIT;

void us_mcu_acc_power_off_timer(void)
{
	if(US_MCU_HW_STATUS == MCU_HW_START){

		if( acc_pwr_off_timer > 0 ){
			off_timer++;
			if((off_timer < acc_pwr_off_timer) && (off_timer == ACC_MSG_TIMER)){
				if(1 == SIG_PIN5){
					us_mcu_signal_send(SIGNAL_IO_5, SIG_PIN5);
				}
			}
			if(off_timer >= acc_pwr_off_timer){
				off_timer = 0;
				acc_pwr_off_timer = 0;
				if( 1 == SIG_PIN5 ){
					GPIO_ResetBits(GPIOB, GPIO_Pin_11);	//CVBS2 OFF
					GPIO_ResetBits(GPIOA, GPIO_Pin_2);		//CVBS1 OFF
					GPIO_ResetBits(GPIOB, GPIO_Pin_14);	//S812 OFF
					GPIO_ResetBits(GPIOB, GPIO_Pin_10);	//LCD OFF
					//LCD_MODE = US_AUTO;

					USART_DEBUG_TRAS 	= 1;				//Start Usart Debug Mode
				}
				US_MCU_HW_STATUS  = MCU_HW_WAIT;
				//Sys_Enter_Standby();
			}
		}else if(acc_pwr_off_timer == 0){
			off_timer = 0;
		}
	}
	
}

void us_mcu_hw_status_calibration(void)
{
	
}
extern int front, rear, u_front, u_rear;
int hardfault_ch = 0;
uint32_t r_sp = 0;

void us_mcu_usart_debug_mode(void)
{
	char data[128] = {0};

	if(timer % 40 == 0){

		sprintf(data, "*****\r\n Timer:%lld \r\n front:%d rear:%d\r\n u_front:%d u_rear:%d\r\n hardfault_ch:%d\r\n SP:0x%x\r\n", timer, front, rear, u_front, u_rear, hardfault_ch, r_sp);
		us_dev_uart_send((unsigned char *)data, (unsigned char )strlen(data));
	}
}



void us_mcu_hungry_dog_timer(void)
{
	if(timer % 40 == 0){
		if(Hungry_Dog_Lock == DATA_UNLOCK){
			if((Hungry_Dog != 0) && 
				(Hungry_Dog <= MAX_HUNGRY_TIME_S) &&
				(Hungry_Dog >= MIN_HUNGRY_TIME_S) &&
				(Hungry_timer != TIMER_STOP)){
				Hungry_timer++;
				if(Hungry_timer >= Hungry_Dog){
					//if(US_MCU_STATUS == MCU_DEV_UNLINK){
						Hungry_Dog = 0;
						Hungry_timer = TIMER_STOP;
						//us_mcu_reset();
						us_android_reset();
					//}else{
					//	Hungry_timer = 8888;
					//}
				}
				
			}
		}
	}
	
}

void us_mcu_defined_timer(void)
{
	if(ACC_PWR == ACC_CONF_EN){
		//us_mcu_acc_power_on_timer();
		us_mcu_acc_power_off_timer();
		us_mcu_hw_status_calibration();
	}
	if(USART_DEBUG_TRAS){
		//us_mcu_usart_debug_mode();
	}
	//Hungry Dog
	us_mcu_hungry_dog_timer();
}

unsigned int Local_Timer = 0;
void us_mcu_host_local_timer(void)
{
	if(timer % 40 == 0){

		Local_Timer++;

	}
}

void TIM2_IRQHandler(void)
{

	if( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) 
	{
		us_mcu_link_check();				//US MCU Link Check
		us_mcu_ping();					//US MCU Ping
		us_mcu_signal_check_fun();		//Signal Check
		us_mcu_usart_timeout();			//Usart Timeout
		us_mcu_defined_timer();			//User-defined Timer
		us_mcu_host_local_timer();		//User defined Local Timer
		timer++;						//Tick
		
		TIM_ClearITPendingBit(TIM2 , TIM_IT_Update);
	}
}  

void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET ) 
	{
		if(timer - usart_rece_time >= 1){
		
			if(DEV_STATUS_CTRL[USART_0] == DEV_ON && US_MCU_STATUS == MCU_DEV_LINK){
				us_usart_trans();
			}
		}
		TIM_ClearITPendingBit(TIM3 , TIM_IT_Update);
	}
}

unsigned long long  exti0_time_temp = 0;
unsigned long long  exti1_time_temp = 0;
unsigned long long  exti1_4_time_temp = 0;

unsigned long long  exti4_time_temp = 0;
unsigned long long  exti8_time_temp = 0;


void EXTI0_IRQHandler(void)
{
	if( EXTI_GetITStatus(EXTI_Line0) !=RESET){

		if(US_MCU_HW_STATUS != MCU_HW_INIT){
			if(DEV_STATUS_CTRL[SIGNAL_IO_1] == DEV_ON){
				if(timer - exti0_time_temp >= 1){
					us_mcu_signal_check(SIGNAL_IO_1, SIG_PIN1);
					exti0_time_temp = timer;
				}
			}
		}
		
		if(US_MCU_HW_STATUS != MCU_HW_INIT){
			us_mcu_signal_check(SIGNAL_IO_5, SIG_PIN5);
		}

		EXTI_ClearITPendingBit(EXTI_Line0);
	}
	

}

void EXTI1_IRQHandler(void)
{
	if( EXTI_GetITStatus(EXTI_Line1) !=RESET){
		
		if(US_MCU_HW_STATUS != MCU_HW_INIT){
			if(DEV_STATUS_CTRL[SIGNAL_IO_2] == DEV_ON){

				if(timer - exti1_time_temp >= 1){
					us_mcu_signal_check(SIGNAL_IO_2, SIG_PIN2);
					exti1_time_temp = timer;
				}
			}
		}
		
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}


void EXTI4_IRQHandler(void)
{
	if( EXTI_GetITStatus(EXTI_Line4) !=RESET){
		
		if(US_MCU_HW_STATUS != MCU_HW_INIT){
			
			if(DEV_STATUS_CTRL[SIGNAL_IO_3] == DEV_ON){
				if(timer - exti4_time_temp >= 1){
					us_mcu_signal_check(SIGNAL_IO_3, SIG_PIN3);
					exti4_time_temp = timer;
				}
			}
			
		}
		
		EXTI_ClearITPendingBit(EXTI_Line4);
	}
}
#if 1
void EXTI9_5_IRQHandler(void)
{
	if( EXTI_GetITStatus(EXTI_Line8) !=RESET){
		
		if(US_MCU_HW_STATUS != MCU_HW_INIT){
			
			if(DEV_STATUS_CTRL[SIGNAL_IO_4] == DEV_ON){
				if(timer - exti8_time_temp >= 1){
					us_mcu_signal_check(SIGNAL_IO_4, SIG_PIN4);
					exti8_time_temp = timer;
				}
			}
			
		}
		
		EXTI_ClearITPendingBit(EXTI_Line8);
	}
}
#endif




/*******************************************************************************
* Function Name  : Enter_LowPowerMode.
* Description    : Power-off system clocks and power while entering suspend mode.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{

}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode.
* Description    : Restores system clocks and power while exiting suspend mode.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{

}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config.
* Description    : Configures the USB interrupts.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* 2 bit for pre-emption priority, 2 bits for subpriority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

#ifdef STM32F10X_CL
	/* Enable the USB Interrupts */
	NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the USB Wake-up interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_WKUP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_Init(&NVIC_InitStructure);  
#else
	/* Enable the USB interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

#endif /* STM32F10X_CL */

}

/*******************************************************************************
* Function Name  : USB_Cable_Config.
* Description    : Software Connection/Disconnection of USB Cable.
* Input          : NewState: new state.
* Output         : None.
* Return         : None
*******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)
{
#ifdef STM32F10X_CL  
  if (NewState != DISABLE)
  {
    USB_DevConnect();
  }
  else
  {
    USB_DevDisconnect();
  }
#else /* USE_STM3210B_EVAL or USE_STM3210E_EVAL */
  if (NewState != DISABLE)
  {
    GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
  else
  {
    GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
#endif /* USE_STM3210C_EVAL */
}

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(__IO uint32_t*)(0x1FFFF7E8);
  Device_Serial1 = *(__IO uint32_t*)(0x1FFFF7EC);
  Device_Serial2 = *(__IO uint32_t*)(0x1FFFF7F0);

  Device_Serial0 += Device_Serial2;

  if (Device_Serial0 != 0)
  {
    IntToUnicode (Device_Serial0, &CustomHID_StringSerial[2] , 8);
    IntToUnicode (Device_Serial1, &CustomHID_StringSerial[18], 4);
  }
}

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10; 
    }
    
    value = value << 4;
    
    pbuf[ 2* idx + 1] = 0;
  }
}
#ifdef STM32F10X_CL
/*******************************************************************************
* Function Name  : USB_OTG_BSP_uDelay.
* Description    : provide delay (usec).
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_OTG_BSP_uDelay (const uint32_t usec)
{
  RCC_ClocksTypeDef  RCC_Clocks;  

  /* Configure HCLK clock as SysTick clock source */
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
  
  RCC_GetClocksFreq(&RCC_Clocks);
  
  SysTick_Config(usec * (RCC_Clocks.HCLK_Frequency / 1000000));  
  
  SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk ;
  
  while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
}
#endif
/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
