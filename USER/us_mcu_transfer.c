#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"
#include "usb_lib.h"


//#include "usb_pwr.h"

#include "us_mcu_transfer.h"
#include "us_can_zyt.h"
#include "usb_pwr.h"
#include "AES_CBC.h"
#include "usart.h"
#include "libMCUAuthenticate.h"

/*Mcu Version Update Log
*
* V3.2 Beta NO SIGNAL PWR ON
* V3.5 ADD STATES AND ADD RESET FUC
* V3.6 ADD SIGNAL SINGLE CHECK
* //V3.7.1 Debug ORE RIG && UsartDebugMode
* V3.7C add light wakeup pwr
* V3.8 add pwr on
* V3.9 add Standby mode
* V4.0 CVS1
********************************/
unsigned char MCU_VERSION[8] = "V4.1";
unsigned char MCU_UPDATE_LOG[64] = 
	"CVS1";

unsigned int ACC_PWR = ACC_CONF_DIS;					//ACC EN

extern unsigned long long timer;

UART_INFO rc_buf[MAX_BUFF_SIZE];

int front, rear;

US_HOST_PING ping_s;

unsigned char DEV_STATUS_CTRL[US_DEV_NUM];
unsigned char US_MCU_STATUS = 0; 
unsigned char US_MCU_HW_STATUS = MCU_HW_INIT;

unsigned long long ping_time = 0;

extern US_HOST_PING ping_s;

unsigned char coder_id[20];
unsigned char coder_length;

unsigned int mcu_id_en[12] = {0};

int ACC_EN = 0;

int u_front = 0;
int u_rear = 0;

int Hungry_Dog = 0;
int Hungry_timer = 0,Hungry_ch = 0;
int Hungry_Dog_Lock = DATA_UNLOCK;
int Light_WakeUp = TAG_ON;

int STOP_MODE = 0;

extern unsigned char USART_SEND_BUFF[USART_MAX_BUFF_SEND];

int us_mcu_rc_buff_init(void)
{
	front = 0;
	rear = 0;
	return OK;
}

int us_mcu_uart_coder(UART_INFO *send)
{
	int ret = 0;
	unsigned char str[MAX_VERSION_LENGTH] = {0};

	if(send == NULL){
		ret = -1;
	}
	if(!ret){
		send->head = UART_HEAD;
		memset((unsigned char *)send->version, 0, MAX_VERSION_LENGTH);
		sprintf((char *)str, "%s", ping_s.id);
		//sprintf((char *)send->version, "%s", ping_s.id);
		memcpy((unsigned char *)send->version, str, ping_s.id_length);
		if(strlen((char *)send->version) >= MAX_VERSION_LENGTH - 1){
			ret = -2;
		}
	}
	if(!ret){
		send->checksum = us_uart_checksum(send);
	}

	if(ret < 0){
		us_dev_error(MCU_CONFIG, (unsigned char *)__func__, strlen(__func__) + 1, ret);
	}

	return 16 + send->length;
}


int us_dev_error(unsigned char dev, unsigned char* func_name, int name_len, int status)
{
	int ret = 0;
	US_DEV_TRANS trans_err = {0};

	if(func_name == NULL || name_len == 0){
		ret = -1;
	}
	if(!ret){
		memcpy(trans_err.data, func_name, name_len);
	}
	if(!ret){
		trans_err.cmd_id		= status;
		trans_err.dev			= dev;
		us_mcu_rc_buff_enter(US_MCU_ERR, (unsigned char*)&trans_err, sizeof(US_DEV_TRANS));
	}
	
	return ret;
}

int us_usart_rc_buff_enter(unsigned char data)
{
	int ret = 0;

	if((u_rear + 1)%USART_MAX_BUFF_SEND == u_front){
		ret = -1;
	}

	if(!ret){
		USART_SEND_BUFF[u_rear] = data;
		u_rear = (u_rear + 1) % USART_MAX_BUFF_SEND;
	}

	if(ret < 0){
		us_dev_error(USART_0, (unsigned char *)__func__, strlen(__func__)+1, ret);
	}

	return ret;
}


int us_usart_rc_buff_delete(unsigned char *data)
{
	int ret = 0;

	if(data == NULL){
		ret = -1;
	}

	if(!ret){
		if( u_front == u_rear ){
			ret = -2;  
		}
	}

	if(!ret){
		*data = USART_SEND_BUFF[u_front];
		u_front =(u_front + 1) % USART_MAX_BUFF_SEND;
	}
	
	return ret;
}

/****************************************************************/
int us_mcu_rc_buff_enter(unsigned char type, unsigned char *send, unsigned char buff_size)
{
	int ret = OK;
	
	if((rear + 1)%MAX_BUFF_SIZE == front){
		ret = -1;
	}
	
	if(!ret){
		rc_buf[rear].type = type;
		rc_buf[rear].length = buff_size;
		memcpy(rc_buf[rear].data, send, buff_size);
		rear = (rear + 1) % MAX_BUFF_SIZE;	
	}

	return ret;	
}

int us_mcu_rc_buff_delete(UART_INFO *str)
{
	int ret = OK;
	if(str == NULL){
		ret = -1;
	}
	if(!ret){
		if( front == rear ){
			ret = -2;  
		}
	}
	if(!ret){
		memcpy(str, &(rc_buf[front]), sizeof(UART_INFO));
		front =(front + 1) % MAX_BUFF_SIZE;
	}

	return ret;
}


UART_INFO recv;
UART_INFO *recv_ptr 		= &recv;
unsigned int CpuID[4]		= {0};


int us_mcu_id_get(void)
{
	CpuID[0]=*(vu32*)(0x1ffff7e8);
	CpuID[1]=*(vu32*)(0x1ffff7ec);
	CpuID[2]=*(vu32*)(0x1ffff7f0);

	return 0;
}

 int us_dev_uart_send(unsigned char *send_buff, unsigned char length)
{
	int ret = 0, n = 0, i = 0; 

	if(send_buff == NULL || length == 0){
		ret = -1;
	}
	if(!ret){
		if(u_front == u_rear){
			n = 1;
		}else{
			n = 0;
		}
	}
	if(!ret){
		for(i = 0; i < (length - n); i++){
			ret = us_usart_rc_buff_enter(*(send_buff+n+i));
		}
	}
 
	if(!ret){
		if( n == 1 ){
			USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
			USART_SendData(USART1,send_buff[0]);
		}
	}

	if(ret < 0){
		us_dev_error(USART_0, (unsigned char *)__func__, strlen(__func__) + 1, ret);
	}

 	return ret;
}

int us_dev_signal_data_get(unsigned char *sig_data)
{
	int ret = 0;
	uint8_t temp = 0;

	if(sig_data == NULL){
		ret = -1;
	}
	if(!ret){
		*sig_data = 0;
		temp = SIG_PIN1;
		*sig_data |= ((temp)<<0);
		
		temp = SIG_PIN2;
		*sig_data |= ((temp)<<1);
		
		temp = SIG_PIN3;
		*sig_data |= ((temp)<<2);
		
		temp = SIG_PIN4;
		*sig_data |= ((temp)<<3);
		
		temp = SIG_PIN5;
		*sig_data |= ((temp)<<4);

	}

	if(ret < 0){
		us_dev_error(SIGNAL_IO_DATA, (unsigned char *)__func__, strlen(__func__) + 1, ret);
	}	
	
	return ret;
}

extern int LCD_MODE;
extern int ACC_MODE;


int us_dev_trans_decoder(unsigned char type, US_DEV_TRANS *trans_buf)
{
	int ret = 0;
	unsigned char dev = trans_buf->dev;
	unsigned char value = 0, sig_data = 0;
	
	US_DEV_TRANS trans_send = {0};
	US_MCU_ID_GET mcu_id_v = {0};

	

	if(DEV_STATUS_CTRL[dev] == DEV_ON && US_MCU_STATUS == MCU_DEV_LINK){
 
		switch(dev){
			case GPIO_OUT_0:						//IO1_OUT
				if(type == US_GPIO_WRITE){
					value = trans_buf->data[0];
					
					if(value == IO_UP){
						GPIO_SetBits(GPIOA, GPIO_Pin_6);	
					}else if(value == IO_DOWN){
						GPIO_ResetBits(GPIOA, GPIO_Pin_6);
					}
				}
				if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_6)){
					value = IO_UP;
				}else{
					value = IO_DOWN;
				}
				trans_send.data[0] =  value;
				trans_send.length = 1;
				break;
			case GPIO_OUT_1:						//IO2_OUT
			
				if(type == US_GPIO_WRITE){
					value = trans_buf->data[0];
					
					if(value == IO_UP){
						GPIO_SetBits(GPIOA, GPIO_Pin_7);	
					}else if(value == IO_DOWN){
						GPIO_ResetBits(GPIOA, GPIO_Pin_7);
					}
				}
				if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_7)){
					value = IO_UP;
				}else{
					value = IO_DOWN;
				}
				trans_send.data[0] =  value;
				trans_send.length = 1;
				break;	
			case GPIO_OUT_2:						//IO3_OUT
			
				if(type == US_GPIO_WRITE){
					value = trans_buf->data[0];
					if(value == IO_UP){
						GPIO_SetBits(GPIOA, GPIO_Pin_5);	
					}else if(value == IO_DOWN){
						GPIO_ResetBits(GPIOA, GPIO_Pin_5);
					}	
				}
				if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_5)){
					value = IO_UP;
				}else{
					value = IO_DOWN;
				}
				trans_send.data[0] =  value;
				trans_send.length = 1;
				break;
			case GPIO_OUT_3:						//REVERSE_OUT_POWER 12v
			
				if(type == US_GPIO_WRITE){
					value = trans_buf->data[0];
					if(value == IO_UP){
						GPIO_SetBits(GPIOB, GPIO_Pin_12);	
					}else if(value == IO_DOWN){
						GPIO_ResetBits(GPIOB, GPIO_Pin_12);
					}
				}
				if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_12)){
					value = IO_UP;
				}else{
					value = IO_DOWN;
				}
				trans_send.data[0] =  value;
				trans_send.length = 1;
				break;
			case GPIO_OUT_4:							//PA2 CVBS1 PWR 5v
				if(type == US_GPIO_WRITE){
					value = trans_buf->data[0];
					if(value == IO_UP){
						GPIO_SetBits(GPIOA, GPIO_Pin_2);	
					}else if(value == IO_DOWN){
						GPIO_ResetBits(GPIOA, GPIO_Pin_2);
					}
				}
				if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_2)){
					value = IO_UP;
				}else{
					value = IO_DOWN;
				}
				trans_send.data[0] =  value;
				trans_send.length = 1;				
				break;
			case GPIO_OUT_5:							//PB11 CVBS2 PWR 5v 
				if(type == US_GPIO_WRITE){
					value = trans_buf->data[0];
					if(value == IO_UP){
						GPIO_SetBits(GPIOB, GPIO_Pin_11);	
					}else if(value == IO_DOWN){
						GPIO_ResetBits(GPIOB, GPIO_Pin_11);
					}
				}
				if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_11)){
					value = IO_UP;
				}else{
					value = IO_DOWN;
				}
				trans_send.data[0] =  value;
				trans_send.length = 1;			
				break;
			case GPIO_OUT_6:					//PB10 LCD PWR 12v 
				if(type == US_GPIO_WRITE){
					LCD_MODE = US_MANU;
					value = trans_buf->data[0];
					if(value == IO_UP){
						GPIO_SetBits(GPIOB, GPIO_Pin_10);	
					}else if(value == IO_DOWN){
						GPIO_ResetBits(GPIOB, GPIO_Pin_10);
					}
				}
				if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_10)){
					value = IO_UP;
				}else{
					value = IO_DOWN;
				}
				trans_send.data[0] =  value;
				trans_send.length = 1;		
				break;
	
				
			case GPIO_IN_0://812_IR_IN
				if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)){
					trans_send.data[0] = IO_UP;
				}else{
					trans_send.data[0] = IO_DOWN;
				}
				trans_send.length = 1;
							
				break;
			case GPIO_IN_1:
				
				break;	
			case GPIO_IN_2:

				break;
			case GPIO_IN_3:

				break;						
			case USART_0:
				if(type == US_USART_WRITE){
 					us_dev_uart_send(trans_buf->data, trans_buf->length);
 					trans_send.length = trans_buf->length; 
				}
				break;

			case SIGNAL_IO_1:
				trans_send.data[0] = SIG_PIN1;//Left_IN
				trans_send.length = 1;
				break;
			case SIGNAL_IO_2:
				trans_send.data[0] = SIG_PIN2;//Right_IN
				trans_send.length = 1;
				break;
			case SIGNAL_IO_3:
				trans_send.data[0] = SIG_PIN3;//Reverse_IN
				trans_send.length = 1;
				break;
			case SIGNAL_IO_4:
				trans_send.data[0] = SIG_PIN4;
				trans_send.length = 1;
				break;
			case SIGNAL_IO_5:
				trans_send.data[0] = SIG_PIN5;//ACC
				trans_send.length = 1;
				break;
			case SIGNAL_IO_DATA:
				us_dev_signal_data_get(&sig_data);
				trans_send.data[0] = sig_data;
				trans_send.length = 1;
				break;
			
			default:
				break;

		};

	}
	
	switch(dev){
		case USB_PWR_1:
			if(type == US_USB_PWR_WRITE){
				value = trans_buf->data[0];
				if(value == USB_PWR_ON){
					GPIO_SetBits(GPIOA, GPIO_Pin_14);	
				}else if(value == USB_PWR_OFF){
					GPIO_ResetBits(GPIOA, GPIO_Pin_14);
				}
			}

			if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_14)){
				value = USB_PWR_ON;
			}else{
				value = USB_PWR_OFF;
			}
			trans_send.data[0] =  value;
			trans_send.length = 1;
			
			break;

		case USB_PWR_2:
			
			if(type == US_USB_PWR_WRITE){
				value = trans_buf->data[0];
				if(value == USB_PWR_ON){
					GPIO_SetBits(GPIOA, GPIO_Pin_15);	
				}else if(value == USB_PWR_OFF){
					GPIO_ResetBits(GPIOA, GPIO_Pin_15);
				}
			}
			
			if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_15)){
				value = USB_PWR_ON;
			}else{
				value = USB_PWR_OFF;
			}
			trans_send.data[0] =  value;

			trans_send.length = 1;
			
			break;
		case USB_PWR_3:
			
			if(type == US_USB_PWR_WRITE){
				value = trans_buf->data[0];
				if(value == USB_PWR_ON){	
					GPIO_SetBits(GPIOB, GPIO_Pin_3);	
				}else if(value == USB_PWR_OFF){
					GPIO_ResetBits(GPIOB, GPIO_Pin_3);
				}
			}
			if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_3)){
				value = USB_PWR_ON;
			}else{
				value = USB_PWR_OFF;
			}
			trans_send.data[0] =  value;

			trans_send.length = 1;
			
			break;
		case USB_PWR_4:
			
			if(type == US_USB_PWR_WRITE){
				value = trans_buf->data[0];
				if(value == USB_PWR_ON){
					GPIO_SetBits(GPIOB, GPIO_Pin_4);	
				}else if(value == USB_PWR_OFF){
					GPIO_ResetBits(GPIOB, GPIO_Pin_4);
				}
			}
			if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_4)){
				value = USB_PWR_ON;
			}else{
				value = USB_PWR_OFF;
			}
			trans_send.data[0] =  value;

			trans_send.length = 1;
			
			break;
		case USB_PWR_5:
			
			if(type == US_USB_PWR_WRITE){
				value = trans_buf->data[0];
				if(value == USB_PWR_ON){
					GPIO_SetBits(GPIOB, GPIO_Pin_5);	
				}else if(value == USB_PWR_OFF){
					GPIO_ResetBits(GPIOB, GPIO_Pin_5);
				}
			}
			if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_5)){
				value = USB_PWR_ON;
			}else{
				value = USB_PWR_OFF;
			}
			trans_send.data[0] =  value;
			trans_send.length = 1;
			
			break;
		case USB_PWR_6:
			
			if(type == US_USB_PWR_WRITE){
				value = trans_buf->data[0];
				if(value == USB_PWR_ON){
					GPIO_SetBits(GPIOB, GPIO_Pin_8);	
				}else if(value == USB_PWR_OFF){
					GPIO_ResetBits(GPIOB, GPIO_Pin_8);
				}
			}
			
			if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_8)){
				value = USB_PWR_ON;
			}else{
				value = USB_PWR_OFF;
			}
			trans_send.data[0] =  value;
			trans_send.length = 1;
			
			break;
		case MCU_DEV_ID:
			
			memcpy(mcu_id_v.MCU_Version, MCU_VERSION, 8);
			if(*(u32*)(STM32FLASH_EN_ID_START_ADDR) != 0xffffffff){
				mcu_id_v.MCU_V[0] = '@';
			}else{
				mcu_id_v.MCU_V[0] = 0;
			};
			memcpy(mcu_id_v.CPUID, (unsigned char*)CpuID, MCU_ID_SIZE);
			
			memcpy(trans_send.data, (unsigned char*)&mcu_id_v, sizeof(US_MCU_ID_GET));
			trans_send.length = sizeof(US_MCU_ID_GET);
			
			break;
		default:
			break;
			
	}
	
	trans_send.cmd_id		= trans_buf->cmd_id;
	trans_send.dev		= trans_buf->dev;
	trans_send.status		= 0;
	us_mcu_rc_buff_enter(type, (unsigned char*)&trans_send, sizeof(US_DEV_TRANS));


	return ret;
}

int us_dev_trans_config(unsigned char type, US_DEV_TRANS *trans_buf)
{
	int ret = 0;
	unsigned char dev = trans_buf->dev;
	US_DEV_TRANS trans_send = {0};

	if(dev == MCU_DEV_CTRL){
		if(type == US_DEV_CFG_READ){
			memcpy(trans_send.data, DEV_STATUS_CTRL, US_DEV_NUM);
			trans_send.length		= US_DEV_NUM;		
			trans_send.status		= 0;
		}else if(dev == US_DEV_CFG_WRITE){
			memcpy(DEV_STATUS_CTRL, trans_send.data, US_DEV_NUM);
			trans_send.length		= 1;		
			trans_send.status		= 0;
		}
		
	}else{
		if(DEV_STATUS_CTRL[dev]	!= DEV_NULL){
			if(trans_buf->data[0] == DEV_OFF){
				DEV_STATUS_CTRL[dev] 	= 	trans_buf->data[0];
			}else if(trans_buf->data[0] 		== 	DEV_ON){
				DEV_STATUS_CTRL[dev] 	= 	trans_buf->data[0];
			}
			trans_send.status	= 0;
		}else{
			trans_send.status	= 5;
		}
		
		trans_send.length		= 1;
	}

	trans_send.cmd_id		= trans_buf->cmd_id;
	trans_send.dev		= trans_buf->dev;
	us_mcu_rc_buff_enter(type, (unsigned char*)&trans_send, sizeof(US_DEV_TRANS));

	return ret;
}
void delay_ms(u16 time)
{        
	u16 i=0;     
	while(time--)    { 
		i=12000; 
		while(i--){}  
	}
}

void us_mcu_reset(void)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_14);
	delay_ms(1000);
	__disable_fault_irq();   
    NVIC_SystemReset();
}

void us_android_reset(void)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_14);
	delay_ms(2000);
	GPIO_SetBits(GPIOB, GPIO_Pin_14);						//S812_POWER_EN	
}

int us_mcu_input_data(unsigned char *input, unsigned int *length)
{
	int ret = 0, j = 0;
	unsigned char *input_data = input;
	unsigned int flash_RSA_code[12] = {0};

	if(input == NULL || length == NULL){
		ret = -1;
	}

	if(!ret){
		for(j = 0; j < 4; j++){
			flash_RSA_code[j] = *(u32*)(STM32FLASH_EN_ID_START_ADDR + j*4);
		}		
		
		memcpy(input_data, (unsigned char *)CpuID, MCU_ID_SIZE);
		memcpy(input_data + MCU_ID_SIZE, (unsigned char*)flash_RSA_code, RSA_KEY_BIT_SIZE/8);
		memcpy(input_data + MCU_ID_SIZE + RSA_KEY_BIT_SIZE/8, coder_id, TOKEN_SIZE);

		*length =  ENCODE_SIZE;
	}

	if(ret < 0){
		us_dev_error(MCU_CONFIG, (unsigned char *)__func__, strlen(__func__) + 1, ret);
	}	

	return ret;
}

unsigned int write_flash[24];
US_MCU_CONFIG_LIST conf_list;
unsigned int Usart_BaudRate = 9600;
extern unsigned int Local_Timer;

int us_mcu_config_store(int cfg_num, unsigned char *data, int size)
{
	int ret = 0, i = 0;
	unsigned int config_temp[10];
	
	if(cfg_num < 0 || data == NULL || size <= 0){
		ret = -1;
	}
	if(!ret){
		memcpy(config_temp, (unsigned char *)(IMAGE_VERSION_MAIN_ADDR), MAX_CONFIG_LENGTH);
		memcpy((unsigned char *)(config_temp + cfg_num), data, size);

		FLASH_Unlock();
		FLASH_ErasePage(IMAGE_VERSION_MAIN_ADDR);	
		
		for(i = 0; i < MAX_CONFIG_LENGTH/(sizeof(int)) ; i++){
			if( FLASH_WaitForLastOperation(100000) != FLASH_TIMEOUT ){
				FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);
			}
			FLASH_ProgramWord(IMAGE_VERSION_MAIN_ADDR + 4*i, *(config_temp+i));
		}

		FLASH_Lock();
	}
	
	return ret;
	
}

int us_mcu_config_load(int cfg_num, unsigned char *data, int byte)
{
	int ret = 0;
	unsigned int config_temp[10];
	
	if(cfg_num < 0 || data == NULL || byte <= 0){
		ret = -1;
	}
	if(!ret){
		memcpy(config_temp, (unsigned char *)(IMAGE_VERSION_MAIN_ADDR), MAX_CONFIG_LENGTH);
	}
	if(!ret){
		memcpy(data, (unsigned char *)(config_temp + cfg_num), byte);
	}
		
	return ret;
	
}


int us_mcu_config(unsigned char type, US_DEV_TRANS *trans_buf)
{
	int ret = 0;
	US_DEV_TRANS trans_send = {0};

	if(type == US_MCU_CONF_READ){
		conf_list.ACC_PWR_EN 	= *(unsigned int *)US_ACC_CONFIG;
		conf_list.BaudRate 	= Usart_BaudRate;
		conf_list.Local_Time 	= Local_Timer;
		conf_list.HungryDog 	= Hungry_Dog;
		conf_list.HungryTimer	= Hungry_timer;
		conf_list.Light_WakeUp	= Light_WakeUp;
		conf_list.LCD_MODE	= LCD_MODE;

		
		trans_send.cmd_id		= trans_buf->cmd_id;
		trans_send.dev		= trans_buf->dev;
		trans_send.status		= 0;
		trans_send.length		= sizeof(US_MCU_CONFIG_LIST);
		memcpy(trans_send.data, &conf_list, sizeof(US_MCU_CONFIG_LIST));
		us_mcu_rc_buff_enter(type, (unsigned char*)&trans_send, sizeof(US_DEV_TRANS));

		
	}else if(type == US_MCU_CONF_WRITE){
		memcpy((unsigned char *)&conf_list, trans_buf->data, sizeof(US_MCU_CONFIG_LIST));
		
		if(conf_list.BaudRate != 0){
			Usart_BaudRate = conf_list.BaudRate;
			USART_Configuration(conf_list.BaudRate);
		}

		Local_Timer = conf_list.Local_Time;

		if((conf_list.Light_WakeUp == TAG_ON) || (conf_list.Light_WakeUp == TAG_OFF)){
			Light_WakeUp = conf_list.Light_WakeUp;
		}

		if((conf_list.ACC_PWR_EN == 1) || (conf_list.ACC_PWR_EN == 2)){
			
			ACC_PWR = conf_list.ACC_PWR_EN;

			us_mcu_config_store(ACC_CONFIG, (unsigned char *)&(conf_list.ACC_PWR_EN), sizeof(conf_list.ACC_PWR_EN));
		}

		if((conf_list.HungryDog == 0) || 
			((conf_list.HungryDog >= MIN_HUNGRY_TIME_S) &&
			(conf_list.HungryDog <= MAX_HUNGRY_TIME_S))){
			
			Hungry_Dog_Lock = DATA_LOCK;
			
			delay_ms(30);

			Hungry_Dog		= 0;
			Hungry_ch		= 0;
			Hungry_timer	= TIMER_STOP;
			Hungry_Dog 		= conf_list.HungryDog;
			
			Hungry_Dog_Lock = DATA_UNLOCK;
			
		}
		
		if((conf_list.LCD_MODE == US_MANU) || (conf_list.LCD_MODE == US_AUTO)){
			
			LCD_MODE = conf_list.LCD_MODE;

		}
	}
	
	return ret;
}



unsigned char flash_cache[PAGE_SIZE+100];
unsigned char *addr_ptr;

unsigned int flash_get_data_size = 0;

int us_mcu_flash_load(unsigned char type, US_FLASH_TRANS *trans)
{
	int ret  = 0;
	
	addr_ptr = (unsigned char *)(MCU_FLASH_DATA_ADDR+trans->address);

	memcpy(flash_cache, addr_ptr, PAGE_SIZE);

	return ret;
}

int us_mcu_flash_store(unsigned char type, US_FLASH_TRANS *trans)
{
	int ret  = 0, i = 0;

	FLASH_Unlock();

	FLASH_ErasePage(MCU_FLASH_DATA_ADDR+trans->address);

	for (i = 0; i < (PAGE_SIZE /4); i++){
		if( FLASH_WaitForLastOperation(100000) != FLASH_TIMEOUT ){
			FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);
		}
		FLASH_ProgramWord(MCU_FLASH_DATA_ADDR + trans->address + i * 4, *((unsigned int *)(flash_cache + i*4)));
	}
	
	FLASH_Lock();

	return ret;
}

unsigned long long FILL_CHECKSUM = 0;
US_FLASH_TRANS flash_data;

int us_mcu_flash_fill(unsigned char type, US_FLASH_TRANS *trans)
{
	int ret  = 0;//, i = 0;

	US_FLASH_TRANS *flash_trans = &flash_data;
	
	memcpy(flash_trans, trans, sizeof(US_FLASH_TRANS));
	
	if(flash_get_data_size == 0){
		//FILL_CHECKSUM = 0;
	}
	
	if(flash_get_data_size <  flash_trans->total_length){
		memcpy(flash_cache + flash_get_data_size, flash_trans->data, flash_trans->send_length);
		flash_get_data_size += flash_trans->send_length;
	}

	if(flash_get_data_size == flash_trans->total_length){
		flash_get_data_size = 0;
		//for(i = 0 ; i < flash_trans->total_length ; i++){
		//	FILL_CHECKSUM+=flash_cache[i];
		//}
		//if(FILL_CHECKSUM != flash_trans->v_sum){
		//	us_dev_error(MCU_FLASH, (unsigned char *)__func__, strlen(__func__) + 1, 1);
		//} else {
		//	us_dev_error(MCU_FLASH, (unsigned char *)__func__, strlen(__func__) + 1, 0);
		//}
		//FILL_CHECKSUM = 0;
	}


	return ret;
}

unsigned long long FETCH_CHECKSUM = 0;
unsigned int fetch_send_length = 0;
int us_mcu_flash_fetch(unsigned char type, US_FLASH_TRANS *trans)
{
	int ret  = 0;
	int i = 0, num = 0;//, j = 0;
	US_FLASH_TRANS data = {0};
	unsigned char *addr_ptr = NULL;

	addr_ptr = flash_cache;
	
	data.total_length = trans->total_length;


	num = data.total_length/UPDATA_PKG_LENGTH;
	for(i = 0; i < num; i++){
		data.send_length = UPDATA_PKG_LENGTH;
		memcpy(data.data, addr_ptr + ( i * UPDATA_PKG_LENGTH ), UPDATA_PKG_LENGTH);
		//for(j = 0; j < data.send_length; j++){
		//	FETCH_CHECKSUM+=data.data[j];
		//}
		//data.v_sum = FETCH_CHECKSUM;
		
		us_mcu_rc_buff_enter(US_MCU_FLASH_READ, (unsigned char*)&data, sizeof(US_FLASH_TRANS));
		fetch_send_length+=UPDATA_PKG_LENGTH;
	}

	if(data.total_length % UPDATA_PKG_LENGTH != 0){
		data.send_length = data.total_length % UPDATA_PKG_LENGTH;
		memcpy(data.data, addr_ptr + ( i * UPDATA_PKG_LENGTH ), data.send_length);
		//for(j = 0; j < data.send_length; j++){
		//	FETCH_CHECKSUM+=data.data[j];
		//}
		//data.v_sum = FETCH_CHECKSUM;

		us_mcu_rc_buff_enter(US_MCU_FLASH_READ, (unsigned char*)&data, sizeof(US_FLASH_TRANS));
		fetch_send_length += data.send_length;
	}

	if(fetch_send_length == data.total_length){
		fetch_send_length = 0;
		//FETCH_CHECKSUM = 0;
	}
	
	return ret;
}

//young
int us_mcu_aes_trans(unsigned char type, US_DEV_TRANS *trans_buf)
{
	int ret = 0;
	unsigned char input[64] = {0}, output[64] = {0}, out_test[64] = {0};
	unsigned int input_length = 0, output_length;

	US_DEV_TRANS trans_send = {0};

	if(trans_buf == NULL){
		ret = -1;
	}
	
	if(!ret){
		if(us_mcu_input_data(input, &input_length) < 0){
			ret = -2;
		}
	}
	if(!ret){
		if(MCUAuthenticateEncode(input, output, out_test) < 0){
			ret = -3;
		}
	}

	if(!ret){
		trans_send.length		= 32;
		trans_send.cmd_id		= trans_buf->cmd_id;
		trans_send.dev			= trans_buf->dev;
		trans_send.status		= 0;

		memcpy(trans_send.data, out_test, 32);
		us_mcu_rc_buff_enter(type, (unsigned char*)&trans_send, sizeof(US_DEV_TRANS));
	}

	if(ret < 0){
		us_dev_error(MCU_CONFIG, (unsigned char *)__func__, strlen(__func__) + 1, ret);
	}

	return ret;
	
}

int us_mcu_write_coderid(unsigned char type, US_DEV_TRANS *trans_buf)
{
	int ret = 0;

	coder_length = trans_buf->length;
	memcpy(coder_id, trans_buf->data, trans_buf->length);

	return ret;
}

int us_mcu_rsa_config(unsigned char type, US_DEV_TRANS *trans_buf)
{
	int ret = 0, n = 0;
	unsigned char rsa_length = trans_buf->length;
	
	memcpy((unsigned char *)write_flash, trans_buf->data, rsa_length);

	FLASH_Unlock();
	FLASH_ErasePage  (STM32FLASH_EN_ID_START_ADDR);
		
	for(n = 0;n < rsa_length/4; n++){
		if( FLASH_WaitForLastOperation(100000) != FLASH_TIMEOUT )
		{
			FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);
		}
		FLASH_ProgramWord(STM32FLASH_EN_ID_START_ADDR+4*n, write_flash[n]);
	}
	FLASH_Lock();

	return ret;
}




int us_dev_host_id_get(unsigned char *data, unsigned char length)
{
	int ret = 0;
	US_HOST_PING *ping_data = (US_HOST_PING *)data;
	
	if(length == sizeof(US_HOST_PING)){
		memcpy(ping_s.id, ping_data->id, ping_data->id_length);
		
		ping_s.id_length 	= ping_data->id_length;
		ping_s.link 		= ping_data->link;
		ping_s.host_status = ping_data->host_status;
		
		ping_time = timer;
	}

	return ret;
}
int us_dev_init(void)
{
	int ret = 0;

	/*DEV GPIO*/
		/*GPIO IN*/
		DEV_STATUS_CTRL[GPIO_IN_0] 			= DEV_ON;
		/*GPIO OUT*/
		DEV_STATUS_CTRL[GPIO_OUT_0] 		= DEV_ON;
		DEV_STATUS_CTRL[GPIO_OUT_1] 		= DEV_ON;
		DEV_STATUS_CTRL[GPIO_OUT_2] 		= DEV_ON;
		DEV_STATUS_CTRL[GPIO_OUT_3] 		= DEV_ON;
		DEV_STATUS_CTRL[GPIO_OUT_4] 		= DEV_ON;
		DEV_STATUS_CTRL[GPIO_OUT_5] 		= DEV_ON;
		DEV_STATUS_CTRL[GPIO_OUT_6] 		= DEV_ON;
	/*DEV USART*/
		DEV_STATUS_CTRL[USART_0]			= DEV_ON;
		DEV_STATUS_CTRL[USART_1]			= DEV_OFF;
	/*DEV USB PWR*/
		DEV_STATUS_CTRL[USB_PWR_1]		= DEV_ON;
		DEV_STATUS_CTRL[USB_PWR_2]		= DEV_ON;
		DEV_STATUS_CTRL[USB_PWR_3]		= DEV_ON;
		DEV_STATUS_CTRL[USB_PWR_4]		= DEV_ON;
		DEV_STATUS_CTRL[USB_PWR_5]		= DEV_ON;
	/*DEV SIGNAL*/
		DEV_STATUS_CTRL[SIGNAL_IO_1]		= DEV_ON;
		DEV_STATUS_CTRL[SIGNAL_IO_2]		= DEV_ON;
		DEV_STATUS_CTRL[SIGNAL_IO_3]		= DEV_ON;
		DEV_STATUS_CTRL[SIGNAL_IO_4]		= DEV_ON;
		DEV_STATUS_CTRL[SIGNAL_IO_5]		= DEV_ON;

		DEV_STATUS_CTRL[SIGNAL_IO_DATA]	= DEV_ON;
		DEV_STATUS_CTRL[MCU_DEV_CTRL]		= DEV_ON;

	return ret;
}

int us_mcu_hungry_dog_init(void)
{
	int ret = 0, hd_temp = 0;
	
	Hungry_Dog_Lock = DATA_LOCK;
	
	us_mcu_config_load(HUNGRY_DOG_CFG, (unsigned char *)&hd_temp, 4);
	if( 	(hd_temp 	<= MAX_HUNGRY_TIME_S) && 
		(hd_temp 	>= MIN_HUNGRY_TIME_S)){
		Hungry_Dog = hd_temp;
	}else{
		Hungry_Dog = 0;
	}
	
	hd_temp = 0;
	us_mcu_config_store(HUNGRY_DOG_CFG, (unsigned char *)&hd_temp, sizeof(hd_temp));

	Hungry_Dog_Lock = DATA_UNLOCK;
	
	return ret;
}

#if 1
void Sys_Standby(void) //�ڻ��ѳ�ʼ���е���
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	PWR_WakeUpPinCmd(ENABLE);
	PWR_EnterSTANDBYMode();
}

void Sys_Enter_Standby(void)
{
	RCC_APB2PeriphResetCmd(0X01FC,DISABLE);
	Sys_Standby();
}

//Stop Mode
void Sys_Enter_Stop(void)
{
	RCC_APB2PeriphResetCmd(0X01FC,DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);  //ʹ��Pwrʱ��
	PWR_EnterSTOPMode (PWR_Regulator_LowPower,PWR_STOPEntry_WFI);
}
uint8_t clock_source_wakeup = 0;
RCC_ClocksTypeDef clock_status_wakeup;

static void SYSCLKConfig_STOP(void)
{
	RCC_HSEConfig(RCC_HSE_ON);
	while(RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);
	RCC_PLLCmd(ENABLE);
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	while(RCC_GetSYSCLKSource() != 0X08);
}
uint8_t clock_source_config;
RCC_ClocksTypeDef clock_status_config;

void Sys_Stop_Mode_Wakeup(void)
{
	clock_source_wakeup = RCC_GetSYSCLKSource();

	RCC_GetClocksFreq(&clock_status_wakeup);
	SYSCLKConfig_STOP();
	clock_source_config = RCC_GetSYSCLKSource();

	RCC_GetClocksFreq(&clock_status_config);
}

#endif


extern uint8_t USB_Receive_Buffer[REPORT_COUNT];

void us_mcu_recave(void)
{
	unsigned char type = 0;
#ifndef STM32F10X_CL
	PMAToUserBufferCopy((unsigned char *)recv_ptr, ENDP1_RXADDR, REPORT_COUNT);
	SetEPRxStatus(ENDP1, EP_RX_VALID);
#else
	USB_SIL_Read(EP1_OUT,(unsigned char *)recv_ptr);
#endif

	if(1){
		if(recv_ptr->checksum == us_uart_checksum(recv_ptr)){
			type = recv_ptr->type;
			
			switch(type){
				case US_HOST_ID:
					us_dev_host_id_get(recv_ptr->data, recv_ptr->length);
					break;
				case US_GPIO_READ:
				case US_GPIO_WRITE:
				case US_USART_WRITE:
				case US_USB_PWR_READ:
				case US_USB_PWR_WRITE:
				case US_SINGNAL_DATA:
				case US_MCU_ID_READ:
					us_dev_trans_decoder(type, (US_DEV_TRANS *)recv_ptr->data);
					break;
				case US_GPIO_CONFIG:
				case US_USART_CONFIG:
				case US_USB_PWR_CONFIG:
				case US_SIGNAL_CONFIG:
				case US_DEV_CFG_READ:
				case US_DEV_CFG_WRITE:
					us_dev_trans_config(type, (US_DEV_TRANS *)recv_ptr->data);
					break;
				case US_MCU_CONF_WRITE:
				case US_MCU_CONF_READ:
					us_mcu_config(type, (US_DEV_TRANS *)recv_ptr->data);
					break;
				case US_MCU_FLASH_LOAD:
					us_mcu_flash_load(type, (US_FLASH_TRANS *)recv_ptr->data);
					break;
				case US_MCU_FLASH_STORE:
					us_mcu_flash_store(type, (US_FLASH_TRANS *)recv_ptr->data);
					break;
				case US_MCU_FLASH_FILL:
					us_mcu_flash_fill(type, (US_FLASH_TRANS *)recv_ptr->data);
					break;	
				case US_MCU_FLASH_FETCH:
					us_mcu_flash_fetch(type, (US_FLASH_TRANS *)recv_ptr->data);
					break;
				
				#if 1
				case US_MCU_AES_READ:
					us_mcu_aes_trans(type, (US_DEV_TRANS *)recv_ptr->data);
					break;
				case US_MCU_AES_WRITE:
					us_mcu_write_coderid(type, (US_DEV_TRANS *)recv_ptr->data);
					break;
				case US_MCU_RSA_LOCK_CONFIG:
					us_mcu_rsa_config(type, (US_DEV_TRANS *)recv_ptr->data);
					break;	
				#endif
				
				case US_MCU_RESET:
					us_mcu_reset();
					break;
				case US_MCU_STANDBY:
					Sys_Enter_Standby();
					//STOP_MODE = 1;
					//SYSCLKConfig_STOP();
					//__WFI();
					break;
			};
			
		}else{
			//us_dev_error(MCU_CONFIG, (unsigned char *)__func__, strlen(__func__) + 1, type);	
		}
	}

	return;
}

