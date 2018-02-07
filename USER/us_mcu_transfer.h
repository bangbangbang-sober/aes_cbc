#ifndef _CP_EXT_RC_BUFFER_H
#define _CP_EXT_RC_BUFFER_H

#include "stm32f10x.h"

#include "usb_desc.h"

#define ERROR 							-1
#define OK								0
#define MAX_BUFF_SIZE					64

/*Can info*/
//Define

#define MAX_CAN_DATA_LENGTH			0x81

#define MAX_DATA_LENGTH				51
#define MAX_UART_LENGTH				MAX_DATA_LENGTH + 5
#define MAX_VERSION_LENGTH				12

#define MAX_TYPE_NUM					12

#define MCU_ID_SIZE						12
#define MCU_CODER_SIZE					48


#define RSA_KEY_BIT_SIZE				128
#define TOKEN_SIZE						4
#define ENCODE_SIZE						(MCU_ID_SIZE + RSA_KEY_BIT_SIZE/8 + TOKEN_SIZE)



/*Uart send*/
#define UART_HEAD						0x2E

/*Uart send type*/
#define US_MCU_ERR						0x01					//SEND ERROR CODE

#define US_CAN_DATA						0x0A
#define US_RED_DATA						0x0B
#define US_CAN_DATA_TEST				0x0C
#define US_ADC_DATA						0x0D
#define US_GPIO_DATA					0x0E
#define US_CPUID_DATA					0x0F
#define US_MCU_PING						0x1A
#define US_MCU_CONFIG					0x1B
#define US_HOST_ID						0x1C
#define US_ID_CODER_DATA				0x1D
#define US_ID_CLEAR_DATA				0x1E

#define US_GPIO_READ					0x1F
#define US_GPIO_WRITE					0x20
#define US_GPIO_CONFIG					0x21

#define US_USART_READ					0x22
#define US_USART_WRITE					0x23
#define US_USART_CONFIG					0x24

#define US_SINGNAL_DATA					0x25
#define US_SIGNAL_CONFIG				0X26

#define US_USB_PWR_READ					0x27
#define US_USB_PWR_WRITE				0x28
#define US_USB_PWR_CONFIG				0x29

#define US_DEV_CFG_READ					0x30
#define US_DEV_CFG_WRITE				0x31

#define US_MCU_AES_READ					0x32		//AES Read
#define US_MCU_AES_WRITE				0x33		//AES Write
#define US_MCU_AES_CONFIG				0x34


#define US_MCU_ID_READ					0x35

#define US_MCU_RSA_LOCK_WRITE			0X36
#define US_MCU_RSA_LOCK_READ			0X37
#define US_MCU_RSA_LOCK_CONFIG			0X38


#define US_MCU_UPDATA_READ				0X39
#define US_MCU_UPDATA_WRITE				0X40
#define US_MCU_UPDATA_CONFIG			0X41


#define US_USART_GPS_READ				0x42
#define US_USART_GPS_WRITE				0x43
#define US_USART_GPS_CONFIG				0x44

#define US_MCU_CONF_WRITE				0x45
#define US_MCU_CONF_READ				0x46



#define US_MCU_FLASH_WRITE				0X52
#define US_MCU_FLASH_READ				0X53
#define US_MCU_FLASH_CONFIG				0X54

#define US_MCU_FLASH_LOAD				0X55
#define US_MCU_FLASH_STORE				0X56
#define US_MCU_FLASH_FILL				0X57
#define US_MCU_FLASH_FETCH				0X58

#define US_MCU_RESET					0X59
#define US_MCU_STANDBY					0X60

/*CAR INFO*/
#define US_BACK_CAR						0xC1
#define US_TIRE_CORNER_INFO				0xC2

/*ID CODER*/

#define idN								32387
#define idD								15685
#define EN_SIZE							12

#define CTRL_NUM						3
#define DEV_NUM							17

#define MAX_TRANS_INFO					40		
#define UPDATA_PKG_LENGTH				(MAX_TRANS_INFO - 12)

#define SIG_PIN1							GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)//Left_IN
#define SIG_PIN2 							GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1)//Right_IN
#define SIG_PIN3 							GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4)//Reverse_IN

#define SIG_PIN4 							GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)
#define SIG_PIN5 							GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) //ACC


#define US_DEV_NUM						40	

#define MAX_ID_SIZE						16

/*MCU ADDR***************************************************************/
#define IMAGE_VERSION_MAIN_ADDR			0x08009800
#define IMAGE_VERSION_UPDATA_ADDR		0x08009804
#define IMAGE_UPDATA_SIZE_ADDR			0x08009808
#define US_ACC_CONFIG					0x0800980C

#define STM32FLASH_EN_ID_START_ADDR		0x08009810
	
#define IMAGE_MAIN_ADDR					0x08003400

#define IMAGE_UPDATA_ADDR				0x08009c00

#define USART_MAX_BUFF_SEND			512

#define MAX_USART_BUFF_NUM			10
#define MAX_UASRT_BUFF_LENGRH			30


#define ApplicationAddress    				IMAGE_MAIN_ADDR

#define MCU_FLASH_DATA_ADDR			0x08009800



#if defined (STM32F10X_MD) || defined (STM32F10X_MD_VL)
 #define PAGE_SIZE                         (0x400)    /* 1 Kbyte */
 #define FLASH_SIZE                        (0x20000)  /* 128 KBytes */
#elif defined STM32F10X_CL
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x40000)  /* 256 KBytes */
#elif defined STM32F10X_HD || defined (STM32F10X_HD_VL)
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x80000)  /* 512 KBytes */
#elif defined STM32F10X_XL
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x100000) /* 1 MByte */
#else 
 #error "Please select first the STM32 device to be used (in stm32f10x.h)"    
#endif

typedef  void (*pFunction)(void);

/*Function******************************************************************/
#define MIN_HUNGRY_TIME_S		(60*1)
#define MAX_HUNGRY_TIME_S		(60*3)

#define MAX_CONFIG_LENGTH		40
#define TIMER_STOP				9999	

/**************************************************************************/
typedef enum _US_DEV_CTRL{
	DEV_NULL = 0,
	DEV_ON  = 1,
	DEV_OFF = 2
}US_DEV_CTRL;


typedef enum _MCU_IO_VALUE{
	IO_DOWN = 1,
	IO_UP = 2,
	USB_PWR_ON = 3,
	USB_PWR_OFF = 4
}MCU_IO_VALUE;

typedef enum _MCU_ACC{
	ACC_CONF_NULL = 0,
	ACC_CONF_DIS = 1,
	ACC_CONF_EN = 2
}MCU_ACC;

typedef enum _US_PWR_TAG{
	TAG_NULL = 0,
	TAG_ON = 1,
	TAG_OFF = 2
}US_PWR_TAG;

typedef enum _US_SJS_MODE{
	US_AUTO = 0,
	US_MANU = 1
}US_SJS_MODE;



/*MCU DEVICE LIST*/
typedef enum _MCU_DEVICE_LIST{	
	/*MCU GPIO*/
	/*MCU GPIO IN DEV*/
	GPIO_IN_0 = 1,
	GPIO_IN_1 = 2,
	GPIO_IN_2 = 3,
	GPIO_IN_3 = 4,
	GPIO_IN_4 = 5,
	GPIO_IN_5 = 6,
	GPIO_IN_6 = 7,
	GPIO_IN_7 = 8,
	/*MCU GPIO OUT DEV*/
	GPIO_OUT_0 = 9,
	GPIO_OUT_1 = 10,
	GPIO_OUT_2 = 11,
	GPIO_OUT_3 = 12,
	GPIO_OUT_4 = 13,
	GPIO_OUT_5 = 14,
	GPIO_OUT_6 = 15,
	GPIO_OUT_7 = 16,
	/*MCU USART*/
	USART_0 = 17,
	USART_1 = 18,	
	/*MCU USB*/
	USB_PWR_1 = 19,
	USB_PWR_2 = 20,
	USB_PWR_3 = 21,
	USB_PWR_4 = 22,
	USB_PWR_5 = 23,
	USB_PWR_6 = 24,	
	/*MCU SINGAL*/
	SIGNAL_IO_1 = 25,
	SIGNAL_IO_2 = 26,
	SIGNAL_IO_3 = 27,
	SIGNAL_IO_4 = 28,
	SIGNAL_IO_5 = 29,
	SIGNAL_IO_6 = 30,
	SIGNAL_IO_7 = 31,
	SIGNAL_IO_8 = 32,
	SIGNAL_IO_DATA = 33,
	MCU_CONFIG = 34,
	MCU_DEV_CTRL = 35,
	MCU_DEV_LOCK = 36,	
	MCU_DEV_ID = 37,
	MCU_RSA_LOCK = 38,
	MCU_UPDATA = 39
}MCU_DEVICE_LIST;



typedef struct _CAR_INFO{
	unsigned long time;
	unsigned char type;
	unsigned char data[MAX_DATA_LENGTH];
}CAR_INFO;

typedef struct _US_HOST_PING{
	unsigned char id[MAX_ID_SIZE];
	unsigned char id_length;
	unsigned char link;
	unsigned char host_status;
	unsigned char mcu_status;
}US_HOST_PING;

typedef struct _UART_INFO{
	unsigned char head;
	unsigned char version[MAX_VERSION_LENGTH];
	unsigned char type;
	unsigned char length;
	unsigned char checksum;
	unsigned char data[MAX_UART_LENGTH];
}UART_INFO;

typedef struct _US_DEV_TRANS{
	unsigned char dev;
	unsigned char status;
	unsigned char length;
	unsigned char temp;
	unsigned long cmd_id;
	unsigned char data[MAX_TRANS_INFO];
}US_DEV_TRANS;

typedef struct _US_MCU_CONFIG_LIST{
	unsigned int BaudRate;
	unsigned int GPS_BaudRate;
	unsigned int ACC_PWR_EN;
	unsigned int Local_Time;
	int HungryDog;
	int HungryTimer;
	int Light_WakeUp;
	int LCD_MODE;
}US_MCU_CONFIG_LIST;



typedef enum _MCU_DEVICE_STATUS{
	MCU_DEV_INIT = 1,
	MCU_DEV_DECODE = 2,
	MCU_DEV_LINK = 3,
	MCU_DEV_UNLINK = 4,
	MCU_DEV_START = 5,
	MCU_DEV_STOP = 6,
	MCU_DEV_ERROR = 7
}MCU_STATUS;

typedef enum _MCU_HW_STATUS{
	MCU_HW_INIT = 0,
	MCU_HW_WAIT = 1,
	MCU_HW_START = 2,
	MCU_HW_WORK = 3
}MCU_HW_STATUS;

typedef struct _UART_RC_BUFF{
	unsigned char data[MAX_UASRT_BUFF_LENGRH];
	unsigned char length;
	unsigned char temp;
}UART_RC_BUFF;

typedef enum _LINK_STATUS{
	UNLINK = 0,
	LINK_OK = 1
}LINK_STATUS;

typedef struct _US_CODER_TRANS{
	unsigned char num;
	unsigned char send_length;
	unsigned char total_length;
	unsigned char temp;
	unsigned char data[UPDATA_PKG_LENGTH];
}US_CODER_TRANS;

typedef struct _US_UPDATA_TRANS{
	unsigned int version;
	unsigned int send_length;
	unsigned int total_length;
	unsigned char data[UPDATA_PKG_LENGTH];
}US_UPDATA_TRANS;

typedef struct _US_FLASH_TRANS{
	unsigned int id;
	unsigned int address;
	unsigned int send_length;
	unsigned int total_length;
	unsigned char data[UPDATA_PKG_LENGTH];
	unsigned int v_sum;
}US_FLASH_TRANS;

typedef enum FLASH_UPDATA_STATUS{
	FLASH_UNLOCK = 0,
	ERASE_FLASH_PAGE,
	FLASH_DOWNLOAD,
	FLASH_LOCK,
	RUN_FLASH_TEST
}FLASH_UPDATA_STATUS;

typedef struct _US_MCU_ID_GET{
	unsigned char MCU_Version[8];
	unsigned char MCU_V[4];
	unsigned char CPUID[12];
}US_MCU_ID_GET;

typedef enum _US_MCU_DATA_LOCK{
	DATA_UNLOCK = 0,
	DATA_LOCK = 1
}US_MCU_DATA_LOCK;

typedef enum _CONFIG_ADDR_LIST{
	MAIN_VERSION 			= 0,
	UPDATA_VERSION 		= 1,
	UPDATA_SIZE 			= 2,
	ACC_CONFIG 				= 3,
	RSA_CODE 				= 4,
	HUNGRY_DOG_CFG 		= 5,
	ANDROID_LAST_STATUS	= 6,
	MCU_RESET				= 7
}CONFIG_ADDR_LIST;

typedef enum _US_ANDROID_STATUS{
	ANDROID_INIT = 0,
	ANDROID_PWR_OFF = 1,
	ANDROID_PWR_ON = 2,
	ANDROID_PWR_WORKING = 3
}US_ANDROID_STATUS;

/*Function*/
int us_mcu_uart_coder(UART_INFO *send);
unsigned char us_uart_checksum(UART_INFO *recv_ptr);

/*buffer circle function*/
int us_mcu_rc_buff_init(void);
int us_mcu_uart_coder(UART_INFO *send);

int us_mcu_rc_buff_enter(unsigned char type, unsigned char *send, unsigned char buff_size);
int us_mcu_rc_buff_delete(UART_INFO *str);

int us_usart_rc_buff_enter(unsigned char data);
int us_usart_rc_buff_delete(unsigned char *data);


int us_dev_uart_send(unsigned char *send_buff, unsigned char length);
/*MCU USB*/


/*for test*/
//int us_mcu_test_back_car(UART_INFO *send, unsigned char pwr);
int us_mcu_test_tire_corner(UART_INFO *send, char data);

void us_mcu_recave(void);

/*CPU ID*/
int us_mcu_id_get(void);

int us_mcu_send_cpu_id(void);

int us_dev_init(void);
int us_mcu_hungry_dog_init(void);


/*US MCU Config*/
int us_mcu_config_store(int cfg_num, unsigned char *data, int size);
int us_mcu_config_load(int cfg_num, unsigned char *data, int byte);

/*US Func*/
void us_mcu_reset(void);
void us_android_reset(void);

int us_dev_error(unsigned char dev, unsigned char* func_name, int name_len, int status);

#endif
