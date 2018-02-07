#include <stdio.h>
#include <stdio.h>
#include <string.h>


#include "libMCUAuthenticate.h"
#include "us_mcu_transfer.h"
#include "AES_CBC.h"	
#define US_MCU

#define RSA_KEY_BIT_SIZE	128
#define MCU_ID_SIZE				12
#define TOKEN_SIZE				4
#define ENCODE_SIZE				(MCU_ID_SIZE + RSA_KEY_BIT_SIZE/8 + TOKEN_SIZE)


//128bits key.
/* Key to be used for AES encryption/decryption */
unsigned char Host_Key[16] =
{
	0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
	0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c
};

/* Initialization Vector, used only in non-ECB modes */
unsigned char Host_IV[16] =
{
	0xf0 , 0xf1 , 0xf2 , 0xf3 , 0xf4 , 0xf5 , 0xf6 , 0xf7,
	0xf8 , 0xf9 , 0xfa , 0xfb , 0xfc , 0xfd , 0xfe , 0xff
};


int MCUAuthenticateEncode(const unsigned char* inputData,
								unsigned char* outputData, unsigned char * outputDataSize )
{
	int ret = 0;
	struct AES_ctx ctx;
	
	AES_init_ctx_iv(&ctx, Host_Key, Host_IV);
	AES_CBC_encrypt_buffer(&ctx, (uint8_t*)inputData, (uint8_t*)outputData, 32);

//	AES_init_ctx_iv(&ctx, Host_Key, Host_IV);
//	AES_CBC_decrypt_buffer(&ctx, (uint8_t*)outputData, (uint8_t*)outputDataSize, 32);

	if(ret < 0){
		us_dev_error(MCU_DEV_ID, (unsigned char *)__func__, strlen(__func__) + 1, ret);
	}	

	return ret;
}

