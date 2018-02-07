#ifndef LIBMCUAUTHENTICATE_H
#define LIBMCUAUTHENTICATE_H

#ifdef __cplusplus
extern "C" {
#endif

int MCUAuthenticateEncode(const unsigned char* inputData,
								unsigned char* outputData, unsigned char* outputDataSize ); 

#ifdef __cplusplus
} // extern "C"
#endif

#endif /*LIBMCUAUTHENTICATE*/



