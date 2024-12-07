#ifndef __AES_H__
#define __AES_H__
#include "stdio.h"
typedef unsigned char uint8_t;

// ECB密码本模式
#ifndef ECB
#define ECB
#endif

#define AES128

#define AES_DATA_LEN 16
#define AES_KEY_LEN 16
#define AES_ROUND_KEY_LEN 176



void cipher(uint8_t *state, uint8_t *key);
void invcipher(uint8_t *state, uint8_t *key);
#endif // ! __AES_H__
