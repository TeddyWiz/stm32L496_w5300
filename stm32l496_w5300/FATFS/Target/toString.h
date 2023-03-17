#ifndef __TOSTRING_H
#define __TOSTRING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

//void addIntValToString1(uint8_t *value, char* writeBuffer, char** pCurrent);
/**
 * @brief 
 * 
 * @param value 
 * @param writeBuffer 
 * @param pCurrent 
 */
void addIntValToString(uint8_t *value, char* writeBuffer, char** pCurrent);

/**
 * @brief 
 * 
 * @param value 
 * @param writeBuffer 
 * @param pCurrent 
 */
void addStrValToString(char *value, char* writeBuffer, char** pCurrent);
	
//void addHexValToString(char *value, char* writeBuffer, char** pCurrent);
/**
 * @brief  int를 16 bit Hex값으로 변경
 * 
 * @param value int data
 * @param hex  0xzzzz
 */
void intToHex16(int value, char hex[]);

/**
 * @brief int를 8bit Hex값으로 변경
 * 
 * @param value int datda
 * @param hex 0xzz
 */
void intToHex8(int value, char hex[]);

/**
 * @brief string을 Hex값으로 변경
 * 
 * @param input String
 * @param output Hex
 */
void string2hex(char* input, char* output);

/**
 * @brief Num만큼 Hex[num]에 저장
 * 
 * @param value Int
 * @param hex  Hex
 * @param num 변환할 갯수
 */
void int2hex(int value, char hex[], int num);

/**
 * @brief char 를 hex로 변경
 * 
 * @param stringData 변경할 데이터
 * @return 
 */
char *char2hex(char *stringData);

/**
 * @brief Hex를 int로 변경
 * 
 * @param stringData 변경할 데이터
 * @return 
 */
int hex2int(char *stringData);

/**
 * @brief int를 Char로 변경
 * 
 * @param val int 
 * @return 
 */
 
int float2char(float val, char* result);

/**
 * @brief 
 * 
 * @param value 
 * @return char* 
 */
char *intToChar2(int value);

/**
 * @brief 
 * 
 * @param val 
 * @param result 
 * @return int 
 */
int int2char(int val, char* result);

/**
 * @brief Char 를 Ascii로 변경
 * 
 * @param stringData 변경할 데이터
 * @return 
 */
char *chartoAscii(char *stringData);

/**
 * @brief 
 * 
 * @param HexData 
 * @param lens 
 * @param StrData 
 * @return int 
 */
int Hex2String(char *HexData, int lens, char *StrData);

/**
 * @brief string을 int형으로 별환한다.
 * 
 * @param str 
 * @return int 
 */
int str2int(char *str);
#endif