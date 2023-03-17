#include "toString.h"
#include "main.h"
#include <stdio.h>
#include <string.h>


/*********************************************************/
/*           ADD Int/Char* Value to char* Functions                               */
/*********************************************************/


void addIntValToString(uint8_t *value, char* writeBuffer, char** pCurrent)
{
	
	*pCurrent += sprintf(*pCurrent,"%d\n", *value );
	
	printf("[writeBuffer]\r\n[%s]\r\n", writeBuffer);
	
	//strcpy(writeBuffer+10, (char*)staticInfo->num);
}

void addStrValToString(char *value, char* writeBuffer, char** pCurrent)
{
	
	*pCurrent += sprintf(*pCurrent,"%s\n", value );
	
	printf("[writeBuffer]\r\n [%s]\r\n", writeBuffer);
	
	//strcpy(writeBuffer+10, (char*)staticInfo->num);
}

/*
void addHexValToString(char *value, char* writeBuffer, char** pCurrent)
{
	*pCurrent += sprintf(*pCurrent,"%s", value );
	
	//strcpy(writeBuffer+10, (char*)staticInfo->num);
}
*/

void intToHex16(int value, char hex[])
{
	sprintf(hex,"%04x",value);
	//printf("%s\r\n", hex);
	
}

void intToHex8(int value, char hex[])
{
	sprintf(hex,"%02x",value);
	//printf("%s\r\n", hex);
	
}

char *intToChar2(int value)
{
	char result[2];

	if(value<10)
	{
		result[0]='0';
		result[1]=value+'0';
	}
	else
	{
		result[0]= (int)(value/10)+'0';
		result[1]= (value%10)+'0';
	}
	result[2]=0;
	
	return result;
}

void int2hex(int value, char hex[], int num)
{
	unsigned char saucHex[] = "0123456789ABCDEF";
	for(int i=num-1; i>=0; i--) {
		hex[i]=saucHex[value%16];
		value/=16;
	}
	hex[num]=0;

}

void string2hex(char* input, char* output)
{
    int loop;
    int i; 
    
    i=0;
    loop=0;
    
    while(input[loop] != '\0')
    {
        sprintf((char*)(output+i),"%02X", input[loop]);
        loop+=1;
        i+=2;
    }
    //insert NULL at the end of the output string
    output[i++] = '\0';
}


char *char2hex(char *stringData)
{
	char result[32];
	int i;
	unsigned char saucHex[] = "0123456789ABCDEF";
	for(i=0; i<strlen(stringData); i++){
		result[2*i] = saucHex[stringData[i] >> 4];
		result[2*i+1] =saucHex[stringData[i] &0xF];
	}
	result[2*i]=0;
	return result;
}

int hex2int(char *stringData)
{
	int temp, time=0;
	for(int i=0; i<(strlen(stringData)/2); i++){
		temp=stringData[2*i+1];
		if(temp>='0' && temp <='9') time=time*10+temp-'0';
	}
	return time;

}

int float2char(float val, char* result)
{
	char temp[32];
	int count=0, i=0;
	float tmp_val=val;
	if(tmp_val<0){
		result[i++]='-';
		tmp_val=-tmp_val;
	}
	while(1){
		temp[count++]=((int)tmp_val)%10+'0';
		tmp_val=tmp_val/10;
		if(((int)tmp_val)==0) break;
	}
	
	for(int j=0; j<count; j++){
		result[i++]=temp[count-j-1];
	}
	
	result[i++]='.';
	result[i++]= ((int)(val*10))%10+'0';
	result[i++]= ((int)(val*100))%10+'0';
	result[i]=0;
	printf("float2char : %s\r\n", result);
	return i;
}

int int2char(int val, char* result)
{
	char temp[32];
	int count=0, i=0;
	if(val<0){
		result[i++]='-';
		val=-val;
	}
	while(1){
		temp[count++]=val%10+'0';
		val=val/10;
		if(val==0) break;
	}
	
	for(int j=0; j<count; j++){
		result[i++]=temp[count-j-1];
	}
	
	result[i]=0;
	printf("int2char : %s\r\n", result);
	return i;
}

char *chartoAscii(char *stringData)
{
	char result[30];
	int i;
	unsigned char saucHex[] = "0123456789ABCDEF";

	for(i=0; i<strlen(stringData); i++){
		result[2*i] = saucHex[stringData[i] >> 4];
		result[2*i+1] =saucHex[stringData[i] &0xF];
	}
	result[2*i]=0;
	return result;
}

int Hex2String(char *HexData, int lens, char *StrData)
{
    int i = 0;
    char *Temp_Str = StrData;
    char TransData =0;
    unsigned char saucHex[17] = "0123456789abcdef";
    int result = 0;
    for(i=0; i<lens; i++)
    {
        *Temp_Str++ = saucHex[(HexData[i] >> 4) & 0x0f];
        //Temp_Str++;
        result++;
        *Temp_Str++ = saucHex[HexData[i] & 0x0f];
        //Temp_Str++;
        result++;
    }
    return 0;
}

int str2int(char *str)
{
	int cnt=0;
	int val=0;
	while(1)
	{
		val=val+str[cnt++]-0x30;
		if(str[cnt]<'0' || str[cnt]>'9') break;
		val*=10;
	}
	return val;
}