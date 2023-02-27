/*********************************************************************************
 提供数组位相关操作API。
*********************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"

#include "bitarray.h"

// ---------------------------------------------------------数组位操作
#define BITARRAY_BIT_OP(array, bit, op) ((array)[(bit) / (sizeof((array)[0]) * 8)] op (1 << ((bit) % (sizeof((array)[0]) * 8))))

/**********************************************************************
函数名称：bitArrayGet
函数功能：获取数组位
函数形参：数组，位
函数返回值：数组位
函数描述：None 
**********************************************************************/
bool bitArrayGet(const void *array, unsigned bit)
{
    return BITARRAY_BIT_OP((uint32_t*)array, bit, &);
}

/**********************************************************************
函数名称：bitArraySet
函数功能：设置数组位
函数形参：数组，位
函数返回值：None  
函数描述：None 
**********************************************************************/
void bitArraySet(void *array, unsigned bit)
{
    BITARRAY_BIT_OP((uint32_t*)array, bit, |=);
}

/**********************************************************************
函数名称：bitArraySet
函数功能：清除数组位
函数形参：数组，位
函数返回值：None  
函数描述：None 
**********************************************************************/
void bitArrayClr(void *array, unsigned bit)
{
    BITARRAY_BIT_OP((uint32_t*)array, bit, &=~);
}

/**********************************************************************
函数名称：bitArrayXor
函数功能：异或数组位
函数形参：目标，大小，源1，源2
函数返回值：None  
函数描述：None 
**********************************************************************/
void bitArrayXor(void *dest, size_t size, void *op1, void *op2)
{
    for (size_t i = 0; i < size; i++) {
        ((uint8_t*)dest)[i] = ((uint8_t*)op1)[i] ^ ((uint8_t*)op2)[i];
    }
}


