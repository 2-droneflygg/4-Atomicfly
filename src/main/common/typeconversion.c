#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "maths.h"

#ifdef REQUIRE_PRINTF_LONG_SUPPORT

/**********************************************************************
函数名称：uli2a
函数功能：uli2a
函数形参：num，base，uc，bf
函数返回值：None
函数描述：None
**********************************************************************/
void uli2a(unsigned long int num, unsigned int base, int uc, char *bf)
{
    unsigned int d = 1;

    while (num / d >= base)
        d *= base;

    while (d != 0) {
        int dgt = num / d;
    *bf++ = dgt + (dgt < 10 ? '0' : (uc ? 'A' : 'a') - 10);

    // 下一个数字
        num %= d;
        d /= base;
    }
    *bf = 0;
}

/**********************************************************************
函数名称：li2a
函数功能：li2a
函数形参：num，bf
函数返回值：None
函数描述：None
**********************************************************************/
void li2a(long num, char *bf)
{
    if (num < 0) {
        num = -num;
        *bf++ = '-';
    }
    uli2a(num, 10, 0, bf);
}
#endif

/**********************************************************************
函数名称：ui2a
函数功能：ui2a
函数形参：num，base，uc，bf
函数返回值：None
函数描述：None
**********************************************************************/
void ui2a(unsigned int num, unsigned int base, int uc, char *bf)
{
    unsigned int d = 1;

    while (num / d >= base)
        d *= base;

    while (d != 0) {
        int dgt = num / d;
    *bf++ = dgt + (dgt < 10 ? '0' : (uc ? 'A' : 'a') - 10);

    // 下一个数字
        num %= d;
        d /= base;
    }
    *bf = 0;
}

/**********************************************************************
函数名称：i2a
函数功能：i2a
函数形参：num，bf
函数返回值：None
函数描述：None
**********************************************************************/
void i2a(int num, char *bf)
{
    if (num < 0) {
        num = -num;
        *bf++ = '-';
    }
    ui2a(num, 10, 0, bf);
}

/**********************************************************************
函数名称：a2d
函数功能：a2d
函数形参：ch
函数返回值：ch
函数描述：None
**********************************************************************/
int a2d(char ch)
{
    if (ch >= '0' && ch <= '9')
        return ch - '0';
    else if (ch >= 'a' && ch <= 'f')
        return ch - 'a' + 10;
    else if (ch >= 'A' && ch <= 'F')
        return ch - 'A' + 10;
    else
        return -1;
}

/**********************************************************************
函数名称：a2i
函数功能：a2i
函数形参：ch，src，base，nump
函数返回值：ch
函数描述：None
**********************************************************************/
char a2i(char ch, const char **src, int base, int *nump)
{
    const char *p = *src;
    int num = 0;
    int digit;
    while ((digit = a2d(ch)) >= 0) {
        if (digit > base)
            break;
        num = num * base + digit;
        ch = *p++;
    }
    *src = p;
    *nump = num;
    return ch;
}

//以下两个函数共同构成了一个itoa()，被public itoa()函数调用
/**********************************************************************
函数名称：_i2a
函数功能：_i2a
函数形参：要转换的整数，一个指向字符转换缓冲区的指针，转换的基数
函数返回值：result
函数描述：
	范围为2到36(包括36)，基数范围错误默认为base10
**********************************************************************/
static char *_i2a(unsigned i, char *a, unsigned base)
{
    if (i / base > 0)
        a = _i2a(i / base, a, base);
    *a = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"[i % base];
    return a + 1;
}

/**********************************************************************
函数名称：itoa
函数功能：itoa
函数形参：要转换的整数，一个指向字符转换缓冲区的指针，转换的基数
函数返回值：result
函数描述：
	范围为2到36(包括36)，基数范围错误默认为base10
**********************************************************************/
char *itoa(int i, char *a, int base)
{
    if ((base < 2) || (base > 36))
        base = 10;
    if (i < 0) {
        *a = '-';
        *_i2a(-(unsigned) i, a + 1, base) = 0;
    } else
        *_i2a(i, a, base) = 0;
    return a;
}

