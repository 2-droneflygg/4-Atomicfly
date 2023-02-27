/*********************************************************************************
	提供了一个简单而小(+200 loc)的printf功能适用于嵌入式系统。
	它们在调试中非常有用，所以不需要费心使用调试器。
	注意，这将引入一些冗长的数学程序从而使你的可执行文件明显更长。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>

#include "platform.h"

#include "build/build_config.h"

#include "printf.h"

#ifdef REQUIRE_PRINTF_LONG_SUPPORT
#include "typeconversion.h"
#endif

#ifdef REQUIRE_CC_ARM_PRINTF_SUPPORT
putcf stdout_putf;
void *stdout_putp;

/**********************************************************************
函数名称：putchREQUIRE_CC_ARM_PRINTF_SUPPORTw
函数功能：打印bf，从左到至少n个字符
函数形参：putp，putf，n，z，bf
函数返回值：written
函数描述：
	填充为0('0')如果z!=0，否则空格(' ')
**********************************************************************/
static int putchw(void *putp, putcf putf, int n, char z, char *bf)
{
    int written = 0;
    char fc = z ? '0' : ' ';
    char ch;
    char *p = bf;
    while (*p++ && n > 0)
        n--;
    while (n-- > 0) {
        putf(putp, fc); written++;
    }
    while ((ch = *bf++)) {
        putf(putp, ch); written++;
    }
    return written;
}

/**********************************************************************
函数名称：tfp_format
函数功能：tfp格式化
函数形参：putp，putf，fmt，va
函数返回值：返回写入的字节数
函数描述：None
**********************************************************************/
int tfp_format(void *putp, putcf putf, const char *fmt, va_list va)
{
    char bf[12];
    int written = 0;
    char ch;

    while ((ch = *(fmt++))) {
        if (ch != '%') {
            putf(putp, ch); written++;
        } else {
            char lz = 0;
#ifdef  REQUIRE_PRINTF_LONG_SUPPORT
            char lng = 0;
#endif
            int w = 0;
            ch = *(fmt++);
            if (ch == '0') {
                ch = *(fmt++);
                lz = 1;
            }
            if (ch >= '0' && ch <= '9') {
                ch = a2i(ch, &fmt, 10, &w);
            }
#ifdef  REQUIRE_PRINTF_LONG_SUPPORT
            if (ch == 'l') {
                ch = *(fmt++);
                lng = 1;
            }
#endif
            switch (ch) {
            case 0:
                goto abort;
            case 'u':{
#ifdef  REQUIRE_PRINTF_LONG_SUPPORT
                    if (lng)
                        uli2a(va_arg(va, unsigned long int), 10, 0, bf);
                    else
#endif
                        ui2a(va_arg(va, unsigned int), 10, 0, bf);
                    written += putchw(putp, putf, w, lz, bf);
                    break;
                }
            case 'd':{
#ifdef  REQUIRE_PRINTF_LONG_SUPPORT
                    if (lng)
                        li2a(va_arg(va, unsigned long int), bf);
                    else
#endif
                        i2a(va_arg(va, int), bf);
                    written += putchw(putp, putf, w, lz, bf);
                    break;
                }
            case 'x':
            case 'X':
#ifdef  REQUIRE_PRINTF_LONG_SUPPORT
                if (lng)
                    uli2a(va_arg(va, unsigned long int), 16, (ch == 'X'), bf);
                else
#endif
                    ui2a(va_arg(va, unsigned int), 16, (ch == 'X'), bf);
                written += putchw(putp, putf, w, lz, bf);
                break;
            case 'c':
                putf(putp, (char) (va_arg(va, int))); written++;
                break;
            case 's':
                written += putchw(putp, putf, w, 0, va_arg(va, char *));
                break;
            case '%':
                putf(putp, ch); written++;
                break;
            case 'n':
                *va_arg(va, int*) = written;
                break;
            default:
                break;
            }
        }
    }
abort:
    return written;
}

/**********************************************************************
函数名称：putcp
函数功能：putcp
函数形参：*p，c
函数返回值：None
函数描述：None
**********************************************************************/
static void putcp(void *p, char c)
{
    *(*((char **) p))++ = c;
}

/**********************************************************************
函数名称：tfp_sprintf
函数功能：tfp_sprintf
函数形参：buff地址，字符串
函数返回值：written
函数描述：None
**********************************************************************/
int tfp_sprintf(char *s, const char *fmt, ...)
{
	// VA_LIST 是在C语言中解决变参问题的一组宏。
	// 所在头文件：#include <stdarg.h>，用于获取不确定个数的参数。
    va_list va;
	// 获取可变参数列表的第一个参数的地址
    va_start(va, fmt);    
    int written = tfp_format(&s, putcp, fmt, va);
    putcp(&s, 0);
    va_end(va);
    return written;
}
#endif // REQUIRE_CC_ARM_PRINTF_SUPPORT

