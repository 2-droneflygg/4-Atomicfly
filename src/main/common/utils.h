/*********************************************************************************
 提供一系列工具类宏定义。
*********************************************************************************/
#pragma once

#include <stddef.h>
#include <stdint.h>

// --------------------------------------------------------------------编译校验类
// 编译校验，校验条件失败，则输出错误信息 - 格式: static_assert(条件，错误信息);
#define STATIC_ASSERT(condition, name) _Static_assert((condition), #name)

// --------------------------------------------------------------------数组类
 // 计算数组元素个数
 #define ARRAYLEN(x) (sizeof(x) / sizeof((x)[0]))
 // 获取数组最后一个元素
 #define ARRAYEND(x) (&(x)[ARRAYLEN(x)])

// --------------------------------------------------------------------参数连接类
// 辅助连接宏参数 - 概念连接符：##操作符
#define CONCAT_HELPER(x,y)    x ## y
#define CONCAT(x,y)           CONCAT_HELPER(x, y)
#define CONCAT2(_1,_2)        CONCAT(_1, _2)
#define CONCAT3(_1,_2,_3)     CONCAT(CONCAT(_1, _2), _3)
#define CONCAT4(_1,_2,_3,_4)  CONCAT(CONCAT3(_1, _2, _3), _4)
#define STR_HELPER(x)         #x
#define STR(x)                STR_HELPER(x)
// __VA_ARGS__ ：可变参数宏，宏定义中参数列表的最后一个参数为省略号（也就是三个点）
#define PP_CALL(macro, ...) macro(__VA_ARGS__)
// 当位为1时展开为t，当位为0时展开为f，不支持其他位值
#define PP_IIF_I(bit, t, f) PP_IIF_ ## bit(t, f)
#define PP_IIF(bit, t, f) PP_IIF_I(bit, t, f)
#define PP_IIF_0(t, f) f
#define PP_IIF_1(t, f) t

// --------------------------------------------------------------------变量转换类
// 未使用的变量和参数
#define UNUSED(x) (void)(x)  
// 显式忽略x的结果(通常是I/O寄存器访问)
#define DISCARD(x) (void)(x) 

// --------------------------------------------------------------------位操作类
// 辅助位移
#define BIT(x) (1 << (x))
// BX_(X):以4bit为一个单位进行BIT_COUNT，在此基础上16进制.
//		以一个4bit的数据x = abcd为例，其中a~d为0或1
//		x = 8a + 4b + 2c + d
//		x>>1 = 4a + 2b + c
//		x>>2 = 2a + b
//		x>>3 = a
//		BX_(x) = x - (x>>1) - (x>>2) - (x>>3) = a + b + c + d 
#define BX_(x)       ((x) - (((x)>>1)&0x77777777) - (((x)>>2)&0x33333333) - (((x)>>3)&0x11111111))
// 	BITCOUNT(x) = sum( 0x000_000f & (BX_(x) >> (i*4)) )，其中sum()表示求和运算，i的取值为0~7。
//		如果x是一个16位的数，比如abcd，其中a~d代表一个4bit的数。
//		BX_(abcd) = efgh， BITCOUNT 应该等于 e+f+g+h             
//		则BITCOUNT(abcd) = ((efgh + 0efg ) & 0x0f0f) %0xff
//						 = 0j0k % 0xff ，其中j= e+f， k = g+h
//						 = j+k = e+f+g+h
//		如果x是一个32位的数，比如ijkl_mnop，则BX_(ijkl_mnop)=abcd_efgh，需要证明 BITCOUNT = sum(a,h)
//		BITCOUNT(x) = ((abcd_efgh + 0abc_defg) & 0x0f0f_0f0f) % 0xff
//					= 0q0r0s0t % 0xff， 其中 q=a+b，r=c+d, s=e+f, t=g+h
//					= q+r+s+t = sum(a,h)
//      功能：将所有位相加。比如1110 0011  =1+1+0+0+0+1+1+1=5
#define BITCOUNT(x)  (((BX_(x)+(BX_(x)>>4)) & 0x0F0F0F0F) % 255)

// --------------------------------------------------------------------结构体操作类
// 通过一个结构变量中一个成员的地址找到这个结构体变量的首地址（__extension__：消除非标准警告）
//	C链表操作中。场景是这样的：
//  获取到一个元素A(或结构体)地址，而它又是另一个结构体B的成员。使用这个函数可以通过A地址求取B结构体地址。
// 		ptr：指向功能成员的指针，存放功能成员的地址
// 		type：宿主结构体的类型
// 		member：功能成员在结构体中的表示（变量名）

// 	1.定义一个变量的格式是： 修饰符+变量类型+变量名 = 右值；
// 		修饰符            变量类型                        变量名        右值
// 		const     typeof( ((type*)0)->member )    *__mptr =  (ptr) ;

// 	“typeof( ((type*)0)->member )”代表的是一种数据类型：
// 		((type*)0)：它把0转换为一个type类型（也就是宿主结构体类型）
// 		((type*)0)->member：这个0指针指向结构体中的member成员
// 		typeof是gcc的c语言扩展保留字，用于获取变量的类型
// 		typeof( ((type*)0)->member )*：得出member的数据类型

//	2.(char *)__mptr - offset(type, member)：
//		用第2行获得的结构体成员地址减去其在结构体中的偏移值便可得到宿主结构体的地址
// 		将__mptr转换为char类型指针：C语言中，一个指向特定数据类型的指针减1，实际上减去的是它所指向数据类型的字节大小sizeof(data)
//		(type*)( (char *)__mptr - offset(type, member) ) 最后把地址转换成宿主结构体的类型type。
#define container_of(ptr, type, member)  ( __extension__ ({     \
        const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
        (type *)( (char *)__mptr - offsetof(type,member) );}))

// --------------------------------------------------------------------辅助计算类
// 辅助作差（uint32_t）
static inline int32_t cmp32(uint32_t a, uint32_t b) { return (int32_t)(a-b); }
// LOG2对数运算 (v ? floor(log2(v)) : 0)  - 向下舍入（沿绝对值减小的方向）
#define LOG2_8BIT(v)  (8 - 90/(((v)/4+14)|1) - 2/((v)/2+1))
#define LOG2_16BIT(v) (8*((v)>255) + LOG2_8BIT((v) >>8*((v)>255)))
#define LOG2_32BIT(v) (16*((v)>65535L) + LOG2_16BIT((v)*1L >>16*((v)>65535L)))
#define LOG2_64BIT(v) (32*((v)/2L>>31 > 0) + LOG2_32BIT((v)*1L >>16*((v)/2L>>31 > 0) >>16*((v)/2L>>31 > 0)))
// 向上兼容，自动判断位数
#define LOG2(v)       LOG2_64BIT(v)

// --------------------------------------------------------------------辅助语法类
// 可以在需要贯穿的case块中使用fallthrough
#if __GNUC__ > 6
#define FALLTHROUGH __attribute__ ((fallthrough))
#else
#define FALLTHROUGH do {} while(0)
#endif

