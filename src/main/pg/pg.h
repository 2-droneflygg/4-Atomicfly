#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "build/build_config.h"

/* --------------------------内部注册表枚举-------------------------- */	
typedef enum {
    PGR_PGN_MASK =          0x0fff,	  		// 寄存器参数组掩码
    PGR_SIZE_MASK =         0x0fff,	  		// 寄存器大小掩码
    PGR_SIZE_SYSTEM_FLAG =  0x0000 	  		// 寄存器大小系统标志
} pgRegistryInternal_e;

/* ---------------------------注册表结构体--------------------------- */	
// PG参数组复位回调功能函数
typedef void (pgResetFunc)(void * /* base */);
// 参数组号类型
typedef uint16_t pgn_t;
typedef struct pgRegistry_s {
    pgn_t pgn;             			  		// 参数组号，前4位保留为版本号 - pg_ids中定义
    uint8_t length;        		      		// 组中元素的个数
    uint16_t size;         			  		// 在RAM中组的大小，前4位预留给标志
    uint8_t *address;      			  		// 参数组在RAM中的地址
    uint8_t *copy;         			  		// 存储器中副本的地址
    uint8_t **ptr;         			  		// 在将记录加载到ram后进行更新的指针
    union {
        void *ptr;         			  		// 初始化模板指针
        pgResetFunc *fn;   			  		// PG参数组复位函数
    } reset;
} pgRegistry_t;

// -----------------------------------------------------------------------PG参数组包类型
#define PG_PACKED __attribute__((packed))

// -----------------------------------------------------------------------参数转换
// 将参数转换为FLOAT
#define CONVERT_PARAMETER_TO_FLOAT(param)   (0.001f * param)		 
// 将参数转换为百分比
#define CONVERT_PARAMETER_TO_PERCENT(param) (0.01f * param)	

// -----------------------------------------------------------------------PG寄存器
extern const pgRegistry_t __pg_registry_start[];					  						     // PG寄存器起始地址
extern const pgRegistry_t __pg_registry_end[];						  					         // PG寄存器结束地址
#define PG_REGISTRY_SIZE (__pg_registry_end - __pg_registry_start)                               // PG寄存器大小
#define PG_REGISTER_ATTRIBUTES __attribute__ ((section(".pg_registry"), used, aligned(4)))      // PG寄存器段
extern const uint8_t __pg_resetdata_start[];						  						     // PG寄存器数据起点
extern const uint8_t __pg_resetdata_end[];							  						     // PG寄存器数据末尾
#define PG_RESETDATA_ATTRIBUTES __attribute__ ((section(".pg_resetdata"), used, aligned(2)))    // PG寄存器数据段

// -----------------------------------------------------------------------内联函数
// 在函数声明或定义中，函数返回类型前加上关键字inline。
// 可以解决一些频繁调用的函数大量消耗栈空间（栈内存）的问题。
// 1.一般函数的代码段只有一份，放在内存中的某个位置上，当程序调用它时，指令就跳转过来；
// 当下一次程序调用它是，指令又跳转过来。
// 2.而内联函数是程序中调用几次内联函数，内联函数的代码就会复制几份放在对应的位置上。
// 没有指令跳转，指令按顺序执行。
// 3.内联函数一般在头文件中定义，而一般函数在头文件中声明，源文件中定义。
// 注意：内联函数一般只会用在函数内容非常简单的时候，内联函数的代码会在任何调用它的地方展开，
//       如果函数太复杂，代码膨胀带来的恶果很可能会大于效率的提高带来的益处。
// 相关功能函数：
static inline uint16_t pgN(const pgRegistry_t* reg) {return reg->pgn & PGR_PGN_MASK;}           // 获取寄存器参数组号
static inline uint16_t pgSize(const pgRegistry_t* reg) {return reg->size & PGR_SIZE_MASK;}		  // 获取寄存器参数组大小
static inline uint8_t pgVersion(const pgRegistry_t* reg) {return (uint8_t)(reg->pgn >> 12);}	 // 获取寄存器参数组版本

// -----------------------------------------------------------------------功能宏
// ----------------------------辅助程序来遍历PG寄存器，比访问者风格的回调成本更低
#define PG_FOREACH(_name)                                               \
    for (const pgRegistry_t *(_name) = __pg_registry_start; (_name) < __pg_registry_end; _name++)

// ----------------------------PG重置为默认值
#define PG_RESET(_name)                                                 \
    do {                                                                \
        extern const pgRegistry_t _name ##_Registry;                    \
        pgReset(&_name ## _Registry);                                   \
    } while (0)                                                         \
    /**/

// ----------------------------PG声明
#define PG_DECLARE(_type, _name)                                        \
    extern _type _name ## _System;                                      \
    extern _type _name ## _Copy;                                        \
    static inline const _type* _name(void) { return &_name ## _System; }\
    static inline _type* _name ## Mutable(void) { return &_name ## _System; }\
    struct _dummy                                                       \
    /**/

// ----------------------------PG数组声明
#define PG_DECLARE_ARRAY(_type, _length, _name)                         \
    extern _type _name ## _SystemArray[_length];                        \
    extern _type _name ## _CopyArray[_length];                          \
    static inline const _type* _name(int _index) { return &_name ## _SystemArray[_index]; }      \
    static inline _type* _name ## Mutable(int _index) { return &_name ## _SystemArray[_index]; } \
    static inline _type (* _name ## _array(void))[_length] { return &_name ## _SystemArray; }    \
    struct _dummy                                                       \
    /**/

// ----------------------------PG注册[结构体变量]
#define PG_REGISTER_I(_type, _name, _pgn, _version, _reset)             \
    _type _name ## _System;                                             \
    _type _name ## _Copy;                                               \
    /* 外部声明PG寄存器 */                								        \
    extern const pgRegistry_t _name ## _Registry;                       \
    const pgRegistry_t _name ##_Registry PG_REGISTER_ATTRIBUTES = {     \
    	/* 寄存器ID */ 													\
        .pgn = _pgn | (_version << 12),                                 \
        /* 元素个数*/ 														\
        .length = 1,                                                    \
        /* 参数组在在RAM中的大小 */ 												\
        .size = sizeof(_type) | PGR_SIZE_SYSTEM_FLAG,                   \
        /* 参数组地址[结构体变量地址] */ 											\
        .address = (uint8_t*)&_name ## _System,                         \
        /* 参数组副本地址[结构体变量地址] */ 											\
        .copy = (uint8_t*)&_name ## _Copy,                              \
        /* 初始化模板指针 */ 													\
        .ptr = 0,                                                       \
        /* 复位函数 */														\
        _reset,                                                         \
    }                                                                   \
    /**/

// ----------------------------PG注册
#define PG_REGISTER(_type, _name, _pgn, _version)                       \
    PG_REGISTER_I(_type, _name, _pgn, _version, .reset = {.ptr = 0})    \
    /**/

// ----------------------------PG注册与重置
#define PG_REGISTER_WITH_RESET_FN(_type, _name, _pgn, _version)         \
    extern void pgResetFn_ ## _name(_type *);                           \
    PG_REGISTER_I(_type, _name, _pgn, _version, .reset = {.fn = (pgResetFunc*)&pgResetFn_ ## _name }) \
    /**/

// ----------------------------PG注册与重置模板
#define PG_REGISTER_WITH_RESET_TEMPLATE(_type, _name, _pgn, _version)   \
    extern const _type pgResetTemplate_ ## _name;                       \
    PG_REGISTER_I(_type, _name, _pgn, _version, .reset = {.ptr = (void*)&pgResetTemplate_ ## _name}) \
    /**/

// ----------------------------PG注册系统配置数组
#define PG_REGISTER_ARRAY_I(_type, _length, _name, _pgn, _version, _reset)  \
    _type _name ## _SystemArray[_length];                               \
    _type _name ## _CopyArray[_length];                                 \
    extern const pgRegistry_t _name ##_Registry;                        \
    const pgRegistry_t _name ## _Registry PG_REGISTER_ATTRIBUTES = {    \
        .pgn = _pgn | (_version << 12),                                 \
        .length = _length,                                              \
        .size = (sizeof(_type) * _length) | PGR_SIZE_SYSTEM_FLAG,       \
        .address = (uint8_t*)&_name ## _SystemArray,                    \
        .copy = (uint8_t*)&_name ## _CopyArray,                         \
        .ptr = 0,                                                       \
        _reset,                                                         \
    }                                                                   \
    /**/

// ----------------------------PG注册数组
#define PG_REGISTER_ARRAY(_type, _length, _name, _pgn, _version)        \
    PG_REGISTER_ARRAY_I(_type, _length, _name, _pgn, _version, .reset = {.ptr = 0}) \
    /**/

// ----------------------------PG重置注册数组 - 调用pgResetFn__name重置函数
#define PG_REGISTER_ARRAY_WITH_RESET_FN(_type, _length, _name, _pgn, _version) \
    extern void pgResetFn_ ## _name(_type *);    \
    PG_REGISTER_ARRAY_I(_type, _length, _name, _pgn, _version, .reset = {.fn = (pgResetFunc*)&pgResetFn_ ## _name}) \
    /**/

// ----------------------------PG注册数组元素偏移
// C库宏offsetof(type, member-designator) 结果在一个常数整数 size_t 类型是一个结构成员的结构从一开始的字节偏移
#define PG_ARRAY_ELEMENT_OFFSET(type, index, member) (index * sizeof(type) + offsetof(type, member))

// ----------------------------PG重置模板
// 为config触发重置缺省值。
// Config必须注册到PG_REGISTER_<xxx>_WITH_RESET_TEMPLATE宏
// __VA_ARGS__：宏定义中参数列表的最后一个参数为省略号，在预处理中，会被实际的参数集（实参列表）所替换
#define PG_RESET_TEMPLATE(_type, _name, ...)                            \
    const _type pgResetTemplate_ ## _name PG_RESETDATA_ATTRIBUTES = {   \
        __VA_ARGS__                                                     \
    }                                                                   \
    /**/

bool pgLoad(const pgRegistry_t* reg, const void *from, int size, int version);
void pgResetAll(void);
void pgResetInstance(const pgRegistry_t *reg, uint8_t *base);
void pgReset(const pgRegistry_t* reg);

