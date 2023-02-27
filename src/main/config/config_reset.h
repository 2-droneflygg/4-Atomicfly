/*********************************************************************************
 提供配置复位相关宏定义。
*********************************************************************************/
#pragma once

#ifndef __UNIQL
# define __UNIQL_CONCAT2(x,y) x ## y
# define __UNIQL_CONCAT(x,y) __UNIQL_CONCAT2(x,y)
// x当前源代码行号
# define __UNIQL(x) __UNIQL_CONCAT(x,__LINE__)
#endif

// 结构体[数组元素]赋值
#define RESET_CONFIG(_type, _name, ...)                                 \
    static const _type __UNIQL(_reset_template_) = {                    \
        __VA_ARGS__                                                     \
    };                                                                  \
    memcpy((_name), &__UNIQL(_reset_template_), sizeof(*(_name)));      \
    /**/

// 结构体[变量]赋值
#define RESET_CONFIG_2(_type, _name, ...)                 \
    *(_name) = (_type) {                                  \
        __VA_ARGS__                                       \
    };                                                    \
    /**/

