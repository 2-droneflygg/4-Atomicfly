#############################################
# 配置 ARM 交叉编译工具链
#############################################
# ---------设置 ARM-NONE-EABI-GCC 版本号
GCC_REQUIRED_VERSION ?= 10.3.1
# ---------获取本地 ARM-NONE-EABI-GCC 版本号
GCC_VERSION = $(shell arm-none-eabi-gcc -dumpversion)
# ---------判断ARM-NONE-EABI-GCC是否安装以及版本是否正确
ifeq ($(GCC_VERSION),)
$(error ERROR *** 未检测到arm-none-eabi-gcc $(GCC_VERSION)交叉编译工具链)
else ifneq ($(GCC_VERSION), $(GCC_REQUIRED_VERSION))
$(error ERROR *** arm-none-eabi-gcc版本：$(GCC_VERSION), 请安装$(GCC_REQUIRED_VERSION)版本)
endif
# ---------设置 ARM_SDK_TOOL 前缀
ARM_SDK_PREFIX ?= arm-none-eabi-
