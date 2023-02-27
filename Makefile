###############################################################################
# 命令显示详细程度（make执行命令时是否将命令显示在标准输出）：
###############################################################################
# --------------------------------------清理文件命令显示控制
V0    := @
# --------------------------------------编译文件命令显示控制
V1    := @

###############################################################################
# 硬件相关配置：
###############################################################################
# --------------------------------------飞控名称
BASE_TARGET     := DFLIGHTF4
# --------------------------------------工程名称
FORKNAME        := Dflight
# --------------------------------------MCU型号
TARGET          := STM32F405RGT6
# --------------------------------------MCU_HSE外部时钟源（8M）
HSE_VALUE       := 8000000
# --------------------------------------MCU_FLASH容量配置（STM32F405）
MCU_FLASH_SIZE  := 1024
# --------------------------------------ARM编译标志
ARCH_FLAGS      := -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
# --------------------------------------设备编译标志[通过宏定义配置设备]（-D：编译器宏定义[替换、条件编译]）
DEVICE_FLAGS    := -DSTM32F40_41xxx -DSTM32F405xx
DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE)
DEVICE_FLAGS    += -DTARGET_FLASH_SIZE=$(MCU_FLASH_SIZE)
DEVICE_FLAGS    += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM4 -D__FPU_PRESENT=1

###############################################################################
# 目录配置(所有工程文件添加到全局访问路径 - VPATH)：
###############################################################################
# --------------------------------------固件版本信息
# 使用shell正则表达式（shell grep）并使用awk命令取第三个字段
# 在Makefile中使用shell命令进行变量引用时要对$符号转义 - 写两个"$$"符号来转义成"$"符号
FC_VER_MAJOR    := $(shell grep " FC_VERSION_MAJOR" src/main/build/version.h | awk '{print $$3}' )
FC_VER_MINOR    := $(shell grep " FC_VERSION_MINOR" src/main/build/version.h | awk '{print $$3}' )
FC_VER_PATCH    := $(shell grep " FC_VERSION_PATCH" src/main/build/version.h | awk '{print $$3}' )
FC_VER          := $(FC_VER_MAJOR).$(FC_VER_MINOR).$(FC_VER_PATCH)

# --------------------------------------工程目录配置
# ---------根目录
ROOT            := .
# ---------STM32_FLASH链接文件目录
LINKER_DIR      := $(ROOT)/src/link
# ---------STM32启动文件目录
STARTUP_DIR     := $(ROOT)/src/main/startup
# ---------CMSIS库文件目录
CMSIS_DIR       := $(ROOT)/lib/CMSIS
# ---------STD库文件目录	
STDPERIPH_DIR   = $(ROOT)/lib/STM32F4xx_StdPeriph_Driver
# ---------USB库目录
USB_FS_DIR = $(ROOT)/lib/STM32_USB_FS_Driver
# ---------源码目录
SRC_DIR         := $(ROOT)/src/main
# ---------硬件平台配置目录
TARGET_DIR      := $(ROOT)/src/main/target/$(BASE_TARGET)
# ---------目标输出目录
OBJECT_DIR      := $(ROOT)/obj
# ---------添加启动文件目录、源码目录到全局访问路径
VPATH           += $(STARTUP_DIR) $(SRC_DIR) $(TARGET_DIR)

# --------------------------------------添加MAKE子文件目录到全局访问路径
VPATH 		    := $(VPATH):$(ROOT)/make

# --------------------------------------源文件目录（管理所有工程文件）
include $(ROOT)/make/source.mk

# --------------------------------------特殊变量：配置构建文件类型（hex/bin）
.DEFAULT_GOAL   := hex

###############################################################################
# 交叉编译器工具配置：
###############################################################################
# --------------------------------------配置 ARM 交叉编译工具
include $(ROOT)/make/tools.mk
# --------------------------------------交叉编译工具
CROSS_CC    			:= $(ARM_SDK_PREFIX)gcc
OBJCOPY     			:= $(ARM_SDK_PREFIX)objcopy
SIZE       			    := $(ARM_SDK_PREFIX)size
# --------------------------------------编译优化选项
OPTIMISATION_BASE       := -flto -fuse-linker-plugin -ffast-math
OPTIMISE_DEFAULT        := -O2
OPTIMISE_SPEED          := -Ofast
OPTIMISE_SIZE           := -Os
CC_DEFAULT_OPTIMISATION := $(OPTIMISATION_BASE) $(OPTIMISE_DEFAULT)
CC_SPEED_OPTIMISATION   := $(OPTIMISATION_BASE) $(OPTIMISE_SPEED)
CC_SIZE_OPTIMISATION    := $(OPTIMISATION_BASE) $(OPTIMISE_SIZE)
CC_NO_OPTIMISATION      := 
# --------------------------------------编译标志
# ---------编译标志（c文件）-（-D：编译器宏定义[替换、条件编译]）
CFLAGS      = $(ARCH_FLAGS) \
              $(DEVICE_FLAGS) \
              $(addprefix -I,$(INCLUDE_DIRS)) \
              -DUSE_STDPERIPH_DRIVER \
              -D'__TARGET__="$(TARGET)"' 
# ---------编译标志（s文件）
ASFLAGS     = $(ARCH_FLAGS) 
# ---------链接标志（ld文件）
LD_FLAGS    = $(ARCH_FLAGS) \
			  -T$(LD_SCRIPT) \
              -Wl,-gc-sections,-Map,$(TARGET_MAP) \
              -Wl,-L$(LINKER_DIR) \
			  -lm \
              -lc \
              -lnosys \
              --specs=nano.specs \
              -static \
              -Wl,--cref \
              -Wl,--no-wchar-size-warning \
              -Wl,--print-memory-usage \
              -nostartfiles 

###############################################################################
# 构建目标：
###############################################################################
# --------------------------------------构建目标名称
# 工程名称_版本_MCU型号
TARGET_BASENAME := $(OBJECT_DIR)/$(FORKNAME)_$(FC_VER)_$(TARGET)
# --------------------------------------构建文件目录
TARGET_BIN      := $(TARGET_BASENAME).bin
TARGET_HEX      := $(TARGET_BASENAME).hex
TARGET_ELF      := $(OBJECT_DIR)/$(TARGET)/$(FORKNAME)_$(TARGET).elf
TARGET_MAP      := $(OBJECT_DIR)/$(TARGET)/$(FORKNAME)_$(TARGET).map
# --------------------------------------.o文件列表 - 根据.c文件生成
# $(addprefix <prefix>, <name1 name2 ...>)：把<prefix>加到name序列中的每一个元素前面（加前缀）
# addsuffix用法与addprefix相同（加后缀）
TARGET_OBJS     := $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(SRC)))) 
# --------------------------------------清理文件目录
CLEAN_ARTIFACTS := $(TARGET_BIN) $(TARGET_HEX) $(TARGET_ELF) $(TARGET_MAP) $(TARGET_OBJS) 

# --------------------------------------汇编c文件->o文件 - VPATH 
# ---------1.定义compile_file函数 - 调用使用call函数，参数对应函数原型中的$(1),$(2)
# 函数功能：
# 	(1)提示："%% (编译优化类型) 带路径文件名[依赖]" 
#	(2)编译：gcc -c 依赖 -o 目标 C编译标志 编译优化选项 
define compile_file
	echo "%% ($(1)) $<" && $(CROSS_CC) -c $< -o $@ $(CFLAGS) $(2)
endef
# ---------2. 编译（隐晦规则[自动推导]）
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	@# 根据规则目标创建对应文件夹 - dir：取目录
	$(V1) mkdir -p $(dir $@)
	@# $(if <condition>,<then-part>,<else-part>) 函数 - 嵌套使用
	@# <condition>：表达式，返回为非空字符串为真，计算<then-part>，否则计算<else-part>
	@# findstring函数 - 在第2个参数中寻找第一个参数并返回
	$(V1) \
	$(if $(findstring $<,$(NOT_OPTIMISED_SRC)), \
		$(call compile_file,not optimised,$(CC_NO_OPTIMISATION)), \
		$(if $(findstring $(subst ./src/main/,,$<),$(SPEED_OPTIMISED_SRC)), \
			$(call compile_file,speed optimised,$(CC_SPEED_OPTIMISATION)), \
			$(if $(findstring $(subst ./src/main/,,$<),$(SIZE_OPTIMISED_SRC)), \
				$(call compile_file,size optimised,$(CC_SIZE_OPTIMISATION)), \
				$(call compile_file,optimised,$(CC_DEFAULT_OPTIMISATION)) \
			) \
		) \
	)
# --------------------------------------汇编s启动文件->o文件 - VPATH 
$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	@# 根据规则目标创建对应文件夹 - dir：取目录
	$(V1) mkdir -p $(dir $@)
	@# notdir：去除目录信息
	@echo "%% $(notdir $<)" 
	@# 编译
	$(V1) $(CROSS_CC) -c $< -o $@ $(ASFLAGS) 
# --------------------------------------FLASH链接文件->ELF可执行文件
$(TARGET_ELF): $(TARGET_OBJS) $(LD_SCRIPT)
	@echo "Linking  $(TARGET)" 
	$(V1) $(CROSS_CC) -o $@ $(filter-out %.ld,$^) $(LD_FLAGS)
	$(V1) $(SIZE) $(TARGET_ELF)
# --------------------------------------ELF可执行文件->烧录文件(单片机可执行文件)
$(TARGET_BIN): $(TARGET_ELF)
	@echo "Creating BIN $(TARGET_BIN)" 
	$(V1) $(OBJCOPY) -O binary $< $@
$(TARGET_HEX): $(TARGET_ELF)
	@echo "Creating HEX $(TARGET_HEX)" 
	$(V1) $(OBJCOPY) -O ihex --set-start 0x8000000 $< $@
# --------------------------------------构建文件 - 根据.DEFAULT_GOAL 指定标签选择 - $(MAKE) = make
# ---------构建bin文件
bin:		    
	$(V1) $(MAKE) -j $(TARGET_BIN)
# ---------构建hex文件
hex:            
	$(V1) $(MAKE) -j $(TARGET_HEX)
# --------------------------------------清理生成文件
clean:
	@echo "Cleaning $(TARGET) ..."
	$(V0) rm -f $(CLEAN_ARTIFACTS)
	$(V0) rm -rf $(OBJECT_DIR)/$(TARGET)
	@echo "Cleaning $(TARGET) succeeded."
# --------------------------------------重新构建
rebuild:
	$(V1) $(MAKE) clean
	$(V1) $(MAKE) -j4
# --------------------------------------查看固件版本信息
version:
	@echo $(FC_VER)
