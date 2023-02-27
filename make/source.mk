########################################################### 
# STM32_FLASH链接文件:
###########################################################	
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f405.ld

########################################################### 
# STM32启动文件:
###########################################################	
STARTUP_SRC     = startup_stm32f40xx.s

###########################################################
# STM32_STD固件库文件：
###########################################################	
# --------------------------------------微控制器软件接口标准文件（CMSIS）
# ---------CMSIS库c文件
CMSIS_SRC       := $(notdir $(wildcard $(CMSIS_DIR)/*.c))
# ---------CMSIS库h文件           
CMSIS_INC       := $(CMSIS_DIR) 
# ---------添加CMSIS目录到全局访问路径
VPATH           += $(ROOT)/lib/CMSIS 
# --------------------------------------STD库文件   
# ---------STD库c文件 - 通配.c文件并去除目录信息	
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))
# ---------STD库h文件
STDPERIPH_INC   := $(STDPERIPH_DIR)/inc
# ---------非使用库文件
EXCLUDES        = stm32f4xx_crc.c \
                  stm32f4xx_can.c \
                  stm32f4xx_fmc.c \
                  stm32f4xx_sai.c \
                  stm32f4xx_cec.c \
                  stm32f4xx_dsi.c \
                  stm32f4xx_flash_ramfunc.c \
                  stm32f4xx_fmpi2c.c \
                  stm32f4xx_lptim.c \
                  stm32f4xx_qspi.c \
                  stm32f4xx_spdifrx.c \
                  stm32f4xx_cryp.c \
                  stm32f4xx_cryp_aes.c \
                  stm32f4xx_hash_md5.c \
                  stm32f4xx_cryp_des.c \
                  stm32f4xx_hash.c \
                  stm32f4xx_dbgmcu.c \
                  stm32f4xx_cryp_tdes.c \
                  stm32f4xx_hash_sha1.c \
                  stm32f4xx_wwdg.c \
                  stm32f4xx_iwdg.c \
                  stm32f4xx_rng.c \
                  stm32f4xx_dcmi.c \
                  stm32f4xx_dma2d.c \
                  stm32f4xx_ltdc.c \
                  stm32f4xx_dac.c \
                  stm32f4xx_dfsdm.c \
                  stm32f4xx_fsmc.c \
                  stm32f4xx_sdio.c
# ---------设备STD库c文件 - 排除非使用库文件
DEVICE_STDPERIPH_SRC := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))
# ---------添加STD库源码目录到全局访问路径
VPATH        += $(STDPERIPH_DIR)/src
# --------------------------------------合并CMSIS库和STD库c文件
STD_SRC = $(CMSIS_SRC) $(DEVICE_STDPERIPH_SRC)

########################################################### 
# USB固件库文件:
###########################################################
# ---------USB库c文件
USB_FS_SRC  = $(notdir $(wildcard $(USB_FS_DIR)/src/*.c))
# ---------USB库h文件
USB_FS_INC := $(USB_FS_DIR)/inc
# ---------添加USB库目录到全局访问路径
VPATH       += $(USB_FS_DIR)/src

########################################################### 
# 源文件目录:
###########################################################
SRC += $(STARTUP_SRC)  $(STD_SRC) $(USB_FS_SRC) 
SRC +=  main.c \
        $(addprefix build/,$(notdir $(wildcard $(SRC_DIR)/build/*.c))) \
        $(addprefix cms/,$(notdir $(wildcard $(SRC_DIR)/cms/*.c))) \
        $(addprefix common/,$(notdir $(wildcard $(SRC_DIR)/common/*.c))) \
        $(addprefix config/,$(notdir $(wildcard $(SRC_DIR)/config/*.c))) \
        $(addprefix drivers/,$(notdir $(wildcard $(SRC_DIR)/drivers/*.c))) \
        $(addprefix drivers/accgyro/,$(notdir $(wildcard $(SRC_DIR)/drivers/accgyro/*.c))) \
        $(addprefix drivers/barometer/,$(notdir $(wildcard $(SRC_DIR)/drivers/barometer/*.c))) \
        $(addprefix drivers/compass/,$(notdir $(wildcard $(SRC_DIR)/drivers/compass/*.c))) \
        $(addprefix fc/,$(notdir $(wildcard $(SRC_DIR)/fc/*.c))) \
        $(addprefix flight/,$(notdir $(wildcard $(SRC_DIR)/flight/*.c))) \
        $(addprefix io/, $(notdir $(wildcard $(SRC_DIR)/io/*.c))) \
        $(addprefix osd/, $(notdir $(wildcard $(SRC_DIR)/osd/*.c))) \
        $(addprefix pg/, $(notdir $(wildcard $(SRC_DIR)/pg/*.c))) \
        $(addprefix rx/, $(notdir $(wildcard $(SRC_DIR)/rx/*.c))) \
        $(addprefix scheduler/, $(notdir $(wildcard $(SRC_DIR)/scheduler/*.c))) \
        $(addprefix sensors/, $(notdir $(wildcard $(SRC_DIR)/sensors/*.c))) \
        $(addprefix startup/, $(notdir $(wildcard $(SRC_DIR)/startup/*.c))) \
        $(addprefix vcp/, $(notdir $(wildcard $(SRC_DIR)/vcp/*.c))) \
        $(notdir $(wildcard $(TARGET_DIR)/*.c)) 

########################################################### 
# 头文件搜索路径:
###########################################################
INCLUDE_DIRS += $(CMSIS_INC) $(STDPERIPH_INC)
INCLUDE_DIRS += $(USB_FS_INC) 
INCLUDE_DIRS += $(STARTUP_DIR) $(SRC_DIR) $(ROOT)/src/main/target $(ROOT)/src/main/vcp $(TARGET_DIR) 

########################################################### 
# 编译优化文件:
###########################################################
# -----------------------------速度优化文件
SPEED_OPTIMISED_SRC := $(SPEED_OPTIMISED_SRC) \
            $(CMSIS_SRC) \
            $(DEVICE_STDPERIPH_SRC) \
            $(USB_FS_SRC) \
            common/encoding.c \
            common/filter.c \
            common/maths.c \
            common/typeconversion.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/adc.c \
            drivers/bus.c \
            drivers/bus_spi.c \
            drivers/exti.c \
            drivers/io.c \
            drivers/rcc.c \
            drivers/serial.c \
            drivers/serial_uart.c \
            drivers/system.c \
            drivers/timer.c \
            fc/core.c \
            fc/tasks.c \
            fc/rc.c \
            fc/rc_controls.c \
            fc/runtime_config.c \
            flight/imu.c \
            flight/mixer.c \
            flight/pid.c \
            rx/rx.c \
            rx/sbus.c \
            rx/sbus_channels.c \
            scheduler/scheduler.c \
            sensors/acceleration.c \
            sensors/boardalignment.c \
            sensors/gyro.c \
            drivers/max7456.c \
            drivers/pwm_output_dshot.c \
            drivers/pwm_output_dshot_shared.c \
# -----------------------------大小优化文件:
SIZE_OPTIMISED_SRC := $(SIZE_OPTIMISED_SRC) \
            drivers/accgyro/accgyro_fake.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/bus_i2c_config.c \
            drivers/bus_spi_config.c \
            drivers/bus_spi_pinconfig.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/serial_pinconfig.c \
            drivers/serial_uart_init.c \
            drivers/serial_uart_pinconfig.c \
            drivers/vtx_common.c \
            fc/init.c \
            config/config_eeprom.c \
            config/feature.c \
            config/config_streamer.c \
            io/serial.c \
            cms/cms.c \
            cms/cms_menu_failsafe.c \
            cms/cms_menu_firmware.c \
            cms/cms_menu_gps_rescue.c\
            cms/cms_menu_imu.c \
            cms/cms_menu_main.c \
            cms/cms_menu_misc.c \
            cms/cms_menu_osd.c \
            cms/cms_menu_power.c \
            cms/cms_menu_saveexit.c \
            cms/cms_menu_vtx_common.c \
            cms/cms_menu_vtx_tramp.c \
            io/vtx.c \
            io/vtx_tramp.c \
            io/vtx_control.c \
            io/spektrum_vtx_control.c \
            osd/osd.c \
            osd/osd_elements.c \
            sensors/gyro_init.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
			