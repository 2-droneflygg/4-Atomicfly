/**********************************************************************
 PG参数组寄存器ID：
**********************************************************************/
#pragma once
	 	
// 板对齐类
#define PG_BOARD_ALIGNMENT 			    1   	   

// 系统类
#define PG_SYSTEM_CONFIG 			    2 	
#define PG_FEATURE_CONFIG 			    3 	
#define PG_ARMING_CONFIG 			    4 		
#define PG_STATUS_LED_CONFIG 		    5
#define PG_BEEPER_CONFIG 			    6
#define PG_BEEPER_DEV_CONFIG 		    7

// 硬件驱动类
#define PG_ADC_CONFIG 				    8	
#define PG_SERIAL_PIN_CONFIG 		    9
#define PG_SERIAL_UART_CONFIG 		    10
#define PG_SERIAL_CONFIG 				11 	
#define PG_TIMER_IO_CONFIG 				12      
#define PG_SPI_PIN_CONFIG 				13
#define PG_I2C_CONFIG 					14

// RX类
#define PG_MODE_ACTIVATION_PROFILE 		15		
#define PG_RX_FAILSAFE_CHANNEL_CONFIG 	16
#define PG_RX_CHANNEL_RANGE_CONFIG 		17
#define PG_RX_CONFIG 					18			 		
#define PG_RC_CONTROLS_CONFIG 			19 	
#define PG_FAILSAFE_CONFIG 				20  	

// 传感器&&控制类
#define PG_CURRENT_SENSOR_ADC_CONFIG 	21
#define PG_VOLTAGE_SENSOR_ADC_CONFIG 	22
#define PG_BATTERY_CONFIG 				23
#define PG_GYRO_DEVICE_CONFIG 			24
#define PG_GYRO_CONFIG 					25
#define PG_ACCELEROMETER_CONFIG 		26   	
#define PG_BAROMETER_CONFIG 			27		
#define PG_COMPASS_CONFIG 				28
#define PG_GPS_CONFIG 					29
#define PG_GPS_RESCUE 					30 
#define PG_POSITION 					31
#define PG_IMU_CONFIG 					32 	
#define PG_PID_CONFIG 					33
#define PG_PID_PROFILE 					34	
#define PG_CONTROL_RATE_PROFILES 		35 	
#define PG_MOTOR_CONFIG 				36			
#define PG_MOTOR_MIXER 					37	   		 	
#define PG_MIXER_CONFIG 				38	

// 显示类
#define PG_MAX7456_CONFIG 				39
#define PG_VCD_CONFIG 					40
#define PG_DISPLAY_PORT_MAX7456_CONFIG 	41
#define PG_OSD_CONFIG 					42
#define PG_OSD_ELEMENT_CONFIG 			43

// 图传类
#define PG_VTX_SETTINGS_CONFIG 			44
#define PG_VTX_CONFIG 					45

// 4095是目前可以用于PGN的最高数字，因为16位值的前4位是为PG存储在EEPROM中的版本保留的
#define PG_RESERVED_FOR_TESTING_1 		4095
#define PG_RESERVED_FOR_TESTING_2 		4094
#define PG_RESERVED_FOR_TESTING_3 		4093

