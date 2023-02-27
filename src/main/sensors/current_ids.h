#pragma once

// 固定id，电流可以在许多不同的地方测量，这些标识符是支持或将考虑支持的
typedef enum {
    CURRENT_METER_ID_NONE = 0,
	// 10-19 for battery meters
    CURRENT_METER_ID_BATTERY_1 = 10,       
    CURRENT_METER_ID_BATTERY_2,
    //..
    CURRENT_METER_ID_BATTERY_10 = 19,
	// 20-29 for 5V meters
    CURRENT_METER_ID_5V_1 = 20,            
    CURRENT_METER_ID_5V_2,
    //..
    CURRENT_METER_ID_5V_10 = 29,
	// 30-39 for 9V meters
    CURRENT_METER_ID_9V_1 = 30,            
    CURRENT_METER_ID_9V_2,
    //..
    CURRENT_METER_ID_9V_10 = 39,
	// 40-49 for 12V meters
    CURRENT_METER_ID_12V_1 = 40,           
    CURRENT_METER_ID_12V_2,
    //..
    CURRENT_METER_ID_12V_10 = 49,
	// 50-59 for ESC combined (it's doubtful an FC would ever expose 51-59 however)
    CURRENT_METER_ID_ESC_COMBINED_1 = 50,  
    // ...
    CURRENT_METER_ID_ESC_COMBINED_10 = 59,
	// 60-79 for ESC motors (20 motors)
    CURRENT_METER_ID_ESC_MOTOR_1 = 60,     
    CURRENT_METER_ID_ESC_MOTOR_2,
    CURRENT_METER_ID_ESC_MOTOR_3,
    CURRENT_METER_ID_ESC_MOTOR_4,
    CURRENT_METER_ID_ESC_MOTOR_5,
    CURRENT_METER_ID_ESC_MOTOR_6,
    CURRENT_METER_ID_ESC_MOTOR_7,
    CURRENT_METER_ID_ESC_MOTOR_8,
    CURRENT_METER_ID_ESC_MOTOR_9,
    CURRENT_METER_ID_ESC_MOTOR_10,
    CURRENT_METER_ID_ESC_MOTOR_11,
    CURRENT_METER_ID_ESC_MOTOR_12,
    //...
    CURRENT_METER_ID_ESC_MOTOR_20 = 79,
	// 80-89 for virtual meters
    CURRENT_METER_ID_VIRTUAL_1 = 80,       
    CURRENT_METER_ID_VIRTUAL_2,
	// 90-99 for MSP meters
    CURRENT_METER_ID_MSP_1 = 90,           
    CURRENT_METER_ID_MSP_2,
} currentMeterId_e;
	
