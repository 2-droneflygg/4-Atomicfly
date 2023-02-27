#pragma once

// 固定id，电压可以在许多不同的地方测量
typedef enum {
    VOLTAGE_METER_ID_NONE = 0,
	// 10-19 for battery meters
    VOLTAGE_METER_ID_BATTERY_1 = 10,     
    VOLTAGE_METER_ID_BATTERY_2,
    //..
    VOLTAGE_METER_ID_BATTERY_10 = 19,
	// 20-29 for 5V meters
    VOLTAGE_METER_ID_5V_1 = 20,            
    VOLTAGE_METER_ID_5V_2,
    //..
    VOLTAGE_METER_ID_5V_10 = 29,
	// 30-39 for 9V meters
    VOLTAGE_METER_ID_9V_1 = 30,            
    VOLTAGE_METER_ID_9V_2,
    //..
    VOLTAGE_METER_ID_9V_10 = 39,
	// 40-49 for 12V meters
    VOLTAGE_METER_ID_12V_1 = 40,           
    VOLTAGE_METER_ID_12V_2,
    //..
    VOLTAGE_METER_ID_12V_10 = 49,
	// 50-59 for ESC combined (it's doubtful an FC would ever expose 51-59 however)
    VOLTAGE_METER_ID_ESC_COMBINED_1 = 50,  
    // ...
    VOLTAGE_METER_ID_ESC_COMBINED_10 = 59,
	// 60-79 for ESC motors (20 motors)
    VOLTAGE_METER_ID_ESC_MOTOR_1 = 60,     
    VOLTAGE_METER_ID_ESC_MOTOR_2,
    VOLTAGE_METER_ID_ESC_MOTOR_3,
    VOLTAGE_METER_ID_ESC_MOTOR_4,
    VOLTAGE_METER_ID_ESC_MOTOR_5,
    VOLTAGE_METER_ID_ESC_MOTOR_6,
    VOLTAGE_METER_ID_ESC_MOTOR_7,
    VOLTAGE_METER_ID_ESC_MOTOR_8,
    VOLTAGE_METER_ID_ESC_MOTOR_9,
    VOLTAGE_METER_ID_ESC_MOTOR_10,
    VOLTAGE_METER_ID_ESC_MOTOR_11,
    VOLTAGE_METER_ID_ESC_MOTOR_12,
    //...
    VOLTAGE_METER_ID_ESC_MOTOR_20 = 79,
	// 80-119 for cell meters (40 cells)
    VOLTAGE_METER_ID_CELL_1 = 80,          
    VOLTAGE_METER_ID_CELL_2,
    //...
    VOLTAGE_METER_ID_CELL_40 = 119,
} voltageMeterId_e;
	
