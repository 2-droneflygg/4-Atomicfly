/*********************************************************************************
 提供传感器相关描述信息。
*********************************************************************************/
#pragma once

/* ---------------------------传感器枚举-------------------------------- */	
typedef enum {
    SENSOR_GYRO = 1 << 0, 
    SENSOR_ACC = 1 << 1,
    SENSOR_BARO = 1 << 2,
    SENSOR_MAG = 1 << 3,
    SENSOR_GPS = 1 << 5,
    SENSOR_GPSMAG = 1 << 6
} sensors_e;

/* -------------------------传感器索引枚举------------------------------ */	
typedef enum {
    SENSOR_INDEX_GYRO = 0,   		// 陀螺仪
    SENSOR_INDEX_ACC,		 		// 加速度计
    SENSOR_INDEX_BARO,       		// 气压计
    SENSOR_INDEX_MAG,        		// 磁力计
    SENSOR_INDEX_COUNT		 		// 传感器数量
} sensorIndex_e;

/* -------------------------姿态描述结构体------------------------------ */	
typedef struct int16_flightDynamicsTrims_s {
    int16_t roll;                   // 横滚
    int16_t pitch;				    // 俯仰
    int16_t yaw;					// 偏航
    int16_t calibrationCompleted;   // 校准完成
} flightDynamicsTrims_def_t;

/* -------------------------校准零偏共用体------------------------------ */	
typedef union flightDynamicsTrims_u {
    int16_t raw[4];
    flightDynamicsTrims_def_t values;
} flightDynamicsTrims_t;

extern uint8_t detectedSensors[SENSOR_INDEX_COUNT];  
	
