#pragma once

#include "common/axis.h"
#include "common/time.h"

#include "pg/pg.h"

#define LAT 0
#define LON 1

#define GPS_DEGREES_DIVIDER 10000000L
#define GPS_X 1
#define GPS_Y 0

/* --------------------------GPS提供枚举-------------------------- */	
typedef enum {
    GPS_NMEA = 0,
} gpsProvider_e;

/* --------------------------GPS波特率枚举------------------------ */	
typedef enum {
    GPS_BAUDRATE_115200 = 0,
    GPS_BAUDRATE_57600,
    GPS_BAUDRATE_38400,
    GPS_BAUDRATE_19200,
    GPS_BAUDRATE_9600
} gpsBaudRate_e;
#define GPS_BAUDRATE_MAX GPS_BAUDRATE_9600

/* -------------------------GPS自动配置枚举----------------------- */	
typedef enum {
    GPS_AUTOCONFIG_OFF = 0,
    GPS_AUTOCONFIG_ON
} gpsAutoConfig_e;

/* ------------------------GPS自动波特率枚举---------------------- */	
typedef enum {
    GPS_AUTOBAUD_OFF = 0,
    GPS_AUTOBAUD_ON
} gpsAutoBaud_e;

/* --------------------------GPS配置结构体------------------------ */	
typedef struct gpsConfig_s {
    gpsProvider_e provider;
    gpsAutoConfig_e autoConfig;
    gpsAutoBaud_e autoBaud;
    uint8_t gps_set_home_point_once;
    uint8_t gps_use_3d_speed;
    uint8_t sbas_integrity;
} gpsConfig_t;
// 声明GPS配置结构体
PG_DECLARE(gpsConfig_t, gpsConfig);

/* --------------------------GPS坐标结构体------------------------ */	
typedef struct gpsCoordinateDDDMMmmmm_s {
    int16_t dddmm;
    int16_t mmmm;
} gpsCoordinateDDDMMmmmm_t;

/* --------------------------GPS定位结构体------------------------ */	
typedef struct gpsLocation_s {
    int32_t lat;                    		 // latitude * 1e+7
    int32_t lon;                    		 // longitude * 1e+7
    int32_t altCm;                  		 // altitude in 0.01m
} gpsLocation_t;

/* -------------------------GPS解析数据结构体--------------------- */	
typedef struct gpsSolutionData_s {
    gpsLocation_t llh;						 // 经纬度
    uint16_t speed3d;               		 // speed in 0.1m/s
    uint16_t groundSpeed;           		 // speed in 0.1m/s
    uint16_t groundCourse;          		 // degrees * 10
    uint16_t hdop;                  		 // 通用HDOP值(*100) - 度量卫星相对于观测者的几何位置所造成误差（卫星几何效应）的单位
    uint8_t numSat;							 // 星数
} gpsSolutionData_t;

/* --------------------------GPS信息状态枚举---------------------- */	
typedef enum {
    GPS_MESSAGE_STATE_IDLE = 0,
    GPS_MESSAGE_STATE_INIT,
    GPS_MESSAGE_STATE_SBAS,
    GPS_MESSAGE_STATE_GNSS,
    GPS_MESSAGE_STATE_INITIALIZED,
    GPS_MESSAGE_STATE_PEDESTRIAN_TO_AIRBORNE,
    GPS_MESSAGE_STATE_ENTRY_COUNT
} gpsMessageState_e;

/* ---------------------------GPS数据结构体----------------------- */	
typedef struct gpsData_s {
    uint32_t errors;                		 // gps误差计数器- crc错误/数据丢失/同步等
    uint32_t timeouts;
    uint32_t lastMessage;           		 // 上次接收有效GPS数据(millis)
    uint32_t lastLastMessage;       		 // 最后一次有效的GPS信息，用于计算增量

    uint32_t state_position;        		 // 循环增量变量
    uint32_t state_ts;              		 // 最后状态位置增量的时间戳
    uint8_t state;                  		 // GPS线程状态，用于检测电缆断开和配置附加设备
    uint8_t baudrateIndex;          	 	 // 索引为自动检测或当前波特率
    gpsMessageState_e messageState;

    uint8_t ackWaitingMsgId;        		 // 等待ACK时的消息id
    uint8_t ackTimeoutCounter;      		 // 应答超时计数器
} gpsData_t;

#define GPS_PACKET_LOG_ENTRY_COUNT 21        // 为了使这一点有用，应该记录尽可能多的数据包


extern char gpsPacketLog[GPS_PACKET_LOG_ENTRY_COUNT];
extern int32_t GPS_home[2];
extern uint16_t GPS_distanceToHome;         
extern int16_t GPS_directionToHome;          
extern uint32_t GPS_distanceFlownInCm;      
extern int16_t GPS_verticalSpeedInCmS;   
extern int16_t GPS_angle[ANGLE_INDEX_COUNT]; 
extern float dTnav;                          
extern float GPS_scaleLonDown;              
extern int16_t nav_takeoff_bearing;
extern gpsData_t gpsData;
extern gpsSolutionData_t gpsSol;
extern uint32_t GPS_packetCount;
extern uint32_t GPS_svInfoReceivedCount;
extern uint8_t GPS_numCh;                  
extern uint8_t GPS_svinfo_chn[16];         
extern uint8_t GPS_svinfo_svid[16];      
extern uint8_t GPS_svinfo_quality[16];    
extern uint8_t GPS_svinfo_cno[16];      
void gpsInit(void);
void gpsUpdate(timeUs_t currentTimeUs);
bool gpsNewFrame(uint8_t c);
bool gpsIsHealthy(void);
struct serialPort_s;
void onGpsNewData(void);
void GPS_reset_home_position(void);
void GPS_calc_longitude_scaling(int32_t lat);
void GPS_distance_cm_bearing(int32_t *currentLat1, int32_t *currentLon1, int32_t *destinationLat2, int32_t *destinationLon2, uint32_t *dist, int32_t *bearing);

