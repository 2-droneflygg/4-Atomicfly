/***************************************************************************************
GPS数据解析（NMEA-0183协议）：
	NMEA协议：有0180、0182和0183这3种，0183可以认为是前两种的升级，也是目前使用最为广泛的一种.
GPS模块参数（北天BN880）：
	波特率：115200.
	输出数据更新频率：5HZ
	水平精度：2m.
	速度精度：0.1m/s
	时间精度：1us.
	启动时间：
		冷启动：26s，初次使用时或电池耗尽导致星历信息丢失时、关机状态下将接收机移动1000公里以上距离；
		温启动：25s，距离上次定位的时间超过两个小时的启动；
		热启动：1s.  距离上次定位的时间小于两个小时的启动.
输出数据：
	GGA：时间、位置、定位类型
	GLL：经度、纬度、UTC时间（世界统一时间，时分秒格式）
	GSA：GPS接收机操作模式、定位使用的卫星、DOP值（相对误差 - 越小精度越高）
	GSV：可见GPS卫星信息、仰角、方位角、信噪比（SNR）（单位dB - 分贝）
	RMC：时间、日期、位置、速度
	VTG：地面速度信息
***************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_GPS
#include "build/build_config.h"

#include "common/axis.h"
#include "common/gps_conversion.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/light_led.h"
#include "drivers/time.h"

#include "io/gps.h"
#include "io/serial.h"

#include "config/config.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/gps_rescue.h"

#include "sensors/sensors.h"

PG_REGISTER_WITH_RESET_TEMPLATE(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);
PG_RESET_TEMPLATE(gpsConfig_t, gpsConfig,
    .provider = GPS_NMEA,
    .autoConfig = GPS_AUTOCONFIG_ON,
    .autoBaud = GPS_AUTOBAUD_OFF,
    .gps_set_home_point_once = true,
    .gps_use_3d_speed = false,
    .sbas_integrity = false
);

#define LOG_ERROR        '?'
#define LOG_IGNORED      '!'
#define LOG_SKIPPED      '>'
#define LOG_NMEA_GGA     'g'
#define LOG_NMEA_RMC     'r'

#define GPS_SV_MAXSATS   16

char gpsPacketLog[GPS_PACKET_LOG_ENTRY_COUNT];
static char *gpsPacketLogChar = gpsPacketLog;

int32_t GPS_home[2];
uint16_t GPS_distanceToHome;        		// 到家的距离，单位为米
int16_t GPS_directionToHome;        		// 指向家的方向或以度数表示的hol点
uint32_t GPS_distanceFlownInCm;     		// 飞行距离以厘米为单位
int16_t GPS_verticalSpeedInCmS;     		// 垂直速度，单位为cm/s
float dTnav;            		    	    // Delta时间，以毫秒为单位进行导航计算，每一次良好的GPS读取都会更新
int16_t nav_takeoff_bearing;

#define GPS_DISTANCE_FLOWN_MIN_SPEED_THRESHOLD_CM_S 15 // 5.4Km/h 3.35mph

gpsSolutionData_t gpsSol;
uint32_t GPS_packetCount = 0;
uint32_t GPS_svInfoReceivedCount = 0; 		// SV = 空间飞行器，每次收到SV信息，计数器增加

uint8_t GPS_numCh;                          // Number of channels
uint8_t GPS_svinfo_chn[GPS_SV_MAXSATS];     // Channel number
uint8_t GPS_svinfo_svid[GPS_SV_MAXSATS];    // 卫星 ID
uint8_t GPS_svinfo_quality[GPS_SV_MAXSATS]; // 设置Qualtity
uint8_t GPS_svinfo_cno[GPS_SV_MAXSATS];     // 载波噪声比(信号强度)

// 错误波特率/断开/等的GPS超时时间(毫秒)(默认2.5秒)
#define GPS_TIMEOUT (2500)
// gpsInitData数组下面有多少项
#define GPS_INIT_ENTRIES (GPS_BAUDRATE_MAX + 1)
#define GPS_BAUDRATE_CHANGE_DELAY (200)
// GPS任务周期中等待ACK/NAK的超时时间(0.1s at 100Hz)
#define UBLOX_ACK_TIMEOUT_MAX_COUNT (10)

static serialPort_t *gpsPort;

/* --------------------------GPS初始化数据结构体------------------------ */	
typedef struct gpsInitData_s {
    uint8_t index;
    uint8_t baudrateIndex; // see baudRate_e
    const char *ubx;
    const char *mtk;
} gpsInitData_t;

// NMEA将通过这些循环，直到收到有效数据
static const gpsInitData_t gpsInitData[] = {
    { GPS_BAUDRATE_115200,  BAUD_115200, "$PUBX,41,1,0003,0001,115200,0*1E\r\n", "$PMTK251,115200*1F\r\n" },
    { GPS_BAUDRATE_57600,    BAUD_57600, "$PUBX,41,1,0003,0001,57600,0*2D\r\n", "$PMTK251,57600*2C\r\n" },
    { GPS_BAUDRATE_38400,    BAUD_38400, "$PUBX,41,1,0003,0001,38400,0*26\r\n", "$PMTK251,38400*27\r\n" },
    { GPS_BAUDRATE_19200,    BAUD_19200, "$PUBX,41,1,0003,0001,19200,0*23\r\n", "$PMTK251,19200*22\r\n" },
    // 9600对于5Hz的更新是不够的-留给只运行在这个速度的NMEA兼容性
    { GPS_BAUDRATE_9600,      BAUD_9600, "$PUBX,41,1,0003,0001,9600,0*16\r\n", "" }
};

#define GPS_INIT_DATA_ENTRY_COUNT (sizeof(gpsInitData) / sizeof(gpsInitData[0]))
#define DEFAULT_BAUD_RATE_INDEX 0

/* -----------------------------GPS状态枚举----------------------------- */	
typedef enum {
    GPS_UNKNOWN,							// GPS未知
    GPS_INITIALIZING,						// GPS正在初始化
    GPS_INITIALIZED,						// GPS初始化完成
    GPS_CHANGE_BAUD,
    GPS_CONFIGURE,
    GPS_RECEIVING_DATA,
    GPS_LOST_COMMUNICATION					// GPS丢失
} gpsState_e;

gpsData_t gpsData;

// 函数声明
static void gpsNewData(uint16_t c);
#ifdef USE_GPS_NMEA
static bool gpsNewFrameNMEA(char c);
#endif

/**********************************************************************
函数名称：shiftPacketLog
函数功能：改变包的日志
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
static void shiftPacketLog(void)
{
    uint32_t i;

    for (i = ARRAYLEN(gpsPacketLog) - 1; i > 0 ; i--) {
        gpsPacketLog[i] = gpsPacketLog[i-1];
    }
}

/**********************************************************************
函数名称：gpsSetState
函数功能：GPS设置状态
函数形参：state
函数返回值：None
函数描述：None
**********************************************************************/
static void gpsSetState(gpsState_e state)
{
    gpsData.state = state;
    gpsData.state_position = 0;
    gpsData.state_ts = millis();
    gpsData.messageState = GPS_MESSAGE_STATE_IDLE;
}

/**********************************************************************
函数名称：gpsInit
函数功能：GPS初始化
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void gpsInit(void)
{
    gpsData.baudrateIndex = 0;
    gpsData.errors = 0;
    gpsData.timeouts = 0;

    memset(gpsPacketLog, 0x00, sizeof(gpsPacketLog));

    // init gpsData结构，如果我们实际上没有启用，就不用做其他的事情了
    gpsSetState(GPS_UNKNOWN);

    gpsData.lastMessage = millis();

	// 查找配置为GPS功能的串口
    const serialPortConfig_t *gpsPortConfig = findSerialPortConfig(FUNCTION_GPS);
    if (!gpsPortConfig) {
        return;
    }

    while (gpsInitData[gpsData.baudrateIndex].baudrateIndex != gpsPortConfig->gps_baudrateIndex) {
        gpsData.baudrateIndex++;
        if (gpsData.baudrateIndex >= GPS_INIT_DATA_ENTRY_COUNT) {
            gpsData.baudrateIndex = DEFAULT_BAUD_RATE_INDEX;
            break;
        }
    }

    portMode_e mode = MODE_RXTX;

    // 打开串口并初始化 - 在gpsUpdate()中不会消耗回调缓冲区
    gpsPort = openSerialPort(gpsPortConfig->identifier, FUNCTION_GPS, NULL, NULL, baudRates[gpsInitData[gpsData.baudrateIndex].baudrateIndex], mode, SERIAL_NOT_INVERTED);
    if (!gpsPort) {
        return;
    }

    // 信号GPS“线程”初始化时，它得到它
    gpsSetState(GPS_INITIALIZING);
}

/**********************************************************************
函数名称：gpsInitNmea
函数功能：GPS初始化Nmea
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
#ifdef USE_GPS_NMEA
void gpsInitNmea(void)
{
    uint32_t now;
    switch (gpsData.state) {
        case GPS_INITIALIZING:
           now = millis();
           if (now - gpsData.state_ts < 1000) {
               return;
           }
           gpsData.state_ts = now;
           if (gpsData.state_position < 1) {
               serialSetBaudRate(gpsPort, 4800);
               gpsData.state_position++;
           } else if (gpsData.state_position < 2) {
               // 打印我们想要的波特率的固定初始化字符串
               serialPrint(gpsPort, "$PSRF100,1,115200,8,1,0*05\r\n");
               gpsData.state_position++;
           } else {
               // 我们现在(希望)是正确的速率，下一个状态将切换到它
               gpsSetState(GPS_CHANGE_BAUD);
           }
           break;
		   
        case GPS_CHANGE_BAUD:
           now = millis();
           if (now - gpsData.state_ts < 1000) {
               return;
           }
           gpsData.state_ts = now;
           if (gpsData.state_position < 1) {
               serialSetBaudRate(gpsPort, baudRates[gpsInitData[gpsData.baudrateIndex].baudrateIndex]);
               gpsData.state_position++;
           } else if (gpsData.state_position < 2) {
               serialPrint(gpsPort, "$PSRF103,00,6,00,0*23\r\n");
               gpsData.state_position++;
           } else
               gpsSetState(GPS_RECEIVING_DATA);
            break;
    }
}
#endif // USE_GPS_NMEA

/**********************************************************************
函数名称：gpsInitHardware
函数功能：GPS硬件初始化
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void gpsInitHardware(void)
{
    switch (gpsConfig()->provider) {
    case GPS_NMEA:
#ifdef USE_GPS_NMEA
        gpsInitNmea();
#endif
        break;
    default:
        break;
    }
}

/**********************************************************************
函数名称：updateGpsIndicator
函数功能：更新GPS指示器
函数形参：currentTimeUs
函数返回值：None
函数描述：None
**********************************************************************/
static void updateGpsIndicator(timeUs_t currentTimeUs)
{
    static uint32_t GPSLEDTime;
    if ((int32_t)(currentTimeUs - GPSLEDTime) >= 0 && (gpsSol.numSat >= 5)) {
        GPSLEDTime = currentTimeUs + 150000;
    }
}

/**********************************************************************
函数名称：gpsUpdate
函数功能：GPS更新
函数形参：currentTimeUs
函数返回值：None
函数描述：
	由调度器以任务形式调用.
**********************************************************************/
void gpsUpdate(timeUs_t currentTimeUs)
{
    // 读出可用的GPS字节
    if (gpsPort) {
        while (serialRxBytesWaiting(gpsPort)) {
            gpsNewData(serialRead(gpsPort));
        }
    } 

	// 轮询GPS线程状态
    switch (gpsData.state) {
        case GPS_UNKNOWN:
        case GPS_INITIALIZED:
            break;
        case GPS_INITIALIZING:				 // GPS初始化
        case GPS_CHANGE_BAUD:
        case GPS_CONFIGURE:
			// GPS硬件初始化
            gpsInitHardware();
            break;

        case GPS_LOST_COMMUNICATION:		 // GPS丢失
            gpsData.timeouts++;
			// 尝试另一个波特率
            if (gpsConfig()->autoBaud) {
                gpsData.baudrateIndex++;
                gpsData.baudrateIndex %= GPS_INIT_ENTRIES;
            }
            gpsData.lastMessage = millis();
            gpsSol.numSat = 0;
            DISABLE_STATE(GPS_FIX);
            gpsSetState(GPS_INITIALIZING);
            break;

        case GPS_RECEIVING_DATA:             // 检查有无数据/gps超时/导线断开等
            if (millis() - gpsData.lastMessage > GPS_TIMEOUT) {
                // 移除GPS功能
                sensorsClear(SENSOR_GPS);
                gpsSetState(GPS_LOST_COMMUNICATION);
            }
            break;
    }

	// 更新GPS指示器
    if (sensors(SENSOR_GPS)) {
        updateGpsIndicator(currentTimeUs);
    }

	// 如果未解锁且未设置家的位置
    if (!ARMING_FLAG(ARMED) && !gpsConfig()->gps_set_home_point_once) {
        DISABLE_STATE(GPS_FIX_HOME);
    }
	
	// 更新GPS救援模式状态
#if defined(USE_GPS_RESCUE)
    if (gpsRescueIsConfigured()) {
        updateGPSRescueState();
    }
#endif
}


/**********************************************************************
函数名称：gpsNewData
函数功能：GPS新数据
函数形参：c
函数返回值：None
函数描述：None
**********************************************************************/
static void gpsNewData(uint16_t c)
{
	// 判断GPS帧正确性
    if (!gpsNewFrame(c)) {
        return;
    }

    // 接收并解析新数据
    gpsData.lastLastMessage = gpsData.lastMessage;
    gpsData.lastMessage = millis();
    sensorsSet(SENSOR_GPS);
    onGpsNewData();
}

/**********************************************************************
函数名称：gpsNewFrame
函数功能：GPS新数据帧
函数形参：c
函数返回值：None
函数描述：None
**********************************************************************/
bool gpsNewFrame(uint8_t c)
{
    switch (gpsConfig()->provider) {
    case GPS_NMEA:          // NMEA
#ifdef USE_GPS_NMEA
        return gpsNewFrameNMEA(c);
#endif
        break;
    default:
        break;
    }
    return false;
}

/**********************************************************************
函数名称：gpsIsHealthy
函数功能：检查通信是否正常
函数形参：None
函数返回值：状态 
函数描述：None
**********************************************************************/
bool gpsIsHealthy()
{
    return (gpsData.state == GPS_RECEIVING_DATA);
}

// 辅助功能
#ifdef USE_GPS_NMEA
/**********************************************************************
函数名称：grab_fields
函数功能：获取字段
函数形参：src，mult
函数返回值：result
函数描述：None
**********************************************************************/
static uint32_t grab_fields(char *src, uint8_t mult)
{                               // 将字符串转换为uint32
    uint32_t i;
    uint32_t tmp = 0;
    int isneg = 0;
    for (i = 0; src[i] != 0; i++) {
        if ((i == 0) && (src[0] == '-')) {  // 检测负号
            isneg = 1;
            continue; 						// 如果第一个字符是负号，则跳到下一个字符
        }
        if (src[i] == '.') {
            i++;
            if (mult == 0) {
                break;
            } else {
                src[i + mult] = 0;
            }
        }
        tmp *= 10;
        if (src[i] >= '0' && src[i] <= '9') {
            tmp += src[i] - '0';
        }
        if (i >= 15) {
            return 0; // 越界
        }
    }
    return isneg ? -tmp : tmp;    // 处理负面的海拔
}

/*	GPS帧解码的轻实现：
	这应该适用于大多数配置为输出5帧的现代GPS设备
	它假定在串行总线上有一些NMEA GGA帧要解码，现在在应用数据之前正确地验证校验和
	这里我们只使用了以下数据:
	- 纬度
	- 经度
	- GPS定位 is/is not ok
	- GPS卫星数(4足够可靠)
	添加到管理信息系统
	- GPS高度(用于OSD显示)
	- GPS速度(用于OSD显示)
*/
#define NO_FRAME   0
#define FRAME_GGA  1
#define FRAME_RMC  2
#define FRAME_GSV  3

/* --------------------------GPS数据名称结构体-------------------------- */	
typedef struct gpsDataNmea_s {
    int32_t latitude;
    int32_t longitude;
    uint8_t numSat;
    int32_t altitudeCm;
    uint16_t speed;
    uint16_t hdop;
    uint16_t ground_course;
    uint32_t time;
    uint32_t date;
} gpsDataNmea_t;

/**********************************************************************
函数名称：gpsNewFrameNMEA
函数功能：GPS新NMEA数据帧
函数形参：字符
函数返回值：状态
函数描述：None
**********************************************************************/
static bool gpsNewFrameNMEA(char c)
{
    static gpsDataNmea_t gps_Msg;

    uint8_t frameOK = 0;
    static uint8_t param = 0, offset = 0, parity = 0;
    static char string[15];
    static uint8_t checksum_param, gps_frame = NO_FRAME;
    static uint8_t svMessageNum = 0;
    uint8_t svSatNum = 0, svPacketIdx = 0, svSatParam = 0;

    switch (c) {
        case '$':
            param = 0;
            offset = 0;
            parity = 0;
            break;
        case ',':
        case '*':
            string[offset] = 0;
            if (param == 0) {       // 帧识别
                gps_frame = NO_FRAME;
                if (0 == strcmp(string, "GPGGA") || 0 == strcmp(string, "GNGGA")) {
                    gps_frame = FRAME_GGA;
                } else if (0 == strcmp(string, "GPRMC") || 0 == strcmp(string, "GNRMC")) {
                    gps_frame = FRAME_RMC;
                } else if (0 == strcmp(string, "GPGSV")) {
                    gps_frame = FRAME_GSV;
                }
            }

            switch (gps_frame) {
                case FRAME_GGA:        //************* GPGGA FRAME parsing
                    switch (param) {
                        case 2:
                            gps_Msg.latitude = GPS_coord_to_degrees(string);
                            break;
                        case 3:
                            if (string[0] == 'S')
                                gps_Msg.latitude *= -1;
                            break;
                        case 4:
                            gps_Msg.longitude = GPS_coord_to_degrees(string);
                            break;
                        case 5:
                            if (string[0] == 'W')
                                gps_Msg.longitude *= -1;
                            break;
                        case 6:
                            if (string[0] > '0') {
                                ENABLE_STATE(GPS_FIX);
                            } else {
                                DISABLE_STATE(GPS_FIX);
                            }
                            break;
                        case 7:
                            gps_Msg.numSat = grab_fields(string, 0);
                            break;
                        case 8:
                            gps_Msg.hdop = grab_fields(string, 1) * 100;          		   // 水平精度
                            break;
                        case 9:
                            gps_Msg.altitudeCm = grab_fields(string, 1) * 10;     		   // 厘米的高度。注:NMEA提供1或3位小数的高度，在0.1米处切割，再乘以10会更安全
                            break;
                    }
                    break;
                case FRAME_RMC:        //************* GPRMC FRAME parsing
                    switch (param) {
                        case 1:
                            gps_Msg.time = grab_fields(string, 2); 						   // UTC时间hhmmss.ss
                            break;
                        case 7:
                            gps_Msg.speed = ((grab_fields(string, 1) * 5144L) / 1000L);    // 管理信息系统增加了cm/s的速度
                            break;
                        case 8:
                            gps_Msg.ground_course = (grab_fields(string, 1));              // 地面温度* 10度
                            break;
                        case 9:
                            gps_Msg.date = grab_fields(string, 0);						   // 日期dd / mm / yy
                            break;
                    }
                    break;
                case FRAME_GSV:
                    switch (param) {
                        case 2:
                            // 消息编号
                            svMessageNum = grab_fields(string, 0);
                            break;
                        case 3:
                            // 可见SVs的总数
                            GPS_numCh = grab_fields(string, 0);
                            break;
                    }
                    if (param < 4)
                        break;

                    svPacketIdx = (param - 4) / 4 + 1; 					  				   // 分组卫星号，1-4
                    svSatNum    = svPacketIdx + (4 * (svMessageNum - 1)); 				   // 卫星数量
                    svSatParam  = param - 3 - (4 * (svPacketIdx - 1));    				   // 卫星参数编号

                    if (svSatNum > GPS_SV_MAXSATS)
                        break;

                    switch (svSatParam) {
                        case 1:
                            // SV打印数量
                            GPS_svinfo_chn[svSatNum - 1]  = svSatNum;
                            GPS_svinfo_svid[svSatNum - 1] = grab_fields(string, 0);
                            break;
                        case 4:
                            // 信噪比，00到99 dB(无跟踪时为null)
                            GPS_svinfo_cno[svSatNum - 1] = grab_fields(string, 0);
                            GPS_svinfo_quality[svSatNum - 1] = 0; 		  				   // 仅供ublox使用
                            break;
                    }

                    GPS_svInfoReceivedCount++;

                    break;
            }

            param++;
            offset = 0;
            if (c == '*')
                checksum_param = 1;
            else
                parity ^= c;
            break;
        case '\r':
        case '\n':
            if (checksum_param) {   													   // 奇偶校验和
                shiftPacketLog();
                uint8_t checksum = 16 * ((string[0] >= 'A') ? string[0] - 'A' + 10 : string[0] - '0') + ((string[1] >= 'A') ? string[1] - 'A' + 10 : string[1] - '0');
                if (checksum == parity) {
                    *gpsPacketLogChar = LOG_IGNORED;
                    GPS_packetCount++;
                    switch (gps_frame) {
                    case FRAME_GGA:
                      *gpsPacketLogChar = LOG_NMEA_GGA;
                      frameOK = 1;
                      if (STATE(GPS_FIX)) {
                            gpsSol.llh.lat = gps_Msg.latitude;
                            gpsSol.llh.lon = gps_Msg.longitude;
                            gpsSol.numSat = gps_Msg.numSat;
                            gpsSol.llh.altCm = gps_Msg.altitudeCm;
                            gpsSol.hdop = gps_Msg.hdop;
                        }
                        break;
                    case FRAME_RMC:
                        *gpsPacketLogChar = LOG_NMEA_RMC;
                        gpsSol.groundSpeed = gps_Msg.speed;
                        gpsSol.groundCourse = gps_Msg.ground_course;
                        break;
                    } // end switch
                } else {
                    *gpsPacketLogChar = LOG_ERROR;
                }
            }
            checksum_param = 0;
            break;
        default:
            if (offset < 15)
                string[offset++] = c;
            if (!checksum_param)
                parity ^= c;
    }
    return frameOK;
}
#endif // USE_GPS_NMEA

/**********************************************************************
函数名称：GPS_calc_longitude_scaling
函数功能：计算GPS经度缩放
函数形参：lat
函数返回值：None
函数描述：None
**********************************************************************/
// 用来抵消向两极前进时收缩的经度
float GPS_scaleLonDown = 1.0f;  
void GPS_calc_longitude_scaling(int32_t lat)
{
    float rads = (fabsf((float)lat) / 10000000.0f) * 0.0174532925f;
    GPS_scaleLonDown = cos_approx(rads);
}

/**********************************************************************
函数名称：GPS_calculateDistanceFlownVerticalSpeed
函数功能：计算飞行距离和垂直速度
函数形参：initialize
函数返回值：None
函数描述：None
**********************************************************************/
static void GPS_calculateDistanceFlownVerticalSpeed(bool initialize)
{
    static int32_t lastCoord[2] = { 0, 0 };
    static int32_t lastAlt;
    static int32_t lastMillis;

    int currentMillis = millis();

    if (initialize) {
        GPS_distanceFlownInCm = 0;
        GPS_verticalSpeedInCmS = 0;
    } else {
        if (STATE(GPS_FIX_HOME) && ARMING_FLAG(ARMED)) {
			// 判断使用速度类型
            uint16_t speed = gpsConfig()->gps_use_3d_speed ? gpsSol.speed3d : gpsSol.groundSpeed;
            // 判断当前速度是否大于最小阈值
            if (speed > GPS_DISTANCE_FLOWN_MIN_SPEED_THRESHOLD_CM_S) {
                uint32_t dist;
                int32_t dir;
				// 计算上个点和当前点之间的距离
                GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &lastCoord[LAT], &lastCoord[LON], &dist, &dir);
                if (gpsConfig()->gps_use_3d_speed) {
                    dist = sqrtf(powf(gpsSol.llh.altCm - lastAlt, 2.0f) + powf(dist, 2.0f));
                }
				// 累加
                GPS_distanceFlownInCm += dist;
            }
        }
		// 计算垂直速度
        GPS_verticalSpeedInCmS = (gpsSol.llh.altCm - lastAlt) * 1000 / (currentMillis - lastMillis);
		// 约束
        GPS_verticalSpeedInCmS = constrain(GPS_verticalSpeedInCmS, -1500, 1500);
    }
	// 记录当前数据为下一次使用
    lastCoord[LON] = gpsSol.llh.lon;
    lastCoord[LAT] = gpsSol.llh.lat;
    lastAlt = gpsSol.llh.altCm;
    lastMillis = currentMillis;
}

/**********************************************************************
函数名称：GPS_reset_home_position
函数功能：复位家的位置
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void GPS_reset_home_position(void)
{
    if (!STATE(GPS_FIX_HOME) || !gpsConfig()->gps_set_home_point_once) {
        if (STATE(GPS_FIX) && gpsSol.numSat >= 5) {
            GPS_home[LAT] = gpsSol.llh.lat;
            GPS_home[LON] = gpsSol.llh.lon;
			// 距离和方位计算需要一个初始值
            GPS_calc_longitude_scaling(gpsSol.llh.lat); 					 
            // 家的位置已确定
            ENABLE_STATE(GPS_FIX_HOME);
        }
    }
	// 计算飞行距离和垂直速度
    GPS_calculateDistanceFlownVerticalSpeed(true);      					
}

/**********************************************************************
函数名称：GPS_reset_home_position
函数功能：获取两点之间的距离，单位为cm，获取从pos1到pos2的方位，返回1deg = 100的精度
函数形参：currentLat1，currentLon1，destinationLat2，destinationLon2，dist，bearing
函数返回值：None
函数描述：None
**********************************************************************/
// 赤道上两个经度点之间的距离 - 单位：百公里
#define DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS 1.113195f
#define TAN_89_99_DEGREES 5729.57795f
void GPS_distance_cm_bearing(int32_t *currentLat1, int32_t *currentLon1, int32_t *destinationLat2, int32_t *destinationLon2, uint32_t *dist, int32_t *bearing)
{
	// 纬度差 - 万分之一度
    float dLat = *destinationLat2 - *currentLat1; 							 
    // 经度差 
    float dLon = (float)(*destinationLon2 - *currentLon1) * GPS_scaleLonDown;
    // 距离
    *dist = sqrtf(sq(dLat) + sq(dLon)) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS;
	// 方向 - 转换输出弧度为100倍*度
    *bearing = 9000.0f + atan2_approx(-dLat, dLon) * TAN_89_99_DEGREES;  
    if (*bearing < 0)
        *bearing += 36000;
}

/**********************************************************************
函数名称：GPS_calculateDistanceAndDirectionToHome
函数功能：计算回家的距离和方向
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void GPS_calculateDistanceAndDirectionToHome(void)
{
	// 如果没有家的位置，则不要计算
    if (STATE(GPS_FIX_HOME)) {      
		// 距离 
        uint32_t dist;        
		// 方向
        int32_t dir;							
		// 获取当前位置距离家的位置
        GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &GPS_home[LAT], &GPS_home[LON], &dist, &dir);
		// 距离：米
		GPS_distanceToHome = dist / 100;	
		// 方向：度
        GPS_directionToHome = dir / 100;		
    } else {
        GPS_distanceToHome = 0;
        GPS_directionToHome = 0;
    }
}

/**********************************************************************
函数名称：onGpsNewData
函数功能：对GPS新数据的使用
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void onGpsNewData(void)
{
	// 判断是否达到定位条件
    if (!(STATE(GPS_FIX) && gpsSol.numSat >= 5)) {
        return;
    }

    // 计算导航回路的时差（单位：秒），范围0-1.0f，计算x,y速度和导航pid的时间
    static uint32_t nav_loopTimer;
	// （当前时间节拍-上一次时间节拍）/ 1000
    dTnav = (float)(millis() - nav_loopTimer) / 1000.0f;
	// 记录当前时间节拍为下一次使用
    nav_loopTimer = millis();
    // 限幅 - 防止错误的GPS数据
    dTnav = MIN(dTnav, 1.0f);

	// 计算回家的距离和方向
    GPS_calculateDistanceAndDirectionToHome();
	// 如果解锁 - 计算飞行距离和垂直速度
    if (ARMING_FLAG(ARMED)) {
        GPS_calculateDistanceFlownVerticalSpeed(false);
    }

#ifdef USE_GPS_RESCUE
	// 如果有新的GPS数据适用的话,更新救援模式主航向
    rescueNewGpsData();
#endif
}
#endif

