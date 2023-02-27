#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "platform.h"

#include "common/maths.h"

#include "scheduler/scheduler.h"

#include "config/config.h"
#include "config/feature.h"

#include "pg/beeper.h"
#include "pg/motor.h"
#include "pg/rx.h"

#include "drivers/serial.h"
#include "drivers/motor.h"
#include "drivers/dshot.h"
#include "drivers/dshot_dpwm.h"
#include "drivers/osd.h"
#include "drivers/display.h"
#include "drivers/vtx_common.h"

#include "io/gps.h"
#include "io/beeper.h"
#include "io/vtx.h"

#include "fc/rc.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/core.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"
#include "flight/failsafe.h"
#include "flight/gps_rescue.h"
#include "flight/mixer.h"
#include "flight/position.h"
#include "flight/imu.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"

#include "rx/rx.h"

#include "osd/osd.h"

#include "msp_command.h"

/**********************************************************************
函数名称：mspRebootFn
函数功能：MSP重启
函数形参：serialPort
函数返回值：None  
函数描述：None 
**********************************************************************/
static void mspRebootFn(serialPort_t *serialPort)
{
	// 未使用该变量
    UNUSED(serialPort);
	// 关闭电机
    motorShutdown();
	// 重启固件
	systemReset();
    // 不返回控制
    while(true);
}

/**********************************************************************
函数名称：mspCommonProcessOutCommand
函数功能：MSP命令回复（上位机索要数据）
函数形参：MSP命令，命令回复缓存，后置处理进程
函数返回值：命令被处理：true，未处理：false
函数描述：None
**********************************************************************/
bool mspProcessOutCommand(int16_t cmdMSP, sbuf_t *dst, mspPostProcessFnPtr *mspPostProcessFn)
{
	switch (cmdMSP) {
		case MSP_STATUS:
			// ----------------------------------MSP相关信息
			// ---------------传感器状态
			sbufWriteU8(dst, sensors(SENSOR_GYRO) | sensors(SENSOR_ACC) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_BARO) << 3 | sensors(SENSOR_GPS) << 4);
			// ---------------禁止解锁标志
			const uint32_t armingDisableFlags = getArmingDisableFlags();
            sbufWriteU32(dst, armingDisableFlags);
			// ---------------CPU负载
			uint16_t systemLoad = getAverageSystemLoadPercent();
			sbufWriteU16(dst,systemLoad);
			// ---------------循环时间
			int32_t timeDelta = getTaskDeltaTimeUs(TASK_PID);
			sbufWriteU32(dst, timeDelta);
			// ---------------GPS数据
			sbufWriteU8(dst, gpsSol.numSat);
	        sbufWriteU32(dst, gpsSol.llh.lat);
	        sbufWriteU32(dst, gpsSol.llh.lon);
		break;
		case MSP_ATTITUDE:
			// ----------------------------------姿态解算数据
	        sbufWriteU16(dst, DECIDEGREES_TO_DEGREES(attitude.values.roll));
	        sbufWriteU16(dst, DECIDEGREES_TO_DEGREES(attitude.values.pitch));
	        sbufWriteU16(dst, DECIDEGREES_TO_DEGREES(attitude.values.yaw));
		break;
		case MSP_ACC_CALIBRATION_STATUS:
			// ----------------------------------加速度计校准状态
			if(accIsCalibrationComplete()) {
				// 校准完成
				sbufWriteU8(dst, 1);
			} else {
				// 校准中
				sbufWriteU8(dst, 0);
			}
		break;
		case MSP_MAG_CALIBRATION_STATUS:
			// ----------------------------------磁力计校准状态
			if(compassIsCalibrationComplete()) {
				// 校准完成
				sbufWriteU8(dst, 1);
			} else {
				// 校准中
				sbufWriteU8(dst, 0);
			}
		break;
		case MSP_SYSTEM_CONFIG:
			// ----------------------------------系统配置
			// ---------------PID循环更新频率
			sbufWriteU8(dst, pidConfig()->pid_process_denom);
			// ---------------最大解锁角度
			sbufWriteU8(dst, currentPidProfile->levelAngleLimit);
			// ---------------磁力计启用状态
			if(compassConfig()->mag_hardware != MAG_NONE) {
				sbufWriteU8(dst, 1);
			} else {
				sbufWriteU8(dst, 0);
			}
			// ---------------气压计启用状态
			if(barometerConfig()->baro_hardware != BARO_NONE) {
				sbufWriteU8(dst, 1);
			} else {
				sbufWriteU8(dst, 0);
			}
			// ---------------陀螺仪/加速度计板对齐
			sbufWriteU8(dst, gyroDeviceConfig(0)->alignment);
			// ---------------磁力计板对齐
			sbufWriteU8(dst, compassConfig()->mag_alignment);
			// ---------------蜂鸣器开启状态
			sbufWriteU16(dst, beeperConfig()->beeper_off_flags);
			// ---------------电机怠速数值
			sbufWriteU16(dst, motorConfig()->digitalIdleOffsetValue);
			// ---------------信标音调
			sbufWriteU8(dst, beeperConfig()->dshotBeaconTone);
			// ---------------通过电机发出蜂鸣音&信号丢失鸣叫
			sbufWriteU32(dst, beeperConfig()->dshotBeaconOffFlags);
			// ---------------高度源类型
			sbufWriteU8(dst, positionConfig()->altSource);
		break;
		case MSP_BATTERY_STATUS:
			// ----------------------------------电池状态
			// ---------------电池存在状态
			sbufWriteU8(dst, getBatteryState());
			// ---------------电池电压
			sbufWriteU16(dst, getBatteryVoltage());
			// ---------------电池电流
			// send current in 0.01 A steps, range is -320A to 320A
			sbufWriteU16(dst, (int16_t)constrain(getAmperage(), -0x8000, 0x7FFF)); 
		break;
		case MSP_BATTERY_CONFIG:
			// ----------------------------------电池配置
			// ---------------最低单芯电压
			sbufWriteU8(dst, batteryConfig()->vbatmincellvoltage / 10);
			// ---------------最高单芯电压
			sbufWriteU8(dst, batteryConfig()->vbatmaxcellvoltage / 10);
			// ---------------警告单芯电压
			sbufWriteU8(dst, batteryConfig()->vbatwarningcellvoltage / 10);
		break;
		case MSP_VOLTAGE_CONFIG:
			// ----------------------------------电压传感器配置
			// ---------------比例
			sbufWriteU8(dst, voltageSensorADCConfig(0)->vbatscale);
			// ---------------电阻分压数值
			sbufWriteU8(dst, voltageSensorADCConfig(0)->vbatresdivval);
			// ---------------倍数
			sbufWriteU8(dst, voltageSensorADCConfig(0)->vbatresdivmultiplier);
		break;
		case MSP_CURRENT_CONFIG:
			// ----------------------------------电流传感器配置
			// ---------------比例
			sbufWriteU16(dst, currentSensorADCConfig()->scale);
			// ---------------偏移量
			sbufWriteU16(dst, currentSensorADCConfig()->offset);
		break;
		case MSP_HEARTBEAT:
			// ----------------------------------心跳包
		break;
		case MSP_FAILSAFE_CONFIG:
			// ----------------------------------失控保护配置
			// ---------------失控保护开关行为
			sbufWriteU8(dst, failsafeConfig()->failsafe_switch_mode);
			// ---------------触发二阶失控保护信号丢失时间
			sbufWriteU8(dst, failsafeConfig()->failsafe_delay);
			// ---------------低油门失控保护延迟
			sbufWriteU16(dst, failsafeConfig()->failsafe_throttle_low_delay);
		break;
		case MSP_RXFAIL_CONFIG:
			// ----------------------------------一阶失控保护通道预设
			// RPYT 1-12 -------mode，step
	        for (int i = 0; i < 16; i++) {
	            sbufWriteU8(dst, rxFailsafeChannelConfigs(i)->mode);
	            sbufWriteU16(dst, RXFAIL_STEP_TO_CHANNEL_VALUE(rxFailsafeChannelConfigs(i)->step));
	        }
		break;
		case MSP_GPS_RESCUE_CONFIG1:
			// ----------------------------------二阶失控保护
			// ---------------有效脉冲范围设置
			sbufWriteU16(dst, rxConfig()->rx_min_usec);
        	sbufWriteU16(dst, rxConfig()->rx_max_usec);
			// ---------------二阶失控保护措施
			// 失控保护措施
			sbufWriteU8(dst, failsafeConfig()->failsafe_procedure);
			// 允许强制解锁
			sbufWriteU16(dst, gpsRescueConfig()->allowArmingWithoutFix);
			// 设置单次返航点
			sbufWriteU8(dst, gpsConfig()->gps_set_home_point_once);
		break;
		case MSP_GPS_RESCUE_CONFIG2:
			// ---------------GPS救援设置
			// 角度
	        sbufWriteU16(dst, gpsRescueConfig()->angle);
			// 地面速度
	        sbufWriteU16(dst, gpsRescueConfig()->rescueGroundspeed);
			// 初始高度
	        sbufWriteU16(dst, gpsRescueConfig()->initialAltitudeM);
			// 下降距离
	        sbufWriteU16(dst, gpsRescueConfig()->descentDistanceM);
			// 最小油门
	        sbufWriteU16(dst, gpsRescueConfig()->throttleMin);
			// 最大油门
	        sbufWriteU16(dst, gpsRescueConfig()->throttleMax);
			// 上升速率
			sbufWriteU16(dst, gpsRescueConfig()->ascendRate);
			// 下降速率
			sbufWriteU16(dst, gpsRescueConfig()->descendRate);
			// 最小星数
	        sbufWriteU8(dst,  gpsRescueConfig()->minSats);
			// 悬停油门
	        sbufWriteU16(dst, gpsRescueConfig()->throttleHover);
			// 最小距离
			sbufWriteU16(dst,  gpsRescueConfig()->minRescueDth);
			// 高度模式
			sbufWriteU8(dst,  gpsRescueConfig()->altitudeMode);
		break;
		case MSP_PID_RATE_PROFILE:
			// ---------------PID&&RATE配置文件
			sbufWriteU8(dst, getCurrentPidProfileIndex());
			sbufWriteU8(dst, getCurrentControlRateProfileIndex());
		break;
		case MSP_PID_CONFIG:
			// ---------------PID配置
			// 内环PID - P I DMAX DMIN F
	        for (int i = 0; i < PID_LEVEL; i++) {
	            sbufWriteU8(dst, currentPidProfile->pid[i].P);
	            sbufWriteU8(dst, currentPidProfile->pid[i].I);
	            sbufWriteU8(dst, currentPidProfile->pid[i].D);
				sbufWriteU8(dst, currentPidProfile->d_min[i]);
				sbufWriteU16(dst, currentPidProfile->pid[i].F);
	        }
			// 外环P
			sbufWriteU8(dst, currentPidProfile->pid[PID_LEVEL].P);
			// 自稳模式角度限制
			sbufWriteU8(dst, currentPidProfile->levelAngleLimit);
			// 油门增压
			sbufWriteU8(dst, currentPidProfile->throttle_boost);
			// I项释放开启状态 && I项释放设定轴
			sbufWriteU8(dst, currentPidProfile->iterm_relax);
			// I项释放截止频率类型
			sbufWriteU8(dst, currentPidProfile->iterm_relax_type);
			// DMIN增益
			sbufWriteU8(dst, currentPidProfile->d_min_gain);
			// DMIN超前
			sbufWriteU8(dst, currentPidProfile->d_min_advance);
			// 反重力开启状态
			uint8_t ANTI_GRAVITY = featureIsEnabled(FEATURE_ANTI_GRAVITY);
			sbufWriteU8(dst, ANTI_GRAVITY);
			// 反重力模式
			sbufWriteU8(dst, currentPidProfile->antiGravityMode);			
		break;
		case MSP_RATE_CONFIG:
			// ---------------RATE配置
	        for (int i = 0 ; i < 3; i++) {
				sbufWriteU8(dst, currentControlRateProfile->rcRates[i]);
				sbufWriteU8(dst, currentControlRateProfile->rates[i]); 
	        	sbufWriteU8(dst, currentControlRateProfile->rcExpo[i]);
        	}
		break;
		case MSP_THR_CONFIG:
			// ---------------THR配置
			// 油门限制类型
			sbufWriteU8(dst, currentControlRateProfile->throttle_limit_type);
			// 限制比例
			sbufWriteU8(dst, currentControlRateProfile->throttle_limit_percent);
			// 油门中点
			sbufWriteU8(dst, currentControlRateProfile->thrMid8);
			// 油门EXPO
			sbufWriteU8(dst, currentControlRateProfile->thrExpo8);
			// TPA模式
			sbufWriteU8(dst, currentControlRateProfile->tpaMode);
			// TPA起点
			sbufWriteU16(dst, currentControlRateProfile->tpa_breakpoint);
			// TPA因素
			float tpaFactor = getThrottlePIDAttenuation();
			sbufWriteU16(dst, (uint16_t)(tpaFactor*100));
		break;
		case MSP_FILTER_CONFIG:
			// ---------------FILTER配置
			// 陀螺仪低通滤波器1[静态]截止频率
			sbufWriteU8(dst, gyroConfig()->gyro_lowpass_hz);
			// 动态最低截止频率[Hz]
        	sbufWriteU16(dst, gyroConfig()->dyn_lpf_gyro_min_hz);
			// 动态最高截止频率[Hz]
			sbufWriteU16(dst, gyroConfig()->dyn_lpf_gyro_max_hz);
			// 陀螺仪低通滤波器2[静态]截止频率
			sbufWriteU16(dst, gyroConfig()->gyro_lowpass2_hz);
			// D_Term低通滤波器1[静态]截止频率
			sbufWriteU16(dst, currentPidProfile->dterm_lowpass_hz);
			// 动态最低截止频率[Hz]
        	sbufWriteU16(dst, currentPidProfile->dyn_lpf_dterm_min_hz);
			// 动态最高截止频率[Hz]
			sbufWriteU16(dst, currentPidProfile->dyn_lpf_dterm_max_hz);
			// D_Term低通滤波器2[静态]截止频率
			sbufWriteU16(dst, currentPidProfile->dterm_lowpass2_hz);
		break;
		case MSP_RC1:
			// ---------------RC数据
	        for (int i = 0; i < 8; i++) {
	            sbufWriteU16(dst, rcData[i]);
	        }
		break;
		case MSP_RC2:
			// ---------------RC数据
	        for (int i = 8; i < 16; i++) {
	            sbufWriteU16(dst, rcData[i]);
	        }
			// RSSI
			sbufWriteU8(dst, getRssiPercent());
			// LQ
			sbufWriteU16(dst, rxGetLinkQualityPercent());
		break;	
		case MSP_RX_CONFIG:
			// ---------------RX配置
			// 摇杆低位阈值
			sbufWriteU16(dst, rxConfig()->mincheck);
			// 摇杆中位
			sbufWriteU16(dst, rxConfig()->midrc);
			// 摇杆高位阈值
			sbufWriteU16(dst, rxConfig()->maxcheck);
			// RC死区区间
        	sbufWriteU8(dst, rcControlsConfig()->deadband);
			// YAW死区区间
			sbufWriteU8(dst, rcControlsConfig()->yaw_deadband);
			// RC插值类型
			sbufWriteU8(dst, rxConfig()->rcInterpolation);
			// RC插值通道
			sbufWriteU8(dst, rxConfig()->rcInterpolationChannels);
		break;	
		case MSP_MODE_RANGES: {
            const modeActivationCondition_t *mac = modeActivationConditions(0);
            sbufWriteU8(dst, mac->auxChannelIndex);
            sbufWriteU8(dst, mac->range.startStep);
            sbufWriteU8(dst, mac->range.endStep);
            const modeActivationCondition_t *mac1 = modeActivationConditions(1);
            sbufWriteU8(dst, mac1->auxChannelIndex);
            sbufWriteU8(dst, mac1->range.startStep);
            sbufWriteU8(dst, mac1->range.endStep);
			const modeActivationCondition_t *mac2 = modeActivationConditions(12);
            sbufWriteU8(dst, mac2->auxChannelIndex);
            sbufWriteU8(dst, mac2->range.startStep);
            sbufWriteU8(dst, mac2->range.endStep);
			const modeActivationCondition_t *mac3 = modeActivationConditions(6);
            sbufWriteU8(dst, mac3->auxChannelIndex);
            sbufWriteU8(dst, mac3->range.startStep);
            sbufWriteU8(dst, mac3->range.endStep);
			const modeActivationCondition_t *mac4 = modeActivationConditions(8);
            sbufWriteU8(dst, mac4->auxChannelIndex);
            sbufWriteU8(dst, mac4->range.startStep);
            sbufWriteU8(dst, mac4->range.endStep);
			const modeActivationCondition_t *mac5 = modeActivationConditions(9);
            sbufWriteU8(dst, mac5->auxChannelIndex);
            sbufWriteU8(dst, mac5->range.startStep);
            sbufWriteU8(dst, mac5->range.endStep);
			const modeActivationCondition_t *mac6 = modeActivationConditions(14);
            sbufWriteU8(dst, mac6->auxChannelIndex);
            sbufWriteU8(dst, mac6->range.startStep);
            sbufWriteU8(dst, mac6->range.endStep);
			const modeActivationCondition_t *mac7 = modeActivationConditions(16);
            sbufWriteU8(dst, mac7->auxChannelIndex);
            sbufWriteU8(dst, mac7->range.startStep);
            sbufWriteU8(dst, mac7->range.endStep); 
		} break;
		case MSP_LINK:
			// ----------------------------------MSP链接
		break;
		case MSP_MOTOR_INFO:
			// ----------------------------------电机辅助数据
			// 陀螺仪滤波数据
            for (int i = 0; i < 3; i++) {
                sbufWriteU16(dst, gyroRateDps(i));
            }
			// 电池电压
			sbufWriteU16(dst, getBatteryVoltage());
			// 电池电流
			sbufWriteU16(dst, (int16_t)constrain(getAmperage(), -0x8000, 0x7FFF)); 
		break;
		case MSP_OSD_CONFIG: {
			// ----------------------------------OSD配置
			// OSD芯片状态
#define OSD_FLAGS_OSD_FEATURE           (1 << 0)
#define OSD_FLAGS_OSD_HARDWARE_MAX_7456 (1 << 1)
#define OSD_FLAGS_OSD_DEVICE_DETECTED   (1 << 2)
	        uint8_t osdFlags = 0;
#if defined(USE_OSD)
	        osdFlags |= OSD_FLAGS_OSD_FEATURE;
	        osdDisplayPortDevice_e deviceType;
	        displayPort_t *osdDisplayPort = osdGetDisplayPort(&deviceType);
	        switch (deviceType) {
		        case OSD_DISPLAYPORT_DEVICE_MAX7456:
		            osdFlags |= OSD_FLAGS_OSD_HARDWARE_MAX_7456;
		            if (osdDisplayPort && displayIsReady(osdDisplayPort)) {
		                osdFlags |= OSD_FLAGS_OSD_DEVICE_DETECTED;
		            }
	            break;
		        default:
		        break;
	        }
#endif
	        sbufWriteU8(dst, osdFlags);
			// 警告RSSI
			sbufWriteU8(dst, osdConfig()->rssi_alarm);
			// 警告LQ
			sbufWriteU16(dst, osdConfig()->link_quality_alarm);
			// 警告高度
			sbufWriteU16(dst, osdConfig()->alt_alarm);
		} break;
		case MSP_CURVE_GYRO:
			// ----------------------------------陀螺仪曲线数据
			// 陀螺仪滤波数据
            for (int i = 0; i < 3; i++) {
                sbufWriteU16(dst, gyroRateDps(i));
            }
		break;
		case MSP_CURVE_ACC: {
			// ----------------------------------加速度计曲线数据
            uint8_t scale;
            if (acc.dev.acc_1G > 512 * 4) {
                scale = 8;
            } else if (acc.dev.acc_1G > 512 * 2) {
                scale = 4;
            } else if (acc.dev.acc_1G >= 512) {
                scale = 2;
            } else {
                scale = 1;
            }
			// 加速度计滤波数据
            for (int i = 0; i < 3; i++) {
                sbufWriteU16(dst, lrintf(acc.accADC[i] / scale));
            }
		} break;
		case MSP_CURVE_MAG:
			// ----------------------------------磁力计曲线数据
			// 磁力计滤波数据
            for (int i = 0; i < 3; i++) {
                sbufWriteU16(dst, lrintf(mag.magADC[i]));
            }
		break;
		case MSP_CURVE_BARO:
			// ----------------------------------气压计曲线数据
			// 高度数据
			sbufWriteU32(dst, getEstimatedAltitudeCm());
		break;
		case MSP_VTX_CONFIG: {
			// ----------------------------------图传配置
            const vtxDevice_t *vtxDevice = vtxCommonDevice();
            unsigned vtxStatus = 0;
            uint8_t deviceIsReady = 0;
            if (vtxDevice) {
                vtxCommonGetStatus(vtxDevice, &vtxStatus);
                deviceIsReady = vtxCommonDeviceIsReady(vtxDevice) ? 1 : 0;
            }
			sbufWriteU8(dst, deviceIsReady);
			sbufWriteU8(dst, vtxSettingsConfig()->band);
            sbufWriteU8(dst, vtxSettingsConfig()->channel);
            sbufWriteU8(dst, vtxSettingsConfig()->power);
            sbufWriteU8(dst, (vtxStatus & VTX_STATUS_PIT_MODE) ? 1 : 0);
		} break;
	    default:
	        return false;
	}
	return true;
}

/**********************************************************************
函数名称：mspCommonProcessInCommand
函数功能：MSP命令响应（获取上位机数据）
函数形参：MSP命令，命令回复缓存，后置处理进程
函数返回值：MSP应答结果
函数描述：None
**********************************************************************/
mspResult_e mspProcessInCommand(int16_t cmdMSP, sbuf_t *src, mspPostProcessFnPtr *mspPostProcessFn)
{
	// 获取数据流缓冲区剩余字节
	const unsigned int dataSize = sbufBytesRemaining(src);
	// 初始化应答状态为无需应答
	int ret = MSP_RESULT_NO_REPLY;
    switch (cmdMSP) {
		case MSP_ACC_CALIBRATION:
			// ----------------------------------加速度计校准
			if (!ARMING_FLAG(ARMED))
            	accStartCalibration();
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		break;	
		case MSP_MAG_CALIBRATION:
			// ----------------------------------磁力计校准
			if (!ARMING_FLAG(ARMED))
            	compassStartCalibration();
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		break;	
		case MSP_SET_SYSTEM_CONFIG1:
			// ----------------------------------系统配置
			// ---------------PID循环更新频率
			pidConfigMutable()->pid_process_denom = sbufReadU8(src);
			// ---------------最大解锁角度
			currentPidProfile->levelAngleLimit = sbufReadU8(src);
			// ---------------磁力计启用状态
			if(sbufReadU8(src)) {
				compassConfigMutable()->mag_hardware = MAG_DEFAULT;
			} else {
				compassConfigMutable()->mag_hardware = MAG_NONE;
			}
			// ---------------气压计启用状态
			if(sbufReadU8(src)) {
				barometerConfigMutable()->baro_hardware = BARO_DEFAULT;
			} else {
				barometerConfigMutable()->baro_hardware = BARO_NONE;
			}
			// ---------------陀螺仪/加速度计板对齐
			gyroDeviceConfigMutable(0)->alignment = sbufReadU8(src);
			// ---------------磁力计板对齐
			compassConfigMutable()->mag_alignment = sbufReadU8(src);
			// ---------------蜂鸣器开启状态
			beeperConfigMutable()->beeper_off_flags = sbufReadU16(src);
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		break;	
		case MSP_SET_SYSTEM_CONFIG2:
			// ---------------电机怠速数值
			motorConfigMutable()->digitalIdleOffsetValue = sbufReadU16(src);
			// ---------------信标音调
			beeperConfigMutable()->dshotBeaconTone = sbufReadU8(src);
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		break;	
		case MSP_SET_SYSTEM_CONFIG3: {
			// ---------------通过电机发出蜂鸣音&信号丢失鸣叫
			if(sbufReadU8(src) == 1) {
				beeperConfigMutable()->dshotBeaconOffFlags &= ~(1 << 9);
			} else {
				beeperConfigMutable()->dshotBeaconOffFlags |= 1 << 9;
			}
			if(sbufReadU8(src) == 1) {
				beeperConfigMutable()->dshotBeaconOffFlags &= ~(1 << 1);
			} else {
				beeperConfigMutable()->dshotBeaconOffFlags |= 1 << 1;
			}
			// ---------------高度源类型
			positionConfigMutable()->altSource = sbufReadU8(src);
			// ---------------保存配置
			// 写入配置
		    writeEEPROM();
			// 读取配置 - 验证和修复配置
		    readEEPROM();
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		} break;	
		case MSP_SYSTEM_REBOOT:
			// ----------------------------------系统重启
			// 注册后置处理进程
            *mspPostProcessFn = mspRebootFn;
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		break;	
		case MSP_SET_BATTERY_CONFIG:
			// ----------------------------------电池配置
			batteryConfigMutable()->vbatmincellvoltage = sbufReadU8(src)*10;
			batteryConfigMutable()->vbatmaxcellvoltage = sbufReadU8(src)*10;
			batteryConfigMutable()->vbatwarningcellvoltage = sbufReadU8(src)*10;
			voltageSensorADCConfigMutable(0)->vbatscale = sbufReadU8(src);
			voltageSensorADCConfigMutable(0)->vbatresdivval = sbufReadU8(src);
			voltageSensorADCConfigMutable(0)->vbatresdivmultiplier = sbufReadU8(src);
			currentSensorADCConfigMutable()->scale = sbufReadU16(src);
			currentSensorADCConfigMutable()->offset = sbufReadU16(src);
			// ---------------保存配置
			// 写入配置
		    writeEEPROM();
			// 读取配置 - 验证和修复配置
		    readEEPROM();
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		break;	
		case MSP_SET_FAILSAFE_CONFIG:
			// ----------------------------------失控保护配置
			failsafeConfigMutable()->failsafe_switch_mode = sbufReadU8(src);
			failsafeConfigMutable()->failsafe_delay = sbufReadU8(src);
			failsafeConfigMutable()->failsafe_throttle_low_delay = sbufReadU16(src);
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		break;	
		case MSP_SET_RXFAIL_CONFIG:
			// ----------------------------------一阶失控保护配置
			// RPYT 1-12 -------mode，step
	        for (int i = 0; i < 16; i++) {
				rxFailsafeChannelConfigsMutable(i)->mode = sbufReadU8(src);
				rxFailsafeChannelConfigsMutable(i)->step = CHANNEL_VALUE_TO_RXFAIL_STEP(sbufReadU16(src));
	        }
			// ---------------保存配置
			// 写入配置
		    writeEEPROM();
			// 读取配置 - 验证和修复配置
		    readEEPROM();
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		break;	
		case MSP_SET_GPS_RESCUE_CONFIG1:
			// ----------------------------------二阶失控保护配置
			rxConfigMutable()->rx_min_usec = sbufReadU16(src);
			rxConfigMutable()->rx_max_usec = sbufReadU16(src);
			failsafeConfigMutable()->failsafe_procedure = sbufReadU8(src);
			gpsRescueConfigMutable()->allowArmingWithoutFix = sbufReadU16(src);
			gpsConfigMutable()->gps_set_home_point_once = sbufReadU8(src);
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		break;	
		case MSP_SET_GPS_RESCUE_CONFIG2:
			// ----------------------------------二阶失控保护配置
			gpsRescueConfigMutable()->angle = sbufReadU16(src);
			gpsRescueConfigMutable()->rescueGroundspeed = sbufReadU16(src);
			gpsRescueConfigMutable()->initialAltitudeM = sbufReadU16(src);
			gpsRescueConfigMutable()->descentDistanceM = sbufReadU16(src);
			gpsRescueConfigMutable()->throttleMin = sbufReadU16(src);
			gpsRescueConfigMutable()->throttleMax = sbufReadU16(src);
			gpsRescueConfigMutable()->ascendRate = sbufReadU16(src);
			gpsRescueConfigMutable()->descendRate = sbufReadU16(src);
			gpsRescueConfigMutable()->minSats = sbufReadU8(src);
			gpsRescueConfigMutable()->throttleHover = sbufReadU16(src);
			gpsRescueConfigMutable()->minRescueDth = sbufReadU16(src);
			gpsRescueConfigMutable()->altitudeMode = sbufReadU8(src);
			// ---------------保存配置
			// 写入配置
		    writeEEPROM();
			// 读取配置 - 验证和修复配置
		    readEEPROM();
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		break;	
		case MSP_SET_PID_RATE_PROFILE: {
			// ----------------------------------PID&&RATE配置文件配置
			uint8_t PidProfile = sbufReadU8(src);
			uint8_t RateProfile = sbufReadU8(src);
			if(PidProfile != getCurrentPidProfileIndex()) {
				changePidProfile(PidProfile);
			}
			if(RateProfile != getCurrentControlRateProfileIndex()) {
				changeControlRateProfile(RateProfile);
			}
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		} break;
		case MSP_SET_PID_CONFIG:
			// ----------------------------------PID配置
	        for (int i = 0; i < PID_LEVEL; i++) {
				currentPidProfile->pid[i].P = sbufReadU8(src);
				currentPidProfile->pid[i].I = sbufReadU8(src);
				currentPidProfile->pid[i].D = sbufReadU8(src); 
				currentPidProfile->d_min[i] = sbufReadU8(src);
				currentPidProfile->pid[i].F = sbufReadU16(src);
	        }
			currentPidProfile->pid[PID_LEVEL].P = sbufReadU8(src);
			currentPidProfile->levelAngleLimit = sbufReadU8(src);
			currentPidProfile->throttle_boost = sbufReadU8(src);
			currentPidProfile->iterm_relax = sbufReadU8(src);
			currentPidProfile->iterm_relax_type = sbufReadU8(src);
			currentPidProfile->d_min_gain = sbufReadU8(src);
			currentPidProfile->d_min_advance = sbufReadU8(src);
			if(sbufReadU8(src)) {
				featureEnableImmediate(FEATURE_ANTI_GRAVITY);
			} else {
				featureDisableImmediate(FEATURE_ANTI_GRAVITY);
			}
			currentPidProfile->antiGravityMode = sbufReadU8(src);
			// ---------------保存配置
			// 写入配置
		    writeEEPROM();
			// 读取配置 - 验证和修复配置
		    readEEPROM();
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		break;	
		case MSP_SET_RATE_CONFIG:
			// ----------------------------------RATE配置
	        for (int i = 0 ; i < 3; i++) {
				currentControlRateProfile->rcRates[i] = sbufReadU8(src);
				currentControlRateProfile->rates[i] = sbufReadU8(src);
				currentControlRateProfile->rcExpo[i] = sbufReadU8(src);
        	}
			// ---------------保存配置
			// 写入配置
		    writeEEPROM();
			// 读取配置 - 验证和修复配置
		    readEEPROM();
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		break;	
		case MSP_SET_THR_CONFIG:
			// ----------------------------------THR配置
			currentControlRateProfile->throttle_limit_type = sbufReadU8(src);
			currentControlRateProfile->throttle_limit_percent = sbufReadU8(src);
			currentControlRateProfile->thrMid8 = sbufReadU8(src);
			currentControlRateProfile->thrExpo8 = sbufReadU8(src);
			currentControlRateProfile->tpaMode = sbufReadU8(src);
			currentControlRateProfile->tpa_breakpoint = sbufReadU16(src);
			// ---------------保存配置
			// 写入配置
		    writeEEPROM();
			// 读取配置 - 验证和修复配置
		    readEEPROM();
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		break;	
		case MSP_SET_FILTER_CONFIG:
			// ----------------------------------FILTER配置
			gyroConfigMutable()->gyro_lowpass_hz = sbufReadU8(src);
			gyroConfigMutable()->dyn_lpf_gyro_min_hz = sbufReadU16(src);
			gyroConfigMutable()->dyn_lpf_gyro_max_hz = sbufReadU16(src);
			gyroConfigMutable()->gyro_lowpass2_hz = sbufReadU16(src);
			currentPidProfile->dterm_lowpass_hz = sbufReadU16(src);
			currentPidProfile->dyn_lpf_dterm_min_hz = sbufReadU16(src);
			currentPidProfile->dyn_lpf_dterm_max_hz = sbufReadU16(src);
			currentPidProfile->dterm_lowpass2_hz = sbufReadU16(src);
			// ---------------保存配置
			// 写入配置
		    writeEEPROM();
			// 读取配置 - 验证和修复配置
		    readEEPROM();
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		break;	
		case MSP_COPY_PID_PROFILE: {
			// ----------------------------------复制PID配置文件
			uint8_t srcPidProfile = sbufReadU8(src);
			uint8_t dstPidProfile = sbufReadU8(src);
			pidCopyProfile(dstPidProfile, srcPidProfile);
		
		// 无需应答
		ret = MSP_RESULT_NO_REPLY;
		} break;
		case MSP_COPY_RATE_PROFILE: {
			// ----------------------------------复制RATE配置文件
			uint8_t srcRateProfile = sbufReadU8(src);
			uint8_t dstRateProfile = sbufReadU8(src);
			copyControlRateProfile(dstRateProfile, srcRateProfile);
		
		// 无需应答
		ret = MSP_RESULT_NO_REPLY;
		} break;	
		case MSP_SET_RX_CONFIG:
			// ----------------------------------RX配置
			rxConfigMutable()->mincheck = sbufReadU16(src);
			rxConfigMutable()->midrc = sbufReadU16(src);
			rxConfigMutable()->maxcheck = sbufReadU16(src);
			rcControlsConfigMutable()->deadband = sbufReadU8(src);
			rcControlsConfigMutable()->yaw_deadband = sbufReadU8(src);
			rxConfigMutable()->rcInterpolation = sbufReadU8(src);
			rxConfigMutable()->rcInterpolationChannels = sbufReadU8(src);
			// ---------------保存配置
			// 写入配置
		    writeEEPROM();
			// 读取配置 - 验证和修复配置
		    readEEPROM();
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		break;	
		case MSP_SET_MODE_RANGES: {
			// ----------------------------------模式范围配置
            modeActivationCondition_t *mac = modeActivationConditionsMutable(0);
			mac->auxChannelIndex = sbufReadU8(src);
			mac->range.startStep = sbufReadU8(src);
			mac->range.endStep = sbufReadU8(src);
            modeActivationCondition_t *mac1 = modeActivationConditionsMutable(1);
			mac1->auxChannelIndex = sbufReadU8(src);
			mac1->range.startStep = sbufReadU8(src);
			mac1->range.endStep = sbufReadU8(src);
			modeActivationCondition_t *mac2 = modeActivationConditionsMutable(12);
			mac2->auxChannelIndex = sbufReadU8(src);
			mac2->range.startStep = sbufReadU8(src);
			mac2->range.endStep = sbufReadU8(src);
			modeActivationCondition_t *mac3 = modeActivationConditionsMutable(6);
			mac3->auxChannelIndex = sbufReadU8(src);
			mac3->range.startStep = sbufReadU8(src);
			mac3->range.endStep = sbufReadU8(src);
			modeActivationCondition_t *mac4 = modeActivationConditionsMutable(8);
			mac4->auxChannelIndex = sbufReadU8(src);
			mac4->range.startStep = sbufReadU8(src);
			mac4->range.endStep = sbufReadU8(src);
			modeActivationCondition_t *mac5 = modeActivationConditionsMutable(9);
			mac5->auxChannelIndex = sbufReadU8(src);
			mac5->range.startStep = sbufReadU8(src);
			mac5->range.endStep = sbufReadU8(src);
			modeActivationCondition_t *mac6 = modeActivationConditionsMutable(14);
			mac6->auxChannelIndex = sbufReadU8(src);
			mac6->range.startStep = sbufReadU8(src);
			mac6->range.endStep = sbufReadU8(src);
			modeActivationCondition_t *mac7 = modeActivationConditionsMutable(16);
			mac7->auxChannelIndex = sbufReadU8(src);
			mac7->range.startStep = sbufReadU8(src);
			mac7->range.endStep = sbufReadU8(src);
			// ---------------保存配置
			// 写入配置
		    writeEEPROM();
			// 读取配置 - 验证和修复配置
		    readEEPROM();
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		} break;	
		case MSP_LINK_STATUS:
			// ----------------------------------MSP链接
			if(sbufReadU8(src)) {
				unsetArmingDisabled(ARMING_DISABLED_MSP);
			} else {
				setArmingDisabled(ARMING_DISABLED_MSP);
			}
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		break;
		case MSP_MOTOR_CONTROL_START:
			// ----------------------------------电机控制开始
			unsetArmingDisabled(ARMING_DISABLED_MSP);
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		break;
		case MSP_MOTOR_CONTROL:
			// ----------------------------------电机控制进行
	        for (int i = 0; i < 4; i++) {
				// 直接更改锁定状态下的电机锁定数据[1000,2000]
	            motor_disarmed[i] = dshotPwmDevice.vTable.convertExternalToMotor(sbufReadU16(src));
	        }
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		break;
		case MSP_MOTOR_CONTROL_END:
			// ----------------------------------电机控制结束
			mixerResetDisarmedMotors();
			setArmingDisabled(ARMING_DISABLED_MSP);
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		break;	
		case MSP_SET_OSD_CONFIG:
			// ----------------------------------OSD配置
			osdConfigMutable()->rssi_alarm = sbufReadU8(src);
			osdConfigMutable()->link_quality_alarm = sbufReadU16(src);
			osdConfigMutable()->alt_alarm = sbufReadU16(src);
			// ---------------保存配置
			// 写入配置
		    writeEEPROM();
			// 读取配置 - 验证和修复配置
		    readEEPROM();
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		break;
		case MSP_OSD_CHARACTER: {
			// ----------------------------------OSD字库烧录
			// OSD字符数据缓冲区
            osdCharacter_t chr;
			// OSD字符地址
            uint16_t addr;
			// -----------------获取OSD字符地址
			addr = sbufReadU8(src);
			// -----------------读取OSD字符数据到缓冲区（64字节）
            for (unsigned ii = 0; ii < 64; ii++) {
                chr.data[ii] = sbufReadU8(src);
            }
			// -----------------检查OSD设备是否存在
            displayPort_t *osdDisplayPort = osdGetDisplayPort(NULL);
            if (!osdDisplayPort) {
                return MSP_RESULT_ERROR;
            }
			// -----------------写入OSD字符数据
            if (!displayWriteFontCharacter(osdDisplayPort, addr, &chr)) {
                return MSP_RESULT_ERROR;
            }
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		} break;
		case MSP_SET_VTX_CONFIG: {
			// ----------------------------------图传配置
			vtxDevice_t *vtxDevice = vtxCommonDevice();
			// 设置频段
			const uint8_t newBand = sbufReadU8(src);
			vtxSettingsConfigMutable()->band = newBand;
			// 设置通道
			const uint8_t newChannel = sbufReadU8(src);
			vtxSettingsConfigMutable()->channel = newChannel;
			vtxCommonSetBandAndChannel(vtxDevice, newBand, newChannel);
			// -----------------设置频率
			vtxSettingsConfigMutable()->freq = vtxCommonLookupFrequency(vtxDevice, newBand, newChannel);
			// -----------------设置功率
			const uint8_t newpower = sbufReadU8(src);
			vtxSettingsConfigMutable()->power = newpower;
			vtxCommonSetPowerByIndex(vtxDevice, newpower);
			// -----------------设置PIT模式
			const uint8_t newPitmode = sbufReadU8(src);
			unsigned vtxCurrentStatus;
            vtxCommonGetStatus(vtxDevice, &vtxCurrentStatus);
	        if ((bool)(vtxCurrentStatus & VTX_STATUS_PIT_MODE) != (bool)newPitmode) {
	            vtxCommonSetPitMode(vtxDevice, newPitmode);
	        }
			// ---------------保存配置
			// 写入配置
		    writeEEPROM();
			// 读取配置 - 验证和修复配置
		    readEEPROM();
			// 无需应答
			ret = MSP_RESULT_NO_REPLY;
		} break;
    }
	return ret;
}

