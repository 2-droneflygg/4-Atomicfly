/*********************************************************************************
 提供特性操作相关API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"


PG_REGISTER_WITH_RESET_TEMPLATE(featureConfig_t, featureConfig, PG_FEATURE_CONFIG, 0);
// 启用特性 - 串行数字接收机，动态滤波，反重力，GPS，软串口
PG_RESET_TEMPLATE(featureConfig_t, featureConfig,
    .enabledFeatures = DEFAULT_FEATURES | DEFAULT_RX_FEATURE | FEATURE_DYNAMIC_FILTER | FEATURE_ANTI_GRAVITY | FEATURE_GPS | FEATURE_SOFTSERIAL ,
);

// -----------------------------------------------------------------------运行时特性掩码
static uint32_t runtimeFeatureMask;

/**********************************************************************
函数名称：featureInit
函数功能：特性初始化
函数形参：None
函数返回值：None
函数描述：
	readEEPROM时进行初始化。
**********************************************************************/
void featureInit(void)
{
    runtimeFeatureMask = featureConfig()->enabledFeatures;
}

/**********************************************************************
函数名称：featureSet
函数功能：设置特性
函数形参：掩码，feature
函数返回值：None
函数描述：None
**********************************************************************/
static void featureSet(const uint32_t mask, uint32_t *features)
{
    *features |= mask;
}

/**********************************************************************
函数名称：featureClear
函数功能：清除特性
函数形参：掩码，feature
函数返回值：None
函数描述：None
**********************************************************************/
static void featureClear(const uint32_t mask, uint32_t *features)
{
    *features &= ~(mask);
}

/**********************************************************************
函数名称：featureIsConfigured
函数功能：直接启用特性
函数形参：掩码
函数返回值：None
函数描述：None
**********************************************************************/
void featureEnableImmediate(const uint32_t mask)
{
    featureSet(mask, &featureConfigMutable()->enabledFeatures);
    featureSet(mask, &runtimeFeatureMask);
}

/**********************************************************************
函数名称：featureDisableImmediate
函数功能：直接禁用特性
函数形参：掩码
函数返回值：None
函数描述：None
**********************************************************************/
void featureDisableImmediate(const uint32_t mask)
{
    featureClear(mask, &featureConfigMutable()->enabledFeatures);
    featureClear(mask, &runtimeFeatureMask);
}

/**********************************************************************
函数名称：featureIsEnabled
函数功能：获取特性使能状态
函数形参：掩码
函数返回值：runtimeFeatureMask & mask
函数描述：None
**********************************************************************/
bool featureIsEnabled(const uint32_t mask)
{
    return runtimeFeatureMask & mask;
}

/**********************************************************************
函数名称：featureIsConfigured
函数功能：获取特性配置状态
函数形参：掩码
函数返回值：featureConfig()->enabledFeatures & mask
函数描述：
	确定特性是否被配置(在配置中设置)，并不意味着特性在运行时是激活的。
	这个函数应该只在配置检查中使用，在启动和写入EEPROM时执行。
**********************************************************************/
bool featureIsConfigured(const uint32_t mask)
{
    return featureConfig()->enabledFeatures & mask;
}

