/*********************************************************************************
 提供图传表配置及图传表初始化API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>

#include "platform.h"

#include "drivers/vtx_table.h"

int          vtxTableBandCount = VTX_TABLE_MAX_BANDS;							// 频组数量
int          vtxTableChannelCount = VTX_TABLE_MAX_CHANNELS;						// 通道数量
uint16_t     vtxTableFrequency[VTX_TABLE_MAX_BANDS][VTX_TABLE_MAX_CHANNELS] = {	// 频率
    { 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725 }, 						// Boscam A
    { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866 }, 						// Boscam B
    { 5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945 }, 						// Boscam E
    { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880 }, 						// FatShark
    { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917 }, 						// RaceBand
};
const char * vtxTableBandNames[VTX_TABLE_MAX_BANDS + 1] = {						// 频组名称
        "--------",
        "BOSCAM A",
        "BOSCAM B",
        "BOSCAM E",
        "FATSHARK",
        "RACEBAND",
};
char         vtxTableBandLetters[VTX_TABLE_MAX_BANDS + 1] = "-ABEFR";			// 频组字母
const char * vtxTableChannelNames[VTX_TABLE_MAX_CHANNELS + 1] = {				// 通道名称
        "-", "1", "2", "3", "4", "5", "6", "7", "8",
};
bool         vtxTableIsFactoryBand[VTX_TABLE_MAX_BANDS];						// 工厂频组
int          vtxTablePowerLevels;												// 功率等级数量
uint16_t     vtxTablePowerValues[VTX_TABLE_MAX_POWER_LEVELS];					// 功率值
const char * vtxTablePowerLabels[VTX_TABLE_MAX_POWER_LEVELS + 1];				// 功率标签	

/**********************************************************************
函数名称：vtxTableInit
函数功能：图传表 - 初始化
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void vtxTableInit(void)
{
	// 遍历所有频组 - 使能所有频组
    for (int band = 0; band < VTX_TABLE_MAX_BANDS; band++) {
        vtxTableIsFactoryBand[band] = true;
    }
	// 遍历所有功率等级 - 清空功率值和功率标签
    for (int powerIndex = 0; powerIndex < VTX_TABLE_MAX_POWER_LEVELS; powerIndex++) {
        vtxTablePowerValues[powerIndex] = 0;
        vtxTablePowerLabels[powerIndex] = NULL;
    }
	// 初始化功率等级数量
    vtxTablePowerLevels = VTX_TABLE_MAX_POWER_LEVELS;
	// 失能所有频组
    vtxTableSetFactoryBands(false);
}

/**********************************************************************
函数名称：vtxTableSetFactoryBands
函数功能：图传表 - 设置频组使能状态
函数形参：isFactory
函数返回值：None
函数描述：None
**********************************************************************/
void vtxTableSetFactoryBands(bool isFactory)
{
    for(int i = 0;i < VTX_TABLE_MAX_BANDS; i++) {
        vtxTableIsFactoryBand[i] = isFactory;
    }
}

