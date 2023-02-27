#include <stdint.h>

#include "platform.h"

#include "drivers/vtx_common.h"

#define VTX_TABLE_MAX_BANDS             5 			// 默认频率表有5个频组
#define VTX_TABLE_MAX_CHANNELS          8 			// 八个通道
#define VTX_TABLE_MAX_POWER_LEVELS      5 			// 五档功率

#define VTX_TABLE_MIN_USER_FREQ         5000		// 最小频率
#define VTX_TABLE_DEFAULT_BAND          1			// 默认频组
#define VTX_TABLE_DEFAULT_CHANNEL       6			// 默认通道
#define VTX_TABLE_DEFAULT_FREQ          5765		// 默认频率
#define VTX_TABLE_DEFAULT_PITMODE_FREQ  0			// 默认PIT模式状态
#define VTX_TABLE_DEFAULT_POWER         1 			// 基于索引。0表示未知，1为最低实际功率模式


extern int            vtxTableBandCount;
extern int            vtxTableChannelCount;
extern uint16_t       vtxTableFrequency[VTX_TABLE_MAX_BANDS][VTX_TABLE_MAX_CHANNELS];
extern const char    *vtxTableBandNames[VTX_TABLE_MAX_BANDS + 1];
extern char           vtxTableBandLetters[VTX_TABLE_MAX_BANDS + 1];
extern const char    *vtxTableChannelNames[VTX_TABLE_MAX_CHANNELS + 1];
extern bool           vtxTableIsFactoryBand[VTX_TABLE_MAX_BANDS];
extern int            vtxTablePowerLevels;
extern uint16_t       vtxTablePowerValues[VTX_TABLE_MAX_POWER_LEVELS];
extern const char    *vtxTablePowerLabels[VTX_TABLE_MAX_POWER_LEVELS + 1];
struct vtxTableConfig_s;
void vtxTableInit(void);
void vtxTableStrncpyWithPad(char *dst, const char *src, int length);
void vtxTableConfigClearBand(struct vtxTableConfig_s *config, int band);
void vtxTableConfigClearPowerValues(struct vtxTableConfig_s *config, int start);
void vtxTableConfigClearPowerLabels(struct vtxTableConfig_s *config, int start);
void vtxTableConfigClearChannels(struct vtxTableConfig_s *config, int band, int channels);
void vtxTableSetFactoryBands(bool isFactory);

