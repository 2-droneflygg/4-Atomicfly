#pragma once

#define BEEP_TOGGLE              systemBeepToggle()		// 蜂鸣器切换
#define BEEP_OFF                 systemBeep(false)		// 蜂鸣器关闭
#define BEEP_ON                  systemBeep(true)		// 蜂鸣器开启

void systemBeep(bool on);
void systemBeepToggle(void);
struct beeperDevConfig_s;
void beeperInit(const struct beeperDevConfig_s *beeperDevConfig);

