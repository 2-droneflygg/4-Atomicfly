#ifdef USE_DSHOT

extern FAST_RAM_ZERO_INIT uint8_t dmaMotorTimerCount;
extern motorDmaTimer_t dmaMotorTimers[MAX_DMA_TIMERS];
extern motorDmaOutput_t dmaMotors[MAX_SUPPORTED_MOTORS];
uint8_t getTimerIndex(TIM_TypeDef *timer);
motorDmaOutput_t *getMotorDmaOutput(uint8_t index);
void dshotEnableChannels(uint8_t motorCount);
#endif

