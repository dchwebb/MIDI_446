#pragma once

#include "stm32f4xx.h"
#include <algorithm>


extern volatile uint32_t SysTickVal;

void SystemClock_Config(void);
void InitBtnLED(void);

void InitSysTick();
void InitLCDHardware(void);
void InitSampleAcquisition();
void InitCoverageTimer();
void InitDebounceTimer();
void InitEncoders();
void InitUART();
//void InitUSB();

