#ifndef __MAIN_H
#define __MAIN_H

#include "hwconfig.h"
#include "stm32f0xx_it.h"
#include <stdio.h>
#include "stm320518_eval.h"


#define SPI_MASTER
//#define SPI_SLAVE

void TimingDelay_Decrement(void);
void SysTick_Handler(void);

#endif /* __MAIN_H */
