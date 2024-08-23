/*
 * wdog.c
 *
 *  Created on: 2020年5月18日
 *      Author: Zcc
 */

#include "wdog.h"

IfxScuWdt_Config config;

// 只用savety watchdog
void wdog_init_ms(uint16 ms)
{
	boolean interrupt_state = disableInterrupts();

	Ifx_SCU_WDTS *watchdog = &MODULE_SCU.WDTS;
	uint16 num = ms * 3;

	IfxScuWdt_initConfig(&config); //	0xFC	1ms
	config.password = IfxScuWdt_getSafetyWatchdogPassword();
	config.reload = 0xFFFF - num;
	IfxScuWdt_initSafetyWatchdog(watchdog, &config);

	restoreInterrupts(interrupt_state);
}

// 喂狗
void wdog_feed(void)
{
	uint16 password = IfxScuWdt_getSafetyWatchdogPassword();
	IfxScuWdt_changeSafetyWatchdogReload(password, config.reload);
}

// 禁用狗
void wdog_disable(void)
{
	IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());
}

void wdog_enable(void)
{
	IfxScuWdt_enableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());
}
