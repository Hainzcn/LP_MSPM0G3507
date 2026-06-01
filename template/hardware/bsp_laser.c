/**
 * @file    bsp_laser.c
 * @brief   激光使能 GPIO 实现，详见 bsp_laser.h。
 */

#include "bsp_laser.h"

#include "bsp_gpio.h"

#include "ti_msp_dl_config.h"

#include <stdio.h>

static bool s_laser_on;

static void laser_apply(bool on)
{
#if BSP_LASER_ACTIVE_LOW
    if (on) {
        DL_GPIO_clearPins(BSP_LASER_EN_PORT, BSP_LASER_EN_PIN);
    } else {
        DL_GPIO_setPins(BSP_LASER_EN_PORT, BSP_LASER_EN_PIN);
    }
#else
    if (on) {
        DL_GPIO_setPins(BSP_LASER_EN_PORT, BSP_LASER_EN_PIN);
    } else {
        DL_GPIO_clearPins(BSP_LASER_EN_PORT, BSP_LASER_EN_PIN);
    }
#endif
    s_laser_on = on;
    (void)printf("[laser] %s (PA1=%s)\r\n", on ? "ON" : "OFF",
        on ? (BSP_LASER_ACTIVE_LOW ? "L" : "H") : (BSP_LASER_ACTIVE_LOW ? "H" : "L"));
}

void bsp_laser_init(void)
{
    DL_GPIO_initDigitalOutput(BSP_LASER_EN_IOMUX);
    DL_GPIO_enableOutput(BSP_LASER_EN_PORT, BSP_LASER_EN_PIN);
    laser_apply(false);
}

void bsp_laser_set_enable(bool on)
{
    if (on == s_laser_on) {
        return;
    }
    laser_apply(on);
}

bool bsp_laser_is_enabled(void)
{
    return s_laser_on;
}
