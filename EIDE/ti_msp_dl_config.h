/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_MOTOR */
#define PWM_MOTOR_INST                                                     TIMA0
#define PWM_MOTOR_INST_IRQHandler                               TIMA0_IRQHandler
#define PWM_MOTOR_INST_INT_IRQN                                 (TIMA0_INT_IRQn)
#define PWM_MOTOR_INST_CLK_FREQ                                         32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_C0_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C0_PIN                                      DL_GPIO_PIN_8
#define GPIO_PWM_MOTOR_C0_IOMUX                                  (IOMUX_PINCM19)
#define GPIO_PWM_MOTOR_C0_IOMUX_FUNC                 IOMUX_PINCM19_PF_TIMA0_CCP0
#define GPIO_PWM_MOTOR_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_C1_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C1_PIN                                      DL_GPIO_PIN_9
#define GPIO_PWM_MOTOR_C1_IOMUX                                  (IOMUX_PINCM20)
#define GPIO_PWM_MOTOR_C1_IOMUX_FUNC                 IOMUX_PINCM20_PF_TIMA0_CCP1
#define GPIO_PWM_MOTOR_C1_IDX                                DL_TIMER_CC_1_INDEX




/* Defines for QEI_LEFT */
#define QEI_LEFT_INST                                                      TIMG8
#define QEI_LEFT_INST_IRQHandler                                TIMG8_IRQHandler
#define QEI_LEFT_INST_INT_IRQN                                  (TIMG8_INT_IRQn)
/* Pin configuration defines for QEI_LEFT PHA Pin */
#define GPIO_QEI_LEFT_PHA_PORT                                             GPIOA
#define GPIO_QEI_LEFT_PHA_PIN                                     DL_GPIO_PIN_29
#define GPIO_QEI_LEFT_PHA_IOMUX                                   (IOMUX_PINCM4)
#define GPIO_QEI_LEFT_PHA_IOMUX_FUNC                  IOMUX_PINCM4_PF_TIMG8_CCP0
/* Pin configuration defines for QEI_LEFT PHB Pin */
#define GPIO_QEI_LEFT_PHB_PORT                                             GPIOA
#define GPIO_QEI_LEFT_PHB_PIN                                     DL_GPIO_PIN_30
#define GPIO_QEI_LEFT_PHB_IOMUX                                   (IOMUX_PINCM5)
#define GPIO_QEI_LEFT_PHB_IOMUX_FUNC                  IOMUX_PINCM5_PF_TIMG8_CCP1
/* Pin configuration defines for QEI_LEFT IDX Pin */
#define GPIO_QEI_LEFT_IDX_PORT                                             GPIOB
#define GPIO_QEI_LEFT_IDX_PIN                                     DL_GPIO_PIN_14
#define GPIO_QEI_LEFT_IDX_IOMUX                                  (IOMUX_PINCM31)
#define GPIO_QEI_LEFT_IDX_IOMUX_FUNC                  IOMUX_PINCM31_PF_TIMG8_IDX


/* Defines for UART_IMU */
#define UART_IMU_INST                                                      UART3
#define UART_IMU_INST_FREQUENCY                                         32000000
#define UART_IMU_INST_IRQHandler                                UART3_IRQHandler
#define UART_IMU_INST_INT_IRQN                                    UART3_INT_IRQn
#define GPIO_UART_IMU_RX_PORT                                              GPIOB
#define GPIO_UART_IMU_TX_PORT                                              GPIOB
#define GPIO_UART_IMU_RX_PIN                                      DL_GPIO_PIN_13
#define GPIO_UART_IMU_TX_PIN                                      DL_GPIO_PIN_12
#define GPIO_UART_IMU_IOMUX_RX                                   (IOMUX_PINCM30)
#define GPIO_UART_IMU_IOMUX_TX                                   (IOMUX_PINCM29)
#define GPIO_UART_IMU_IOMUX_RX_FUNC                    IOMUX_PINCM30_PF_UART3_RX
#define GPIO_UART_IMU_IOMUX_TX_FUNC                    IOMUX_PINCM29_PF_UART3_TX
#define UART_IMU_BAUD_RATE                                              (115200)
#define UART_IMU_IBRD_32_MHZ_115200_BAUD                                    (17)
#define UART_IMU_FBRD_32_MHZ_115200_BAUD                                    (23)
/* Defines for UART_LOG */
#define UART_LOG_INST                                                      UART0
#define UART_LOG_INST_FREQUENCY                                         32000000
#define UART_LOG_INST_IRQHandler                                UART0_IRQHandler
#define UART_LOG_INST_INT_IRQN                                    UART0_INT_IRQn
#define GPIO_UART_LOG_RX_PORT                                              GPIOA
#define GPIO_UART_LOG_TX_PORT                                              GPIOA
#define GPIO_UART_LOG_RX_PIN                                      DL_GPIO_PIN_11
#define GPIO_UART_LOG_TX_PIN                                      DL_GPIO_PIN_10
#define GPIO_UART_LOG_IOMUX_RX                                   (IOMUX_PINCM22)
#define GPIO_UART_LOG_IOMUX_TX                                   (IOMUX_PINCM21)
#define GPIO_UART_LOG_IOMUX_RX_FUNC                    IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_LOG_IOMUX_TX_FUNC                    IOMUX_PINCM21_PF_UART0_TX
#define UART_LOG_BAUD_RATE                                              (115200)
#define UART_LOG_IBRD_32_MHZ_115200_BAUD                                    (17)
#define UART_LOG_FBRD_32_MHZ_115200_BAUD                                    (23)
/* Defines for UART_K230 */
#define UART_K230_INST                                                     UART1
#define UART_K230_INST_FREQUENCY                                        32000000
#define UART_K230_INST_IRQHandler                               UART1_IRQHandler
#define UART_K230_INST_INT_IRQN                                   UART1_INT_IRQn
#define GPIO_UART_K230_RX_PORT                                             GPIOB
#define GPIO_UART_K230_TX_PORT                                             GPIOB
#define GPIO_UART_K230_RX_PIN                                      DL_GPIO_PIN_7
#define GPIO_UART_K230_TX_PIN                                      DL_GPIO_PIN_6
#define GPIO_UART_K230_IOMUX_RX                                  (IOMUX_PINCM24)
#define GPIO_UART_K230_IOMUX_TX                                  (IOMUX_PINCM23)
#define GPIO_UART_K230_IOMUX_RX_FUNC                   IOMUX_PINCM24_PF_UART1_RX
#define GPIO_UART_K230_IOMUX_TX_FUNC                   IOMUX_PINCM23_PF_UART1_TX
#define UART_K230_BAUD_RATE                                             (921600)
#define UART_K230_IBRD_32_MHZ_921600_BAUD                                    (2)
#define UART_K230_FBRD_32_MHZ_921600_BAUD                                   (11)





/* Defines for ADC_BAT */
#define ADC_BAT_INST                                                        ADC0
#define ADC_BAT_INST_IRQHandler                                  ADC0_IRQHandler
#define ADC_BAT_INST_INT_IRQN                                    (ADC0_INT_IRQn)
#define ADC_BAT_ADCMEM_0                                      DL_ADC12_MEM_IDX_0
#define ADC_BAT_ADCMEM_0_REF                     DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC_BAT_ADCMEM_0_REF_VOLTAGE_V                                       3.3
#define GPIO_ADC_BAT_C5_PORT                                               GPIOB
#define GPIO_ADC_BAT_C5_PIN                                       DL_GPIO_PIN_24
#define GPIO_ADC_BAT_IOMUX_C5                                    (IOMUX_PINCM52)
#define GPIO_ADC_BAT_IOMUX_C5_FUNC                (IOMUX_PINCM52_PF_UNCONNECTED)



/* Defines for DMA_CH0 */
#define DMA_CH0_CHAN_ID                                                      (1)
#define UART_K230_INST_DMA_TRIGGER_0                         (DMA_UART1_RX_TRIG)
/* Defines for DMA_CH1 */
#define DMA_CH1_CHAN_ID                                                      (0)
#define UART_K230_INST_DMA_TRIGGER_1                         (DMA_UART1_TX_TRIG)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_MOTOR_init(void);
void SYSCFG_DL_QEI_LEFT_init(void);
void SYSCFG_DL_UART_IMU_init(void);
void SYSCFG_DL_UART_LOG_init(void);
void SYSCFG_DL_UART_K230_init(void);
void SYSCFG_DL_ADC_BAT_init(void);
void SYSCFG_DL_DMA_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
