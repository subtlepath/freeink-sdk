#pragma once

// FreeInk simulator — soc_caps shim.
//
// These are the capability macros the SDK gates on. They describe the MCU the
// bundle was built for, so the simulated build takes the same branches as the
// real one: the bundle's -DFREEINK_DEVICE_* choice already fixed the family,
// and the build script defines FSIM_SOC_* to match it.

#ifndef FSIM_SOC_SPI_PERIPH_NUM
#define FSIM_SOC_SPI_PERIPH_NUM 2
#endif
#ifndef FSIM_SOC_GPIO_PIN_COUNT
#define FSIM_SOC_GPIO_PIN_COUNT 48
#endif

#define SOC_SPI_PERIPH_NUM FSIM_SOC_SPI_PERIPH_NUM
#define SOC_GPIO_PIN_COUNT FSIM_SOC_GPIO_PIN_COUNT
#define SOC_RTCIO_PIN_COUNT FSIM_SOC_GPIO_PIN_COUNT
#define SOC_ADC_PERIPH_NUM 2
#define SOC_I2C_NUM 1
#define SOC_TOUCH_SENSOR_NUM 0
#define SOC_PM_SUPPORT_EXT0_WAKEUP 1
#define SOC_PM_SUPPORT_EXT1_WAKEUP 1
#define SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP 1
