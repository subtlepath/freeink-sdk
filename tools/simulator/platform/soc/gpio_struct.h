#pragma once

// FreeInk simulator — soc/gpio_struct shim.
//
// Direct register banging has no meaning against a modelled GPIO matrix, so the
// register file here is plain memory: writes are accepted and readable back,
// but they do NOT move virtual pins. Code that needs pins to move must use
// gpio_set_level()/digitalWrite(). The daemon logs a one-time warning on the
// `sim.warn` channel the first time a bundle touches this, so a driver that
// silently depends on register access is visible rather than mysterious.

#include <cstdint>

typedef struct {
  volatile uint32_t out;
  volatile uint32_t out_w1ts;
  volatile uint32_t out_w1tc;
  volatile uint32_t out1;
  volatile uint32_t out1_w1ts;
  volatile uint32_t out1_w1tc;
  volatile uint32_t enable;
  volatile uint32_t enable_w1ts;
  volatile uint32_t enable_w1tc;
  volatile uint32_t in;
  volatile uint32_t in1;
  volatile uint32_t status;
  volatile uint32_t status_w1ts;
  volatile uint32_t status_w1tc;
} fsim_gpio_dev_t;

typedef fsim_gpio_dev_t gpio_dev_t;

extern "C" gpio_dev_t GPIO;
