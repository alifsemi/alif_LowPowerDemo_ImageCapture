#include <stdint.h>
#include <stdbool.h>

void OSC_initialize();
void OSC_uninitialize();

void PLL_initialize(uint32_t xtal_freq);
void PLL_uninitialize();

void PLL_clkpll_start(uint32_t xtal_freq, bool faststart);
void PLL_clkpll_stop();