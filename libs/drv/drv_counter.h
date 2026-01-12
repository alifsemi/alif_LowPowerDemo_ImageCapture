#include <stdint.h>

void refclk_cntr_init();
void refclk_cntr_update();
uint32_t refclk_cntr_val();
uint64_t refclk_cntr_val64();

void s32k_cntr_init();
uint32_t s32k_cntr_val();
uint64_t s32k_cntr_val64();

void delay_ms_s32k(uint32_t nticks);
void delay_us_s32k(uint32_t nticks);
void delay_us_refclk(uint32_t nticks);

void refclk_cntr_enable_cntbase(uint32_t cntbase);
void refclk_cntr_disable_cntbase(uint32_t cntbase);
void refclk_cntr_enable_cntbase_intr(uint32_t cntbase, uint64_t compare_val);
void refclk_cntr_disable_cntbase_intr(uint32_t cntbase);

void s32k_cntr_enable_cntbase(uint32_t cntbase);
void s32k_cntr_disable_cntbase(uint32_t cntbase);
void s32k_cntr_enable_cntbase_intr(uint32_t cntbase, uint32_t compare_val);
void s32k_cntr_disable_cntbase_intr(uint32_t cntbase);
