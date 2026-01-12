#include <stdint.h>

#define SYST_SRAM0 1
#define SYST_SRAM1 2

#define RETAIN_1024KB   (3U << 4)
#define RETAIN_1280KB   ((3U << 8) | RETAIN_1024KB)
#define RETAIN_1536KB   ((3U << 12)| RETAIN_1280KB)


void enable_dcdc_pfm(void);
void enable_dcdc_pwm(void);
void enable_syst_sram(uint32_t sram_select);

void enable_pd1_aon(uint32_t retention_select);
void disable_pd1_aon();
void enable_pd4_sram(uint32_t clk_sel);
void disable_pd4_sram();
