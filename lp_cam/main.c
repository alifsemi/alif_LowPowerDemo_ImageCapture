#include "stdio.h"
#include "string.h"
#include "inttypes.h"
#include "assert.h"

#include "RTE_Components.h"
#include "alif.h"

#include "app_utils.h"
#include "drv_counter.h"
#include "drv_pll.h"

#include "gpio.h"
#include "lptimer.h"

#include "soc_clk.h"
#include "soc_pwr.h"
#include "sys_ctrl_cpi.h"
#include "sys_ctrl_csi.h"
#include "sys_ctrl_dphy.h"

#if defined(RTE_CMSIS_Compiler_STDIN) || defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_config.h"
#endif

#include "Driver_CPI.h"
extern ARM_DRIVER_CPI Driver_CPI;
static void cpi_capture();
static void cpi_init();
static void cpi_cb(uint32_t);
static volatile uint32_t cpi_cb_count;
static volatile bool cpi_done;

/* Camera Controller Resolution. */
#if defined(RTE_Drivers_CAMERA_SENSOR_MT9M114)
#define CAM_FRAME_WIDTH        (RTE_MT9M114_CAMERA_SENSOR_MIPI_FRAME_WIDTH)
#define CAM_FRAME_HEIGHT       (RTE_MT9M114_CAMERA_SENSOR_MIPI_FRAME_HEIGHT)
#define CAM_COLOR_CORRECTION   (0)
#define CAM_USE_RGB565         (RTE_MT9M114_CAMERA_SENSOR_MIPI_IMAGE_CONFIG != 1)
#define RGB_BUFFER_SECTION     ".bss.camera_frame_bayer_to_rgb_buf_at_sram0"
#elif defined(RTE_Drivers_CAMERA_SENSOR_ARX3A0)
#define CAM_FRAME_WIDTH        (RTE_ARX3A0_CAMERA_SENSOR_FRAME_WIDTH)
#define CAM_FRAME_HEIGHT       (RTE_ARX3A0_CAMERA_SENSOR_FRAME_HEIGHT)
#define CAM_COLOR_CORRECTION   (1)
#define CAM_USE_RGB565         (0)
#define RGB_BUFFER_SECTION     ".bss.camera_frame_bayer_to_rgb_buf"
#endif

#define CAM_FRAME_SIZE (CAM_FRAME_WIDTH * CAM_FRAME_HEIGHT)
#define CAM_FRAME_SIZE_BYTES (CAM_FRAME_SIZE + CAM_USE_RGB565 * CAM_FRAME_SIZE)
static uint8_t buf_cam[CAM_FRAME_SIZE_BYTES] __attribute__((aligned(32), section(".bss.sram1.camera_frame_buff")));

#define TRIGGER_PIN         (PIN_4)
#define TRIGGER_PIN_MASK    (1U << TRIGGER_PIN)
static void lpgpio_init();
static volatile uint32_t lpgpio_cb_count;
static volatile bool cpi_trigger;

#define LPTIMER_CH          (0)
#define LPTIMER_CH_MASK     (1U << LPTIMER_CH)
static void lptimer_self_test();

static void run_on_reset();
static void run_on_cpi_trigger();
static void transition_down();
static void transition_up();

#if defined(ENSEMBLE_SOC_GEN2)
#define CLK_ENA_CLK100M                     (1U << 7)
#else
#define CLK_ENA_CLK100M                     (1U << 21)
#endif

static inline void enable_cgu_clk100m(void)
{
    CGU->CLK_ENA |= CLK_ENA_CLK100M;
}

static inline void disable_cgu_clk100m(void)
{
    CGU->CLK_ENA &= ~CLK_ENA_CLK100M;
}

#include "uart.h"
#define _UART_BASE_(n)      UART##n##_BASE
#define UART_BASE(n)        _UART_BASE_(n)
#define _UART_CLK_SRC_(n)   RTE_UART##n##_CLK_SOURCE
#define UART_CLK_SRC(n)     _UART_CLK_SRC_(n)
/* run this function every time the clock source for UART is changed */
static void reconfigure_uart()
{
#if defined(RTE_CMSIS_Compiler_STDIN_Custom) || defined(RTE_CMSIS_Compiler_STDOUT_Custom)
#if PRINTF_UART_CONSOLE == LP
    uart_set_baudrate((UART_Type*)LPUART_BASE, SystemCoreClock, PRINTF_UART_CONSOLE_BAUD_RATE);
#elif UART_CLK_SRC(PRINTF_UART_CONSOLE) == 0
    uart_set_baudrate((UART_Type*)UART_BASE(PRINTF_UART_CONSOLE), SocTopClockHFOSC(), PRINTF_UART_CONSOLE_BAUD_RATE);
#else
    uart_set_baudrate((UART_Type*)UART_BASE(PRINTF_UART_CONSOLE), SystBusClkUpdate(), PRINTF_UART_CONSOLE_BAUD_RATE);
#endif
#endif
}

int main (void)
{
    /* UART driver uses SystemCoreClock variable to calculate baud rate divider */
    SystemCoreClock = 76800000;
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    if (stdout_init() != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP;
    }
#endif

    printf("Low Power Demo: Image Capture with Trigger Pin\n");
    delay_ms_s32k(100);

    /* SETUP */
    run_on_reset();
    transition_down();
    lptimer_self_test();

    /* LOOP */
    while(1) {
        pm_core_enter_deep_sleep();
        if(cpi_trigger == true) {
            transition_up();
            run_on_cpi_trigger();
            transition_down();
        }
    }
}

/*
 *  functions to run only once at first start up
 * adjusting clock dividers, enabling xtal and pll, etc.
 */
static void run_on_reset()
{
    refclk_cntr_init();

    /* start the PLL */
    PLL_initialize(38400000);

    /* Configure RTSS clock options */
    CoreClockConfig(0, 0);

    /* Adjust top-level clock dividers */
    DivClockConfig(4, 7, 3);

    /* Switch the SYSPLL clock and RTSS-HE clock to PLL */
    PllClockConfig((1U << 20) | (1U << 4));

    /* Divide SYST_ACLK by 4 (divider is n + 1) */
#if defined(ENSEMBLE_SOC_GEN2) || defined(ENSEMBLE_SOC_E1C)
    BusClockConfig(2, 3, 2, 2); /* HCLK & PCLK are derived from SYSPLL */
#else
    BusClockConfig(2, 3, 0, 0); /* HCLK & PCLK are derived from ACLK */
#endif

    /* Parse the clock tree and update variables tracking the clock state */
    CoreClockUpdate();
    SystBusClkUpdate();
    refclk_cntr_update();
    reconfigure_uart();

    /* CAM_XVCLK: clock driven by MCU to CAM module */
    pinconf_set(0, 3, 6, 0);

    /* I2C: interface to CAM module registers */
    pinconf_set(7, 2, 5, PADCTRL_READ_ENABLE);
    pinconf_set(7, 3, 5, PADCTRL_READ_ENABLE);

    /* LPGPIO: snapshot trigger pin */
    lpgpio_init();

    /* MIPI-CSI Clock & Power */
    VBAT->PWR_CTRL = 0x33303;
    enable_cgu_clk100m();
    cpi_init();
}

/* functions to run after pin trigger */
static void run_on_cpi_trigger()
{
    cpi_capture();
    printf("M55-HE: run from cpi_trigger (lpgpio_cb_count = %u)\n", lpgpio_cb_count);
    printf("M55-HE: capture finished (cpi_cb_count = %u)\n", cpi_cb_count);
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    delay_ms_s32k(10); /* software delay to hold CPU until UART is done printing */
#endif
}

static void transition_down()
{
    /* Power Gate RX D-PHY */
    VBAT->PWR_CTRL = 0x31333;

    /* Stop the XVCLK output to camera module */
    clear_cpi_pixel_clk();

    /* Clock Gate */
    enable_syst_sram(0);
    disable_cpi_periph_clk();
    disable_csi_periph_clk();
    disable_rxdphy_configure_clock();

    /* Stop the PLL */
    PllClockConfig(0);
    BusClockConfig(2, 0, 0, 0);

    PLL_clkpll_stop();
    enable_dcdc_pfm();
}

static void transition_up()
{
    enable_dcdc_pwm();
#if defined(ENSEMBLE_SOC_GEN2) || defined(ENSEMBLE_SOC_E1C)
    BusClockConfig(2, 3, 2, 2); /* HCLK & PCLK are derived from SYSPLL, divide by 4 */
#else
    BusClockConfig(2, 3, 0, 0); /* HCLK & PCLK are derived from ACLK, divide by 1 */
#endif
    PLL_clkpll_start(38400000, true);
    PllClockConfig((1U << 20) | (1U << 4));

    /* Clock Enable */
    enable_rxdphy_configure_clock();
    enable_csi_periph_clk();
    enable_cpi_periph_clk();
    enable_syst_sram(SYST_SRAM1);

    /* Resume the XVCLK output to camera module */
    set_cpi_pixel_clk(CPI_PIX_CLKSEL_400MZ, RTE_MT9M114_CAMERA_SENSOR_MIPI_CSI_CLK_SCR_DIV);

    /* Power Enable RX D-PHY */
    VBAT->PWR_CTRL = 0x33303;
}

static void cpi_cb(uint32_t event)
{
    if(ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED & event)
    {
        cpi_done = true;
    }
}

static void cpi_init()
{
    int err;

    err = Driver_CPI.Initialize(cpi_cb);
    assert(ARM_DRIVER_OK == err);

    err = Driver_CPI.PowerControl(ARM_POWER_FULL);
    assert(ARM_DRIVER_OK == err);

    err = Driver_CPI.Control(CPI_CONFIGURE, 0);
    assert(ARM_DRIVER_OK == err);

    err = Driver_CPI.Control(CPI_CAMERA_SENSOR_CONFIGURE, 0);
    assert(ARM_DRIVER_OK == err);

    err = Driver_CPI.Control(CPI_EVENTS_CONFIGURE, ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED);
    assert(ARM_DRIVER_OK == err);
}

static void cpi_capture()
{
    int err;
    cpi_done = false;
    err = Driver_CPI.CaptureFrame(buf_cam);
    assert(ARM_DRIVER_OK == err);

    while (cpi_trigger == true) {
        /* LPGPIO interrupt clear */
        GPIO_Type *gpio = (GPIO_Type *) LPGPIO_BASE;
        gpio_interrupt_eoi(gpio, TRIGGER_PIN);

        while(cpi_done == false) {
            pm_core_enter_normal_sleep();
        }

        cpi_done = false;
        cpi_trigger = false;
        NVIC_ClearPendingIRQ(LPGPIO_COMB_IRQ_IRQn);
        NVIC_EnableIRQ(LPGPIO_COMB_IRQ_IRQn);
    }

    err = Driver_CPI.Stop();
    assert(ARM_DRIVER_OK == err);

    err = Driver_CPI.Control(CPI_EVENTS_CONFIGURE, ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED);
    assert(ARM_DRIVER_OK == err);
}

/* Function for LPGPIO combined interrupt */
void LPGPIO_COMB_IRQHandler(uint32_t event)
{
    (void) event;
    lpgpio_cb_count++;

    cpi_trigger = true;
    NVIC_DisableIRQ(LPGPIO_COMB_IRQ_IRQn);
}

/* Function for initializing the LPGPIO peripheral */
static void lpgpio_init()
{
    /* Disable the LPGPIO combined interrupt */
    NVIC_DisableIRQ(LPGPIO_COMB_IRQ_IRQn);

    uint32_t padconf = PADCTRL_READ_ENABLE;
    pinconf_set(PORT_15, TRIGGER_PIN, 0, padconf);

    GPIO_Type *gpio = (GPIO_Type *) LPGPIO_BASE;
    gpio_enable_interrupt(gpio, TRIGGER_PIN);
    gpio_interrupt_set_edge_trigger(gpio, TRIGGER_PIN);
    gpio_interrupt_set_polarity_high(gpio, TRIGGER_PIN);
    gpio_interrupt_eoi(gpio, TRIGGER_PIN);

    /* reset the interrupt flags */
    cpi_trigger = false;
    lpgpio_cb_count = 0;

    /* Enable the LPGPIO combined interrupt */
    NVIC_ClearPendingIRQ(LPGPIO_COMB_IRQ_IRQn);
    NVIC_EnableIRQ(LPGPIO_COMB_IRQ_IRQn);
}

static void lptimer_self_test()
{
    /* make LPTIMER output available through LPGPIO pin */
    gpio_set_hardware_mode((GPIO_Type*)LPGPIO_BASE, LPTIMER_CH);

    uint32_t count = (32768/2)-1;
    LPTIMER_Type *lptimer = (LPTIMER_Type *) LPTIMER_BASE;
    lptimer_disable_counter(lptimer, LPTIMER_CH);
    lptimer_set_mode_userdefined(lptimer, LPTIMER_CH);
    lptimer_load_count(lptimer, LPTIMER_CH, &count);
    lptimer_enable_counter(lptimer, LPTIMER_CH);

    /* Disable the LPTIMER channel interrupt, not used */
    NVIC_DisableIRQ(LPTIMER0_IRQ_IRQn + LPTIMER_CH);
}
