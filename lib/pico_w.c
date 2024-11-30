#include "pico_w.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/dma.h"

void OV7670_capture(uint32_t *dest, uint16_t width, uint16_t height,
                    volatile uint32_t *vsync_reg, uint32_t vsync_bit,
                    volatile uint32_t *hsync_reg, uint32_t hsync_bit) {
    // Configure the PWM for XCLK
    uint slice_num = pwm_gpio_to_slice_num(OV7670_XCLK_PIN);
    pwm_set_wrap(slice_num, 1);
    pwm_set_gpio_level(OV7670_XCLK_PIN, 1);
    pwm_set_enabled(slice_num, true);

    // Configure DMA for capturing data
    dma_channel_config c = dma_channel_get_default_config(DMA_CHANNEL);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    dma_channel_configure(DMA_CHANNEL, &c,
                          dest,        // Destination pointer
                          &PIO_SM_RXF, // Source pointer
                          width * height, // Number of transfers
                          true);       // Start immediately

    // Wait for VSYNC
    while (!(*vsync_reg & vsync_bit)) {}
    while (*vsync_reg & vsync_bit) {}

    // Wait for DMA to complete
    dma_channel_wait_for_finish_blocking(DMA_CHANNEL);

    // Disable PWM
    pwm_set_enabled(slice_num, false);
}