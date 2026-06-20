/*
 * st7789.c - ST7789 SPI LCD backend for pico-infonesPlus (PicoGB hardware).
 * See st7789.h for the design overview. Init sequence, MADCTL (0x60), INVOFF
 * AND the per-transaction CS framing are ported verbatim from the proven
 * pico1-gb-320 driver. This panel REQUIRES CS to be pulsed low/high around
 * every command, data and pixel-burst transaction (holding CS low permanently
 * leaves the panel blank/white).
 */
#include "st7789.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"

/* ST7789 command set */
#define ST7789_SWRESET 0x01
#define ST7789_SLPOUT  0x11
#define ST7789_NORON   0x13
#define ST7789_INVOFF  0x20
#define ST7789_INVON   0x21
#define ST7789_DISPON  0x29
#define ST7789_CASET   0x2A
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_MADCTL  0x36
#define ST7789_COLMOD  0x3A

static int                s_dma_chan = -1;
static dma_channel_config s_dma_cfg;

/* Two ping-pong scanline buffers, full panel width so the emulator can render
 * at the +32 offset; borders stay black (cleared once in st7789_init). */
static uint16_t s_linebuf[2][ST7789_WIDTH];
static int      s_linebuf_idx;

/* ---- pin/bus primitives (match GB driver naming/behaviour) ------------- */

static inline void set_cs(bool s)  { gpio_put(ST7789_PIN_CS, s); }
static inline void set_dc(bool s)  { gpio_put(ST7789_PIN_DC, s); }   /* RS */
static inline void set_rst(bool s) { gpio_put(ST7789_PIN_RST, s); }
static inline void set_bl(bool s)  { gpio_put(ST7789_PIN_BL, s); }

static void spi_w8(const uint8_t *bytes, size_t len)
{
    spi_set_format(ST7789_SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    spi_write_blocking(ST7789_SPI_PORT, bytes, len);
}

static void spi_w16(const uint16_t *hw, size_t len)
{
    spi_set_format(ST7789_SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    spi_write16_blocking(ST7789_SPI_PORT, hw, len);
}

/* Each of these pulses CS low for the transaction then raises it again. */
static void write_cmd(uint8_t cmd)
{
    set_dc(0);
    set_cs(0);
    spi_w8(&cmd, 1);
    set_cs(1);
}

static void write_data8(uint8_t d)
{
    set_dc(1);
    set_cs(0);
    spi_w8(&d, 1);
    set_cs(1);
}

static void write_data8_buf(const uint8_t *buf, size_t len)
{
    set_dc(1);
    set_cs(0);
    spi_w8(buf, len);
    set_cs(1);
}

static void set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t buf[4];
    write_cmd(ST7789_CASET);
    buf[0] = (uint8_t)(x0 >> 8); buf[1] = (uint8_t)x0;
    buf[2] = (uint8_t)(x1 >> 8); buf[3] = (uint8_t)x1;
    write_data8_buf(buf, 4);

    write_cmd(ST7789_RASET);
    buf[0] = (uint8_t)(y0 >> 8); buf[1] = (uint8_t)y0;
    buf[2] = (uint8_t)(y1 >> 8); buf[3] = (uint8_t)y1;
    write_data8_buf(buf, 4);
}

/* ---- public API -------------------------------------------------------- */

void st7789_init(void)
{
    /* (clk_peri is configured in main() before initAll, so the SD card on spi1
     * and this display both run at the final peripheral clock.) */

    /* SPI bus + DMA */
    spi_init(ST7789_SPI_PORT, ST7789_SPI_BAUD);
    spi_set_format(ST7789_SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(ST7789_PIN_CLK, GPIO_FUNC_SPI);
    gpio_set_function(ST7789_PIN_SDA, GPIO_FUNC_SPI);
    gpio_set_slew_rate(ST7789_PIN_CLK, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(ST7789_PIN_SDA, GPIO_SLEW_RATE_FAST);

    s_dma_chan = dma_claim_unused_channel(true);
    s_dma_cfg  = dma_channel_get_default_config(s_dma_chan);
    channel_config_set_transfer_data_size(&s_dma_cfg, DMA_SIZE_16);
    channel_config_set_dreq(&s_dma_cfg, spi_get_dreq(ST7789_SPI_PORT, true));
    channel_config_set_read_increment(&s_dma_cfg, true);
    channel_config_set_write_increment(&s_dma_cfg, false);

    /* Control GPIOs (SIO outputs) */
    gpio_init(ST7789_PIN_CS);  gpio_set_dir(ST7789_PIN_CS,  GPIO_OUT);
    gpio_init(ST7789_PIN_DC);  gpio_set_dir(ST7789_PIN_DC,  GPIO_OUT);
    gpio_init(ST7789_PIN_RST); gpio_set_dir(ST7789_PIN_RST, GPIO_OUT);
    gpio_init(ST7789_PIN_BL);  gpio_set_dir(ST7789_PIN_BL,  GPIO_OUT);

    /* Reset + init sequence (verbatim from pico1-gb-320). */
    set_rst(1);
    set_cs(1);
    set_dc(0);
    sleep_ms(1);

    set_bl(0);                  /* backlight off during init */

    set_rst(0); sleep_ms(10);
    set_rst(1); sleep_ms(120);

    write_cmd(ST7789_SWRESET);  sleep_ms(150);
    write_cmd(ST7789_SLPOUT);   sleep_ms(50);

    write_cmd(ST7789_COLMOD);   write_data8(0x55);   sleep_ms(10);   /* RGB565 */
    write_cmd(ST7789_MADCTL);   write_data8(0x60);                   /* 90deg CW */
    write_cmd(ST7789_INVOFF);   sleep_ms(10);   /* correct polarity for this panel */
    write_cmd(ST7789_NORON);    sleep_ms(10);

    set_window(0, 0, ST7789_WIDTH - 1, ST7789_HEIGHT - 1);
    write_cmd(ST7789_DISPON);   sleep_ms(50);

    st7789_fill(0x0000);        /* clear panel (incl. side bars) */
    set_bl(1);                  /* backlight on */
}

void st7789_fill(uint16_t color)
{
    st7789_wait_idle();
    set_window(0, 0, ST7789_WIDTH - 1, ST7789_HEIGHT - 1);
    write_cmd(ST7789_RAMWR);
    set_dc(1);
    set_cs(0);
    spi_set_format(ST7789_SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    for (uint32_t i = 0; i < (uint32_t)ST7789_WIDTH * ST7789_HEIGHT; i++)
        spi_write16_blocking(ST7789_SPI_PORT, &color, 1);
    set_cs(1);
}

uint16_t *st7789_next_linebuf(void)
{
    uint16_t *b = s_linebuf[s_linebuf_idx];
    s_linebuf_idx ^= 1;
    return b;
}

void st7789_send_line(int row, uint16_t *buf)
{
    /* Finish the previous line (also raises CS to close that burst). */
    st7789_wait_idle();

    /* Convert the line from RGB555 to the panel's RGB565 in place. Reading only
     * bits 0..14 also strips InfoNES's 0x8000 backdrop flag (bit 15) — needed by
     * the sprite-priority logic but not a real colour bit. */
    for (unsigned i = 0; i < ST7789_WIDTH; i++)
    {
        uint16_t v = buf[i];
        buf[i] = (uint16_t)(((v & 0x7C00) << 1) | ((v & 0x03E0) << 1) | (v & 0x001F));
    }

    /* Full panel width: the caller passes a 320-wide line. The menu fills all
     * 320; the NES game fills the 256 centre (X_OFFSET) and leaves the side
     * columns black. One unified path for both. */
    set_window(0, (uint16_t)row, ST7789_WIDTH - 1u, (uint16_t)row);
    write_cmd(ST7789_RAMWR);

    set_dc(1);
    set_cs(0);                  /* assert CS for the pixel burst (raised in wait_idle) */
    spi_set_format(ST7789_SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    dma_channel_configure(s_dma_chan, &s_dma_cfg,
                          &spi_get_hw(ST7789_SPI_PORT)->dr,
                          buf,
                          ST7789_WIDTH,
                          true);
}

void st7789_convert_game_line(uint16_t *buf)
{
    /* Convert the 256 active centre pixels RGB555 -> the panel's RGB565 in place
     * (reading only bits 0..14 also strips InfoNES's 0x8000 backdrop flag). Run on
     * core0 right before committing the line, so core1 is left to do pure DMA. */
    uint16_t *c = buf + ST7789_X_OFFSET;
    for (unsigned i = 0; i < ST7789_NES_WIDTH; i++)
    {
        uint16_t v = c[i];
        c[i] = (uint16_t)(((v & 0x7C00) << 1) | ((v & 0x03E0) << 1) | (v & 0x001F));
    }
}

/* Wait for the in-flight pixel DMA to drain WITHOUT raising CS, so a streamed
 * frame's RAMWR pixel burst stays open across scanlines. */
static void wait_dma_only(void)
{
    if (s_dma_chan >= 0)
        dma_channel_wait_for_finish_blocking(s_dma_chan);
    while (spi_get_hw(ST7789_SPI_PORT)->sr & SPI_SSPSR_BSY_BITS)
        tight_loop_contents();
}

void st7789_send_game_line(int row, uint16_t *buf)
{
    /* The 256 active centre pixels were already converted RGB555->RGB565 on core0
     * (st7789_convert_game_line, called from InfoNES_PostDrawLine) so that core1
     * only has to DMA them — the conversion loop (~2 ms/frame) ran serially here
     * before and was the last thing keeping the display over the 60fps budget. */
    uint16_t *c = buf + ST7789_X_OFFSET;

    /* Continuous streaming: open the window + RAMWR pixel burst ONCE per frame and
     * stream every scanline into auto-incrementing GRAM with CS held low. That
     * removes ALL per-line command/CS/format overhead (the ~5 us/line that pushed
     * us to 18 ms/frame); a frame becomes just the 256-wide pixel DMAs (~13.6 ms),
     * well under the 16.67 ms 60fps budget so the I2S audio stays fed.
     *
     * The window is set on the FIRST scanline of each frame, detected by the row
     * number dropping (InfoNES emits lines low..high, here 4..235, then wraps). It
     * is NOT keyed on row==0 — InfoNES never renders line 0, which is exactly why
     * the earlier streaming attempt produced a black screen. The caller keeps CS
     * low between lines (wait_dma_only) so the burst stays open. */
    static int prev_row = -1;
    bool new_frame = (prev_row < 0) || (row < prev_row);
    prev_row = row;

    if (new_frame)
    {
        set_cs(1); /* close the previous frame's open RAMWR burst */
        /* Column window = the 256 centred columns; row window from this first row
         * down to the bottom of the panel (auto-increment walks rows row..235). */
        set_window(ST7789_X_OFFSET, (uint16_t)row,
                   ST7789_X_OFFSET + ST7789_NES_WIDTH - 1u, ST7789_HEIGHT - 1u);
        write_cmd(ST7789_RAMWR);
        set_dc(1);
        set_cs(0); /* open the burst for the whole frame */
        /* 16-bit format set ONCE per frame (set_window above left it at 8-bit).
         * Never call spi_set_format() again mid-stream — that briefly disables the
         * SPI and corrupts the open RAMWR burst. */
        spi_set_format(ST7789_SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    }

    dma_channel_configure(s_dma_chan, &s_dma_cfg,
                          &spi_get_hw(ST7789_SPI_PORT)->dr,
                          c, ST7789_NES_WIDTH, true);
}

void st7789_wait_idle(void)
{
    if (s_dma_chan >= 0)
        dma_channel_wait_for_finish_blocking(s_dma_chan);
    while (spi_get_hw(ST7789_SPI_PORT)->sr & SPI_SSPSR_BSY_BITS)
        tight_loop_contents();
    set_cs(1);                  /* close any open pixel-burst CS frame */
}

/* ---- core1 display offload (single-producer / single-consumer ring) -----
 * core0 (emulation) renders each NES scanline into a ring slot and commits it;
 * core1 pulls committed slots and does the blocking SPI DMA. This overlaps the
 * emulation with the display transfer so core0's frame drops below the 60fps
 * budget and can keep the I2S audio fed. Only used for the GAME; the menu still
 * draws directly on core0 (core1 sits idle when the ring is empty).            */
#define ST7789_RING_N 24
static uint16_t          s_ring[ST7789_RING_N][ST7789_WIDTH];
static volatile uint16_t s_ring_row[ST7789_RING_N];
static volatile uint32_t s_ring_wr; /* core0 produce counter */
static volatile uint32_t s_ring_rd; /* core1 consume counter */

uint16_t *st7789_ring_acquire(void)
{
    /* Block until a slot is free (ring not full). */
    while ((uint32_t)(s_ring_wr - s_ring_rd) >= ST7789_RING_N)
        tight_loop_contents();
    return s_ring[s_ring_wr % ST7789_RING_N];
}

void st7789_ring_commit(int row)
{
    s_ring_row[s_ring_wr % ST7789_RING_N] = (uint16_t)row;
    __dmb();                    /* publish the pixel writes before the index */
    s_ring_wr = s_ring_wr + 1;
}

void st7789_ring_flush(void)
{
    while (s_ring_rd != s_ring_wr)
        tight_loop_contents();
    st7789_wait_idle();         /* finish the last in-flight DMA + raise CS */
}

#if ST7789_USB_DEVICE
/* DIAG (serial-diagnostics build only): wall-clock time core1 took to transfer the
 * last full frame (first-line of one frame to first-line of the next), and a free-
 * running counter of the total microseconds core1 spent busy transferring pixels.
 * core0 prints both over serial: st7789_last_frame_us vs 16667 shows whether the
 * display fits the 60fps budget; the busy delta/second shows how saturated core1 is
 * (≈1e6 => display-bound). Compiled out of the shipping (host) build entirely. */
volatile uint32_t st7789_last_frame_us = 0;
volatile uint32_t st7789_core1_busy_us = 0;
#endif

static void __attribute__((noreturn)) st7789_core1_run(void)
{
#if ST7789_USB_DEVICE
    uint64_t frame_start = 0;
    int      prev_row = -1;
#endif
    for (;;)
    {
        if (s_ring_rd != s_ring_wr)
        {
            uint32_t slot = s_ring_rd % ST7789_RING_N;
            int      row  = (int)s_ring_row[slot];

#if ST7789_USB_DEVICE
            /* First line of a frame = row dropped below the previous one. */
            if (prev_row < 0 || row < prev_row)
            {
                uint64_t now = time_us_64();
                if (frame_start != 0)
                    st7789_last_frame_us = (uint32_t)(now - frame_start);
                frame_start = now;
            }
            prev_row = row;

            uint64_t t0 = time_us_64();
#endif
            st7789_send_game_line(row, s_ring[slot]); /* opens burst on new frame; starts DMA */
            wait_dma_only(); /* wait the DMA + SPI drain; keep CS low so the burst stays open */
#if ST7789_USB_DEVICE
            st7789_core1_busy_us += (uint32_t)(time_us_64() - t0);
#endif
            __dmb();
            s_ring_rd = s_ring_rd + 1;
        }
        else
        {
            tight_loop_contents();
        }
    }
}

void st7789_start_display_core1(void)
{
    multicore_launch_core1(st7789_core1_run);
}
