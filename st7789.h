/*
 * st7789.h - ST7789 SPI LCD backend for pico-infonesPlus (PicoGB hardware).
 *
 * Drives a 320x240 ST7789 panel in landscape over SPI0 + DMA, replacing the
 * DVI/HDMI video output so the NES emulator can run on the existing Pico-GB
 * physical build with no wiring changes.
 *
 * Geometry: the NES picture is 256x240. The panel is 320x240, so the image is
 * centered horizontally with 32px black bars on each side (ST7789_X_OFFSET).
 * No scaling is performed.
 *
 * Pin map and SPI parameters match the Pico-GB "320 landscape" build
 * (pico1-gb-320): proven-correct init, MADCTL 0x60, INVOFF, mode 0, 70 MHz.
 * Any of the ST7789_PIN_* / ST7789_SPI_* macros can be overridden from the
 * board config (compile definitions) without editing this file.
 */
#ifndef ST7789_H
#define ST7789_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Panel geometry --------------------------------------------------- */
#define ST7789_WIDTH      320u   /* panel width  (landscape) */
#define ST7789_HEIGHT     240u   /* panel height (landscape) */
#define ST7789_NES_WIDTH  256u   /* active NES picture width  */
#define ST7789_X_OFFSET   ((ST7789_WIDTH - ST7789_NES_WIDTH) / 2u)  /* = 32 */

/* ---- Pin map (GP numbers) — overridable from the board config ---------- */
#ifndef ST7789_SPI_PORT
#define ST7789_SPI_PORT   spi0
#endif
#ifndef ST7789_PIN_CS
#define ST7789_PIN_CS     17
#endif
#ifndef ST7789_PIN_CLK
#define ST7789_PIN_CLK    18
#endif
#ifndef ST7789_PIN_SDA          /* MOSI */
#define ST7789_PIN_SDA    19
#endif
#ifndef ST7789_PIN_DC           /* a.k.a. RS */
#define ST7789_PIN_DC     20
#endif
#ifndef ST7789_PIN_RST
#define ST7789_PIN_RST    21
#endif
#ifndef ST7789_PIN_BL           /* backlight / LED */
#define ST7789_PIN_BL     22
#endif

/* SPI clock. 70 MHz is proven on this panel at a 280 MHz system clock
 * (clk_peri/4). The achieved rate depends on clk_peri; the driver requests
 * this and lets the PL022 pick the nearest divisor. */
#ifndef ST7789_SPI_BAUD
#define ST7789_SPI_BAUD   (70u * 1000u * 1000u)
#endif

/*
 * Initialize SPI0, the control GPIOs, the DMA channel, and the panel itself
 * (full reset + ST7789 init sequence), then clear the whole screen to black
 * so the side bars are dark. Call once at startup.
 */
void st7789_init(void);

/* Fill the entire panel with a solid RGB565 colour (blocking). */
void st7789_fill(uint16_t color);

/*
 * Return one of two internal ping-pong line buffers, each ST7789_WIDTH (320)
 * RGB565 words wide. The caller renders the NES scanline into [X_OFFSET .. +255]
 * (the borders are pre-cleared and left untouched). Pair each call with one
 * st7789_send_line() before requesting the next buffer twice in a row.
 */
uint16_t *st7789_next_linebuf(void);

/*
 * Send one scanline (320 RGB555 words) to the panel. Converts the line from
 * RGB555 to the panel's RGB565 IN PLACE (this also strips InfoNES's 0x8000
 * backdrop flag in bit 15), then DMAs it to panel row `row`. Waits for the
 * previous line's DMA to finish first, then starts this transfer and returns
 * (so the emulator can render the next line while the SPI DMA runs).
 * NOTE: `buf` is modified in place (it is one of the internal ping-pong buffers).
 */
void st7789_send_line(int row, uint16_t *buf);

/*
 * Game-optimized scanline: sends ONLY the 256 active pixels (buf + X_OFFSET) to
 * panel columns 32..287, skipping the static black side bars (which must be
 * pre-cleared once, e.g. via st7789_fill, before the game starts). Saves ~20% of
 * the per-frame SPI bandwidth vs the full-width path. Converts RGB555->RGB565 in
 * place like st7789_send_line. `buf` is the full 320-wide line buffer.
 */
void st7789_send_game_line(int row, uint16_t *buf);

/*
 * Convert the 256 active centre pixels of a 320-wide line buffer from RGB555 to
 * the panel's RGB565 in place (and strip InfoNES's 0x8000 backdrop flag). Call
 * this on core0 just before st7789_ring_commit() so core1 only DMAs pixels.
 */
void st7789_convert_game_line(uint16_t *buf);

/* ---- core1 display offload ---------------------------------------------
 * Launch core1 (st7789_start_display_core1) once at startup. During a game the
 * emulator (core0) calls st7789_ring_acquire() to get a line buffer to render
 * into, then st7789_ring_commit(row) to hand it to core1 for the SPI DMA. Call
 * st7789_ring_flush() before drawing on core0 again (e.g. returning to the menu)
 * so core1's in-flight transfers complete and the SPI bus is free. */
void      st7789_start_display_core1(void);
uint16_t *st7789_ring_acquire(void);
void      st7789_ring_commit(int row);
void      st7789_ring_flush(void);

/* Block until the last queued line DMA has fully completed. */
void st7789_wait_idle(void);

#if ST7789_USB_DEVICE
/* DIAG (serial-diagnostics build only): time (us) core1 took to transfer the last
 * full game frame. Compare to 16667 us; if it exceeds that, the display is the
 * 60fps bottleneck. */
extern volatile uint32_t st7789_last_frame_us;

/* DIAG: free-running total of microseconds core1 spent busy pushing pixels.
 * Sample the delta over one second: ~1,000,000 means core1 is saturated (the
 * display is the bottleneck); well below means core0 emulation is. */
extern volatile uint32_t st7789_core1_busy_us;
#endif

#ifdef __cplusplus
}
#endif

#endif /* ST7789_H */
