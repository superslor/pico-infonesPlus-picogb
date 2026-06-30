/*
 * st7789.h - ST7789 SPI LCD backend for pico-infonesPlus (PicoGB hardware).
 *
 * Drives a 320x240 ST7789 panel in landscape over SPI0 + DMA, replacing the
 * DVI/HDMI video output so the NES emulator can run on the existing Pico-GB
 * physical build with no wiring changes.
 *
 * Geometry: the NES picture is 256x240. It is horizontally scaled to ST7789_OUT_WIDTH
 * (default 292 ≈ the NES 8:7 pixel aspect) by core1 as it streams each line, and
 * centered on the 320x240 panel (ST7789_OUT_X_OFFSET black bars on each side).
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
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Panel geometry --------------------------------------------------- */
#define ST7789_WIDTH      320u   /* panel width  (landscape) */
#define ST7789_HEIGHT     240u   /* panel height (landscape) */
#define ST7789_NES_WIDTH  256u   /* active NES picture width (source) */
#define ST7789_X_OFFSET   ((ST7789_WIDTH - ST7789_NES_WIDTH) / 2u)  /* = 32 */

/* Game OUTPUT width: the 256-wide NES picture is horizontally scaled to this and
 * centred on the panel. 256 = 1:1 (no scaling); 292 ≈ the NES 8:7 pixel aspect.
 * The packed ring slots are this wide, so it directly sets the SPI bandwidth. */
#ifndef ST7789_OUT_WIDTH
#define ST7789_OUT_WIDTH  292u   /* 256 * 8/7 ≈ 292: the NES 8:7 pixel aspect ratio */
#endif
#define ST7789_OUT_X_OFFSET ((ST7789_WIDTH - ST7789_OUT_WIDTH) / 2u)

/* 12-bit (RGB444) packed output: 2 pixels -> 3 bytes, so OUT_WIDTH pixels -> OUT_WORDS
 * 16-bit words (OUT_WIDTH must be a multiple of 4). */
#define ST7789_OUT_WORDS ((ST7789_OUT_WIDTH * 3u) / 4u)

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

/* SPI clock REQUEST. The PL022 picks the fastest /2N divisor that does NOT exceed
 * this, so the request must sit at/above the desired clk_peri/4 — otherwise the next
 * divisor (clk_peri/6) is chosen and the bus runs far slower. At clk_peri=292 MHz,
 * /4 = 73 MHz; requesting 70 would drop to /6 = 48.7 MHz. 75 MHz keeps /4 across our
 * clocks: 280→70, 292→73, 300→75 (all under the panel's ~80 MHz limit). */
#ifndef ST7789_SPI_BAUD
#define ST7789_SPI_BAUD   (75u * 1000u * 1000u)
#endif

/*
 * Initialize SPI0, the control GPIOs, the DMA channel, and the panel itself
 * (full reset + ST7789 init sequence), then clear the whole screen to black
 * so the side bars are dark. Call once at startup.
 */
void st7789_init(void);

/* Fill the entire panel with a solid RGB565 colour (blocking). */
void st7789_fill(uint16_t color);

/* Switch the panel pixel format: false = 16-bit RGB565 (menu), true = 12-bit RGB444
 * (game, packed — saves ~25% SPI/line). Call with the panel idle at game entry/exit. */
void st7789_set_color_mode(bool twelve_bit);


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
 * Horizontally scale one rendered NES scanline (ST7789_NES_WIDTH=256 RGB555 source
 * pixels at src[0]) to ST7789_OUT_WIDTH RGB565 output pixels at dst[0], using linear
 * interpolation (the horizontal pass of bilinear), and strip InfoNES's 0x8000 backdrop
 * flag. Run on CORE1 from the display loop, hidden under the per-line SPI DMA wait, so
 * the 8:7 stretch costs the emulator (core0) nothing. NOTE: src[] is converted to
 * RGB565 in place. st7789_init() builds the interpolation table.
 */
void st7789_scale_game_line(uint8_t *src, uint16_t *dst, bool darken, bool aspect_1_1);

/* Hand the driver the real (saturated) NES palette so it can build its index->RGB565 LUT.
 * The ring stores 1-byte palette INDICES (half the RAM of RGB555 => twice the scatter window);
 * core1 expands index->color via this LUT. Call once after the palette is finalized and BEFORE
 * NesPalette is switched to identity (so InfoNES/overlays write indices). pal = RGB555[n]. */
void st7789_set_palette(const uint16_t *pal, int n);

/* Select how core1 turns a committed slot into a panel line: false (default) =
 * scale 256->OUT_WIDTH for games; true = NSF, whose UI is already laid out at the
 * OUT_WIDTH panel columns, so core1 only converts RGB555->RGB565 (no scale). */
void st7789_set_nsf_mode(bool on);

/* Scanline mode: when on, core1 darkens odd output rows (CRT look). Cheap — ~free when off
 * (no per-pixel work). Set from the screen mode at game start / on the in-game toggle. */
void st7789_set_scanlines(bool on);

/* Aspect mode: false = 8:7 (bilinear stretch), true = 1:1 (256 NES pixels centred with
 * black bars, no scale). Set from the screen mode at game start / on the in-game toggle. */
void st7789_set_aspect(bool one_to_one);

/* Retune the per-line DMA pace at runtime (live tear-angle tuning). The DREQ word rate is
 * clk_sys / y, so LARGER y = slower per-line write. Lowering y speeds the write (rotates the
 * parked tear toward horizontal, where the sync pacer can slide it off the top edge); raising
 * it slows the write (toward vertical) and eats core1 headroom. No effect if y is 0 (unpaced)
 * at build time. Persisted as settings.dmaPaceY; applied at game start and on the live knob. */
void st7789_set_dma_pace(uint16_t y);

/* ---- core1 display offload ---------------------------------------------
 * Launch core1 (st7789_start_display_core1) once at startup. During a game the
 * emulator (core0) calls st7789_ring_acquire() to get a packed ST7789_OUT_WIDTH-wide
 * line buffer to render into, then st7789_ring_commit(row) to hand it to core1.
 * core1 batch-DMAs contiguous runs of committed scanlines to the panel. Call
 * st7789_ring_flush() before drawing on core0 again (e.g. returning to the menu)
 * so core1's in-flight transfers complete and the SPI bus is free. */
void      st7789_start_display_core1(void);
uint16_t *st7789_ring_acquire(void);
void      st7789_ring_commit(int row);
void      st7789_ring_flush(void);

/* Block until the last queued line DMA has fully completed. */
void st7789_wait_idle(void);

#if ST7789_USB_DEVICE
/* DIAG (serial-diagnostics build only): period (us) of the last displayed frame
 * (first-line to first-line; ~16667 when paced to 60fps). */
extern volatile uint32_t st7789_last_frame_us;

/* DIAG: free-running total of microseconds core1 spent IDLE waiting for core0.
 * The true display-active time per second = 1,000,000 - (idle delta over 1 s);
 * the smaller that active figure, the more headroom the display has. */
extern volatile uint32_t st7789_core1_idle_us;

/* DIAG: us core0 spent blocked in ring_acquire (ring full). Large => the display
 * (core1) is the bottleneck (decoupling/ping-pong can help); ~0 => emulation is. */
extern volatile uint32_t st7789_acquire_block_us;

/* DIAG: us for one gap-free contiguous DMA burst (raw SPI throughput, no per-line
 * setup). Call once before launching core1. st7789_calib_us holds the result. */
void st7789_calibrate_spi(void);
extern volatile uint32_t st7789_calib_us;
extern volatile uint32_t st7789_calib_clkperi; /* clk_peri Hz */
extern volatile uint32_t st7789_calib_baud;    /* achieved SPI bit clock Hz */
#endif

#ifdef __cplusplus
}
#endif

#endif /* ST7789_H */
