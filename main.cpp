#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/divider.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/watchdog.h"
#include "util/work_meter.h"
#include "InfoNES.h"
#include "InfoNES_System.h"
#include "InfoNES_pAPU.h"
#include "InfoNES_Region.h"
#include "ff.h"
#include "tusb.h"
#include "gamepad.h"
#include "rom_selector.h"
#include "menu.h"
#include "nespad.h"
#include "wiipad.h"
#include "FrensHelpers.h"
#include "settings.h"
#include "FrensFonts.h"
#include "vumeter.h"
#include "menu_settings.h"
#include "state.h"
#include "soundrecorder.h"
#include "pico/bootrom.h"
#include "InfoNES_FDS.h"
#include "InfoNES_NSF.h"
#if USE_ST7789
#include "st7789.h"
#include "gpio_buttons.h"
#endif
#if EMBEDDED_NES_ROM
extern "C" const unsigned char embedded_nes_rom[];
extern "C" const unsigned int embedded_nes_rom_len;
#endif
bool isFatalError = false;
//static bool pendingLoadState = false;
char *romName;
bool showSettings = false;
bool loadSaveStateMenu = false;
SaveStateTypes quickSaveAction = SaveStateTypes::NONE;
static uint32_t start_tick_us = 0;
static uint32_t fps = 0;
static uint8_t framesbeforeAutoStateIsLoaded = 0;
#if USE_ST7789
// 292 MHz on the PicoGB build: SPI0 = clk_peri/4 = 73 MHz baud. The RP2040 PL022
// inserts ~1.5 clocks between 16-bit frames, so effective throughput ≈ 66.7 MHz
// (vs ~64 at 280) — enough to drive the wider 8:7 (292px) image at 60fps. Also
// speeds emulation + the horizontal scaler. MUST be an exactly-achievable PLL freq
// (VCO=12*N): 292 = VCO 876 (12*73) / 3. 290 is NOT achievable -> set_sys_clock_khz
// panics. 300 (achievable) was unstable at 1.20V; we run 1.25V (see vreg below).
#define EMULATOR_CLOCKFREQ_KHZ 292000 //  Overclock frequency in kHz when using Emulator
#else
#define EMULATOR_CLOCKFREQ_KHZ 252000 //  Overclock frequency in kHz when using Emulator
#endif

// Note: When using framebuffer, AUDIOBUFFERSIZE must be increased to 1024
#if PICO_RP2350
#define AUDIOBUFFERSIZE 1024
#elif USE_ST7789
#define AUDIOBUFFERSIZE 1024
#else
#define AUDIOBUFFERSIZE 256
#endif

#if USE_ST7789
// Output gain for the I2S DAC on the ST7789/PicoGB build. The NES APU mix is very
// low level (≈0..2445 of 32767), so this multiplies it toward full scale (clamped).
// Stored in settings.audioGain (persisted). Default 1 (a comfortable level). Adjusted
// in-game with SELECT+LEFT/RIGHT (down/up).
#define ST7789_AUDIO_GAIN_MIN 0
#define ST7789_AUDIO_GAIN_MAX 6

// Frames to keep the on-screen "VOL n" indicator visible after a change, and a debounce
// so we write the settings file only once after adjustment settles.
#define ST7789_OSD_FRAMES       90
#define ST7789_OSD_SAVE_FRAMES  60
static int  g_osdFrames = 0;        // >0 => draw the indicator (counts down per frame)
static int  g_osdSaveCountdown = 0; // >0 => save settings when it reaches 0
enum { OSD_VOL = 0, OSD_SYNC = 1, OSD_PACE = 2 }; // which value the on-screen indicator shows
static int g_osdKind = OSD_VOL;

// In-game sync (anti-tearing pacer) trim range, in TENTHS of a us (0.1us step, SELECT+B+L/R).
// + = faster fps / shorter frame. [-8000,3000] tenths = [-800,+300] us, same span as before.
#define ST7789_SYNC_ADJ_MIN (-20000)  /* tenths-of-us; wide enough to re-park after an FRCTRL2 line-rate change */
#define ST7789_SYNC_ADJ_MAX (8000)
#define ST7789_SYNC_ADJ_STEP (1)       /* fine parking step = 1 tenth = 0.1us per press (SELECT+B+L/R) */

// In-game per-line DMA pace range (tear-angle knob, SELECT+A+L/R). clk_sys/y word rate, so
// smaller y = faster write = toward horizontal. Bounded: below ~70 pacing stops mattering
// (SPI-limited); above ~96 core1 runs out of frame budget (underrun). Watch the diag fps.
#define ST7789_DMA_PACE_MIN (70)
#define ST7789_DMA_PACE_MAX (110)

// Screen mode -> panel: scanlines on for the SCANLINE_* modes, 1:1 aspect for the *_1_1
// modes. Pushed to the panel via st7789_set_scanlines() / st7789_set_aspect().
#define ST7789_SCANLINES_ON() (settings.screenMode == ScreenMode::SCANLINE_8_7 || \
                               settings.screenMode == ScreenMode::SCANLINE_1_1)
#define ST7789_ASPECT_1_1()   (settings.screenMode == ScreenMode::SCANLINE_1_1 || \
                               settings.screenMode == ScreenMode::NOSCANLINE_1_1)

// Cycle the screen mode in-game WITHOUT writing the settings file. Frens::screenMode()
// calls FrensSettings::savesettings() on every press, and that SD write blocks core0 for
// several ms (audio underrun + a video hitch). Instead we cycle the mode in RAM, apply it
// to the panel, and arm the same debounced-save countdown the volume control uses, so the
// SD write happens just once, ~1s after the last press. Mirrors Frens::screenMode()'s wrap.
static bool st7789_cycle_screen_mode(int incr)
{
    constexpr int kModeCount = 4;
    // Cycle order for SELECT+UP (incr=+1): 8:7 no-SL (default) -> 8:7 SL -> 1:1 no-SL -> 1:1 SL -> wrap.
    // SELECT+DOWN (incr=-1) walks it backwards. (The raw enum order differs, so we map through this.)
    static const int kCycle[kModeCount] = {
        static_cast<int>(ScreenMode::NOSCANLINE_8_7),
        static_cast<int>(ScreenMode::SCANLINE_8_7),
        static_cast<int>(ScreenMode::NOSCANLINE_1_1),
        static_cast<int>(ScreenMode::SCANLINE_1_1),
    };
    int pos = 0;
    for (int i = 0; i < kModeCount; i++)
        if (kCycle[i] == static_cast<int>(settings.screenMode)) { pos = i; break; }
    int current = static_cast<int>(settings.screenMode);
    for (int attempts = 0; attempts < kModeCount; attempts++)
    {
        pos = (pos + incr) & 3; // wrap 0..3 within the cycle order
        if (g_available_screen_modes[kCycle[pos]])
        {
            current = kCycle[pos];
            break;
        }
    }
    settings.screenMode = static_cast<ScreenMode>(current);
    bool scaleMode8_7_ = Frens::applyScreenMode(settings.screenMode); // applies, does NOT save
    st7789_set_scanlines(ST7789_SCANLINES_ON());
    st7789_set_aspect(ST7789_ASPECT_1_1());
    g_osdSaveCountdown = ST7789_OSD_SAVE_FRAMES; // persist once, after presses settle
    return scaleMode8_7_;
}

// Match the I2S DAC sample rate to our ACTUAL frame rate so the audio buffer never drifts
// when the pacer is tuned off 60.0fps to park the tearing seam. The APU emits 735 samples
// per frame (NTSC 44100/60). syncSpeedAdj is in TENTHS of a us, so the frame period in tenths
// is (166670 - syncSpeedAdj); matched rate = 735e6 / period_us = 7350e6 / period_tenths.
// ~44100Hz at sync 0, ~43989Hz at -415 — the <0.3% pitch shift is inaudible, but it removes
// the slow-drain/underrun that forced us to 60.0fps. Call at game start and on every change.
static void st7789_match_audio_clock(void)
{
    int32_t period_tenths = 166670 - settings.syncSpeedAdj;
    if (period_tenths > 0 && settings.flags.useExtAudio)
        EXT_AUDIO_SET_SAMPLE_RATE((uint32_t)(7350000000ULL / (uint32_t)period_tenths));
}
#endif

// DVI Gain (Q8). 256 = 1.0x, 384 = 1.5x, 512 = 2.0x
#ifndef DVI_AUDIO_GAIN_Q8
#define DVI_AUDIO_GAIN_Q8 1024
#endif
// Current gain setting (DVI audio)
static int g_dvi_audio_gain_q8 = DVI_AUDIO_GAIN_Q8;

// Recording gain (Q8). 256 = 1.0x, 512 = 2.0x
#ifndef RECORD_GAIN_Q8
#define RECORD_GAIN_Q8 2048
#endif
static int g_record_gain_q8 = RECORD_GAIN_Q8;

static uint32_t CPUFreqKHz = EMULATOR_CLOCKFREQ_KHZ;
// Visibility configuration for options menu (NES specific)
// 1 = show option line, 0 = hide.
// Order must match enum in menu_options.h
#if PICO_RP2350
static const MenuFdsHooks fdsMenuHooks = {
    fdsCurrentSwapValue,
    fdsGetNumSides,
    fdsRequestSwap,
    fdsRequestEject
};
#endif

// Non-const so the FDS disk-swap entry can be flipped on per-ROM after
// fdsParse() detects we're loading a .fds. The pointer in
// `g_settings_visibility` is `const int8_t *`, but it can point at
// non-const storage just fine.
int8_t g_settings_visibility_nes[MOPT_COUNT] = {
    0,                               // Exit Game, or back to menu. Always visible when in-game.
    0,                               // Reset Game
    0,                               // Save / Restore State
    1,                               // Screen Mode
    0,                               // Scanlines toggle (superseded by Screen Mode)
    HSTX,                            // Scanline Type (HSTX only)
    1,                               // FPS Overlay
    0,                               // Audio Enable
    0,                               // Frame Skip
    HSTX && ENABLEDVI,                            // Display Mode (HDMI or DVI, only when HSTX is enabled, because non-HSTX builds always use HDMI)
    (EXT_AUDIO_IS_ENABLED ), // External Audio
    1,                               // Font Color
    1,                               // Font Back Color
    ENABLE_VU_METER,                 // VU Meter
    //(HW_CONFIG == 8),                // Fruit Jam Internal Speaker
    (HW_CONFIG == 8),                // Fruit Jam Volume Control
    0,                               // DMG Palette (NES emulator does not use GameBoy palettes)
    0,                               // Border Mode (Super Gameboy style borders not applicable for NES)
    1,                               // Rapid Fire on A
    1,                               // Rapid Fire on B
    0,                               // Auto Swap FDS, enabled at runtime on RP2350
    0,                               // Auto Insert Disk A, , enabled at runtime on RP2350
    1,                               // Enter bootsel mode
    0                                // FDS Disk Swap (toggled on after fdsParse succeeds)
};
// #if defined(__riscv)
// const uint8_t g_available_screen_modes[] = {
//     0, // SCANLINE_8_7,      
//     0, // NOSCANLINE_8_7,
//     1, // SCANLINE_1_1,
//     1  // NOSCANLINE_1_1
// };
// #else
const uint8_t g_available_screen_modes_nes[] = {
    1, // SCANLINE_8_7,
    1, // NOSCANLINE_8_7,
    1, // SCANLINE_1_1,
    1  // NOSCANLINE_1_1
    };
//#endif

namespace
{
    ROMSelector romSelector_;
}

#if WII_PIN_SDA >= 0 and WII_PIN_SCL >= 0
// Cached Wii pad state updated once per frame in ProcessAfterFrameIsRendered()
static uint16_t wiipad_raw_cached = 0;
#endif
#if 0
#if !HSTX
// convert RGB565 to RGB444
#define CC(x) (((x >> 1) & 15) | (((x >> 6) & 15) << 4) | (((x >> 11) & 15) << 8))
#else 
// convert RGB565 to RGB555
#define CC(x) ((((x) >> 11) & 0x1F) << 10 | (((x) >> 6) & 0x1F) << 5 | ((x) & 0x1F))
#endif
const WORD __not_in_flash_func(NesPalette)[64] = {
    CC(0x39ce), CC(0x1071), CC(0x0015), CC(0x2013), CC(0x440e), CC(0x5402), CC(0x5000), CC(0x3c20),
    CC(0x20a0), CC(0x0100), CC(0x0140), CC(0x00e2), CC(0x0ceb), CC(0x0000), CC(0x0000), CC(0x0000),
    CC(0x5ef7), CC(0x01dd), CC(0x10fd), CC(0x401e), CC(0x5c17), CC(0x700b), CC(0x6ca0), CC(0x6521),
    CC(0x45c0), CC(0x0240), CC(0x02a0), CC(0x0247), CC(0x0211), CC(0x0000), CC(0x0000), CC(0x0000),
    CC(0x7fff), CC(0x1eff), CC(0x2e5f), CC(0x223f), CC(0x79ff), CC(0x7dd6), CC(0x7dcc), CC(0x7e67),
    CC(0x7ae7), CC(0x4342), CC(0x2769), CC(0x2ff3), CC(0x03bb), CC(0x0000), CC(0x0000), CC(0x0000),
    CC(0x7fff), CC(0x579f), CC(0x635f), CC(0x6b3f), CC(0x7f1f), CC(0x7f1b), CC(0x7ef6), CC(0x7f75),
    CC(0x7f94), CC(0x73f4), CC(0x57d7), CC(0x5bf9), CC(0x4ffe), CC(0x0000), CC(0x0000), CC(0x0000)};
#endif
#if 1
#if !HSTX
#if USE_ST7789
// Keep the RGB555 source values as-is. The ST7789 driver converts RGB555->RGB565
// at blit time, which also strips InfoNES's 0x8000 backdrop flag (bit 15). That
// flag is needed by the sprite-priority logic (pPoint[i] >> 15) but must NOT reach
// the RGB565 panel, where bit 15 is the red MSB.
#define CC(x) (x)
#else
// RGB555 to RGB444 (PicoDVI 12bpp line buffer)
#define CC(x) (((x >> 1) & 15) | (((x >> 6) & 15) << 4) | (((x >> 11) & 15) << 8))
#endif
const WORD __not_in_flash_func(NesPalette)[64] = {
    CC(0x39ce), CC(0x1071), CC(0x0015), CC(0x2013), CC(0x440e), CC(0x5402), CC(0x5000), CC(0x3c20),
    CC(0x20a0), CC(0x0100), CC(0x0140), CC(0x00e2), CC(0x0ceb), CC(0x0000), CC(0x0000), CC(0x0000),
    CC(0x5ef7), CC(0x01dd), CC(0x10fd), CC(0x401e), CC(0x5c17), CC(0x700b), CC(0x6ca0), CC(0x6521),
    CC(0x45c0), CC(0x0240), CC(0x02a0), CC(0x0247), CC(0x0211), CC(0x0000), CC(0x0000), CC(0x0000),
    CC(0x7fff), CC(0x1eff), CC(0x2e5f), CC(0x223f), CC(0x79ff), CC(0x7dd6), CC(0x7dcc), CC(0x7e67),
    CC(0x7ae7), CC(0x4342), CC(0x2769), CC(0x2ff3), CC(0x03bb), CC(0x0000), CC(0x0000), CC(0x0000),
    CC(0x7fff), CC(0x579f), CC(0x635f), CC(0x6b3f), CC(0x7f1f), CC(0x7f1b), CC(0x7ef6), CC(0x7f75),
    CC(0x7f94), CC(0x73f4), CC(0x57d7), CC(0x5bf9), CC(0x4ffe), CC(0x0000), CC(0x0000), CC(0x0000)};
#else
// RGB888 to RGB555
#define CC(c) (((c & 0xf8) >> 3) | ((c & 0xf800) >> 6) | ((c & 0xf80000) >> 9))
const WORD __not_in_flash_func(NesPalette)[64] = {
    CC(0x626262), CC(0x001C95), CC(0x1904AC), CC(0x42009D),
    CC(0x61006B), CC(0x6E0025), CC(0x650500), CC(0x491E00),
    CC(0x223700), CC(0x004900), CC(0x004F00), CC(0x004816),
    CC(0x00355E), CC(0x000000), CC(0x000000), CC(0x000000),

    CC(0xABABAB), CC(0x0C4EDB), CC(0x3D2EFF), CC(0x7115F3),
    CC(0x9B0BB9), CC(0xB01262), CC(0xA92704), CC(0x894600),
    CC(0x576600), CC(0x237F00), CC(0x008900), CC(0x008332),
    CC(0x006D90), CC(0x000000), CC(0x000000), CC(0x000000),

    CC(0xFFFFFF), CC(0x57A5FF), CC(0x8287FF), CC(0xB46DFF),
    CC(0xDF60FF), CC(0xF863C6), CC(0xF8746D), CC(0xDE9020),
    CC(0xB3AE00), CC(0x81C800), CC(0x56D522), CC(0x3DD36F),
    CC(0x3EC1C8), CC(0x4E4E4E), CC(0x000000), CC(0x000000),

    CC(0xFFFFFF), CC(0xBEE0FF), CC(0xCDD4FF), CC(0xE0CAFF),
    CC(0xF1C4FF), CC(0xFCC4EF), CC(0xFDCACE), CC(0xF5D4AF),
    CC(0xE6DF9C), CC(0xD3E99A), CC(0xC2EFA8), CC(0xB7EFC4),
    CC(0xB6EAE5), CC(0xB8B8B8), CC(0x000000), CC(0x000000)};
#endif
#endif

#if USE_ST7789
// Very-slight saturation boost, applied once to the NES palette at startup (zero per-
// frame cost). Per RGB555 channel: out = luma + (chan - luma) * NUM/16; NUM=16 is no-op.
#define ST7789_SAT_NUM 17   // 17/16 = +6.25% saturation (gentle)
static void st7789ApplyPaletteSaturation(void)
{
    WORD *pal = (WORD *)NesPalette;   // RAM-resident (__not_in_flash_func); boosted once, pre-render
    for (int i = 0; i < 64; i++)
    {
        int v = pal[i];
        int r = (v >> 10) & 0x1F, g = (v >> 5) & 0x1F, b = v & 0x1F;
        int luma = (5 * r + 9 * g + 2 * b) >> 4;        // approx luma (weights sum to 16)
        r = luma + (((r - luma) * ST7789_SAT_NUM) >> 4);
        g = luma + (((g - luma) * ST7789_SAT_NUM) >> 4);
        b = luma + (((b - luma) * ST7789_SAT_NUM) >> 4);
        if (r < 0) r = 0; else if (r > 31) r = 31;
        if (g < 0) g = 0; else if (g > 31) g = 31;
        if (b < 0) b = 0; else if (b > 31) b = 31;
        pal[i] = (WORD)((r << 10) | (g << 5) | b);
    }
}
#endif

uint32_t getCurrentNVRAMAddr()
{

    if (!romSelector_.getCurrentROM())
    {
        return {};
    }
    int slot = romSelector_.getCurrentNVRAMSlot();
    if (slot < 0)
    {
        return {};
    }
    printf("SRAM slot %d\n", slot);
    return ROM_FILE_ADDR - SRAM_SIZE * (slot + 1);
}

void saveNVRAM()
{
    char pad[FF_MAX_LFN];
    char fileName[FF_MAX_LFN];
    strcpy(fileName, Frens::GetfileNameFromFullPath(romName));
    Frens::stripextensionfromfilename(fileName);
#if PICO_RP2350
    if (IsFDS)
    {
        snprintf(pad, FF_MAX_LFN, "%s/%s_fds", GAMESAVEDIR, fileName);
        fdsSaveSidecar(pad);
        return;
    }
#endif
    if (!SRAMwritten)
    {
        printf("SRAM not updated.\n");
        return;
    }
    snprintf(pad, FF_MAX_LFN, "%s/%s.SAV", GAMESAVEDIR, fileName);
    printf("Save SRAM to %s\n", pad);
    FIL fil;
    FRESULT fr;
    fr = f_open(&fil, pad, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK)
    {
        snprintf(ErrorMessage, ERRORMESSAGESIZE, "Cannot open save file: %d", fr);
        printf("%s\n", ErrorMessage);
        return;
    }
    size_t bytesWritten;
    fr = f_write(&fil, SRAM, SRAM_SIZE, &bytesWritten);
    if (bytesWritten < SRAM_SIZE)
    {
        snprintf(ErrorMessage, ERRORMESSAGESIZE, "Error writing save: %d %d/%d written", fr, bytesWritten, SRAM_SIZE);
        printf("%s\n", ErrorMessage);
    }
    f_close(&fil);
    printf("done\n");
    SRAMwritten = false;
}

bool loadNVRAM()
{
    char pad[FF_MAX_LFN];
    FILINFO fno;
    bool ok = false;
    char fileName[FF_MAX_LFN];
    strcpy(fileName, Frens::GetfileNameFromFullPath(romName));
    Frens::stripextensionfromfilename(fileName);

#if PICO_RP2350
    if (IsFDS)
    {
        snprintf(pad, FF_MAX_LFN, "%s/%s_fds", GAMESAVEDIR, fileName);
        fdsSetSaveBasePath(pad);
        return fdsLoadSidecar(pad);
    }
#endif

    snprintf(pad, FF_MAX_LFN, "%s/%s.SAV", GAMESAVEDIR, fileName);

    FIL fil;
    FRESULT fr;

    size_t bytesRead;
    if (auto addr = getCurrentNVRAMAddr())
    {
        printf("Load SRAM from %s\n", pad);
        fr = f_stat(pad, &fno);
        if (fr == FR_NO_FILE)
        {
            printf("Save file not found, load SRAM from flash %x\n", addr);
            memcpy(SRAM, reinterpret_cast<void *>(addr), SRAM_SIZE);
            ok = true;
        }
        else
        {
            if (fr == FR_OK)
            {
                printf("Loading save file %s\n", pad);
                fr = f_open(&fil, pad, FA_READ);
                if (fr == FR_OK)
                {
                    fr = f_read(&fil, SRAM, SRAM_SIZE, &bytesRead);
                    if (fr == FR_OK)
                    {
                        printf("Savefile read from disk\n");
                        ok = true;
                    }
                    else
                    {
                        snprintf(ErrorMessage, ERRORMESSAGESIZE, "Cannot read save file: %d %d/%d read", fr, bytesRead, SRAM_SIZE);
                        printf("%s\n", ErrorMessage);
                    }
                }
                else
                {
                    snprintf(ErrorMessage, ERRORMESSAGESIZE, "Cannot open save file: %d", fr);
                    printf("%s\n", ErrorMessage);
                }
                f_close(&fil);
            }
            else
            {
                snprintf(ErrorMessage, ERRORMESSAGESIZE, "f_stat() failed on save file: %d", fr);
                printf("%s\n", ErrorMessage);
            }
        }
    }
    else
    {
        ok = true;
    }
    SRAMwritten = false;
    return ok;
}

static DWORD prevButtons[2]{};
static int rapidFireMask[2]{};
static bool g_syncBUsed[2]{}; // per-player: B-hold consumed by a SELECT+B+L/R sync gesture (undoes the B-press rapid-fire toggle)
static bool g_paceAUsed[2]{}; // per-player: A-hold consumed by a SELECT+A+L/R pace gesture (undoes the A-press rapid-fire toggle)
static int rapidFireCounter = 0;
static bool reset = false;
static bool resetGame = false;
void InfoNES_PadState(DWORD *pdwPad1, DWORD *pdwPad2, DWORD *pdwSystem)
{
    static constexpr int LEFT = 1 << 6;
    static constexpr int RIGHT = 1 << 7;
    static constexpr int UP = 1 << 4;
    static constexpr int DOWN = 1 << 5;
    static constexpr int SELECT = 1 << 2;
    static constexpr int START = 1 << 3;
    static constexpr int A = 1 << 0;
    static constexpr int B = 1 << 1;

    // moved variables outside function body because prevButtons gets initialized to 0 everytime the function is called.
    // This is strange because a static variable inside a function is only initialsed once and retains it's value
    // throughout different function calls.
    // Am i missing something?
    // static DWORD prevButtons[2]{};
    // static int rapidFireMask[2]{};
    // static int rapidFireCounter = 0;

    ++rapidFireCounter;
   
    bool usbConnected = false;
    for (int i = 0; i < 2; ++i)
    {
        auto &dst = i == 0 ? *pdwPad1 : *pdwPad2;
        auto &gp = io::getCurrentGamePadState(i);
        if (i == 0)
        {
            usbConnected = gp.isConnected();
        }
        int v = (gp.buttons & io::GamePadState::Button::LEFT ? LEFT : 0) |
                (gp.buttons & io::GamePadState::Button::RIGHT ? RIGHT : 0) |
                (gp.buttons & io::GamePadState::Button::UP ? UP : 0) |
                (gp.buttons & io::GamePadState::Button::DOWN ? DOWN : 0) |
                (gp.buttons & io::GamePadState::Button::A ? A : 0) |
                (gp.buttons & io::GamePadState::Button::B ? B : 0) |
                (gp.buttons & io::GamePadState::Button::SELECT ? SELECT : 0) |
                (gp.buttons & io::GamePadState::Button::START ? START : 0) |
                0;
#if NES_PIN_CLK != -1
        // When USB controller is connected both NES ports act as controller 2
        if (usbConnected)
        {
            if (i == 1)
            {
                v = v | nespad_states[1] | nespad_states[0];
            }
        }
        else
        {
            v |= nespad_states[i];
        }
#endif

// When USB controller is connected  wiipad acts as controller 2
#if WII_PIN_SDA >= 0 and WII_PIN_SCL >= 0
        if (usbConnected)
        {
            if (i == 1)
            {
                v |= wiipad_raw_cached;
            }
        }
        else // if no USB controller is connected, wiipad acts as controller 1
        {
            if (i == 0)
            {
                v |= wiipad_raw_cached;
            }
        }
#endif

        int rv = v;
        rapidFireMask[i] = (settings.flags.rapidFireOnA ? A : 0) |
                           (settings.flags.rapidFireOnB ? B : 0);
        if (rapidFireCounter & 2)
        {
            // 15 fire/sec
            rv &= ~rapidFireMask[i];
        }

        dst = rv;

        // Reboot to BOOTSEL mode for flashing (player 1 only)
        if (i == 0 && (v & (SELECT | START | UP | A)) == (SELECT | START | UP | A)) {
             reset_usb_boot(0, 0);
        }

        auto p1 = v;

        auto pushed = v & ~prevButtons[i];

        if (p1 & START)
        {
            if (pushed & A) // Toggle frame rate
            {
                settings.flags.displayFrameRate = !settings.flags.displayFrameRate;
                // FrensSettings::savesettings();
            } else if (pushed & B)
            {
#if PICO_RP2350
               if (Frens::isPsramEnabled() && !SoundRecorder::isRecording()) {
                     SoundRecorder::startRecording();
               } 
#endif
            } else if (pushed & UP) {
                loadSaveStateMenu = true;
                quickSaveAction = SaveStateTypes::LOAD;
            } else if (pushed & DOWN) {
                loadSaveStateMenu = true;
                quickSaveAction = SaveStateTypes::SAVE;
            } else if (pushed & LEFT) {
#if HW_CONFIG == 8
               settings.fruitjamVolumeLevel = std::max(-63, settings.fruitjamVolumeLevel - 1);
               EXT_AUDIO_SETVOLUME(settings.fruitjamVolumeLevel);
#endif
            } else if (pushed & RIGHT) {
#if HW_CONFIG == 8
               settings.fruitjamVolumeLevel = std::min(23, settings.fruitjamVolumeLevel + 1);
               EXT_AUDIO_SETVOLUME(settings.fruitjamVolumeLevel);
#endif
            }
        }
        // if (p1 & UP) {
        //     if (pushed & SELECT) {
        //         loadSaveStateMenu = true;
        //         quickSaveAction = SaveStateTypes::LOAD;
        //     }
        //     if (pushed & START) {
        //         loadSaveStateMenu = true;
        //         quickSaveAction = SaveStateTypes::SAVE;
        //     }
        // }
        if (p1 & SELECT && !IsNSF)
        {
            if (pushed & START)
            {
                // saveNVRAM();
                // reset = true;
                FrensSettings::savesettings();
                showSettings = true;
            }
            if (pushed & A)
            {
               rapidFireMask[i] ^= io::GamePadState::Button::A;
               g_paceAUsed[i] = false; // fresh A-hold; a SELECT+A+L/R pace gesture will undo this toggle
            }
            if (pushed & B)
            {
                rapidFireMask[i] ^= io::GamePadState::Button::B;
                g_syncBUsed[i] = false; // fresh B-hold; a SELECT+B+L/R sync gesture will undo this toggle
            }
            if (pushed & UP)
            {
#if USE_ST7789
                scaleMode8_7_ = st7789_cycle_screen_mode(+1); // SELECT+UP: forward through the cycle order
#else
                scaleMode8_7_ = Frens::screenMode(-1);
#endif
            } else if (pushed & DOWN)
            {
#if USE_ST7789
                scaleMode8_7_ = st7789_cycle_screen_mode(-1); // SELECT+DOWN: backward through the cycle order
#else
                scaleMode8_7_ = Frens::screenMode(+1);
#endif
            } else if (pushed & LEFT)
            {
#if USE_ST7789
                if (i == 0 && (v & A)) // SELECT+A+LEFT = pace FASTER write (lower y), rotate tear toward horizontal
                {
                    if (!g_paceAUsed[i]) { rapidFireMask[i] ^= io::GamePadState::Button::A; g_paceAUsed[i] = true; } // cancel the A-press rapid-fire toggle
                    if (settings.dmaPaceY > ST7789_DMA_PACE_MIN) settings.dmaPaceY--;
                    st7789_set_dma_pace((uint16_t)settings.dmaPaceY);
                    g_osdKind = OSD_PACE;
                    g_osdFrames = ST7789_OSD_FRAMES;
                    g_osdSaveCountdown = ST7789_OSD_SAVE_FRAMES;
                    printf("dmaPaceY: %d\n", settings.dmaPaceY);
                }
                else if (i == 0 && (v & B)) // SELECT+B+LEFT = sync SLOWER (coarse 10us), park the seam
                {
                    if (!g_syncBUsed[i]) { rapidFireMask[i] ^= io::GamePadState::Button::B; g_syncBUsed[i] = true; } // cancel the B-press rapid-fire toggle
                    settings.syncSpeedAdj -= ST7789_SYNC_ADJ_STEP;
                    if (settings.syncSpeedAdj < ST7789_SYNC_ADJ_MIN) settings.syncSpeedAdj = ST7789_SYNC_ADJ_MIN;
                    st7789_match_audio_clock(); // keep audio locked to the new frame rate
                    g_osdKind = OSD_SYNC;
                    g_osdFrames = ST7789_OSD_FRAMES;
                    g_osdSaveCountdown = ST7789_OSD_SAVE_FRAMES;
                    printf("syncSpeedAdj: %d\n", settings.syncSpeedAdj);
                }
                // SELECT+LEFT = volume down (step 1). No DVI audio path on this build, so
                // the audio-output toggle used on other boards is repurposed for I2S gain.
                else if (i == 0 && settings.audioGain > ST7789_AUDIO_GAIN_MIN)
                {
                    settings.audioGain--;
                    g_osdKind = OSD_VOL;
                    g_osdFrames = ST7789_OSD_FRAMES;
                    g_osdSaveCountdown = ST7789_OSD_SAVE_FRAMES;
                    printf("Audio gain: %d\n", settings.audioGain);
                }
#else
                // Toggle audio output, ignore if HSTX is enabled, because HSTX must use external audio
#if EXT_AUDIO_IS_ENABLED && !HSTX
                settings.flags.useExtAudio = !settings.flags.useExtAudio;
                if (settings.flags.useExtAudio)
                {
                    printf("Using I2S Audio\n");
                }
                else
                {
                    printf("Using DVIAudio\n");
                }

#else
                settings.flags.useExtAudio = 0;
#endif
                //FrensSettings::savesettings();
#endif // USE_ST7789
            }
#if USE_ST7789
            else if (pushed & RIGHT)
            {
                if (i == 0 && (v & A)) // SELECT+A+RIGHT = pace SLOWER write (higher y), rotate tear toward vertical
                {
                    if (!g_paceAUsed[i]) { rapidFireMask[i] ^= io::GamePadState::Button::A; g_paceAUsed[i] = true; } // cancel the A-press rapid-fire toggle
                    if (settings.dmaPaceY < ST7789_DMA_PACE_MAX) settings.dmaPaceY++;
                    st7789_set_dma_pace((uint16_t)settings.dmaPaceY);
                    g_osdKind = OSD_PACE;
                    g_osdFrames = ST7789_OSD_FRAMES;
                    g_osdSaveCountdown = ST7789_OSD_SAVE_FRAMES;
                    printf("dmaPaceY: %d\n", settings.dmaPaceY);
                }
                else if (i == 0 && (v & B)) // SELECT+B+RIGHT = sync FASTER (coarse 10us), park the seam
                {
                    if (!g_syncBUsed[i]) { rapidFireMask[i] ^= io::GamePadState::Button::B; g_syncBUsed[i] = true; } // cancel the B-press rapid-fire toggle
                    settings.syncSpeedAdj += ST7789_SYNC_ADJ_STEP;
                    if (settings.syncSpeedAdj > ST7789_SYNC_ADJ_MAX) settings.syncSpeedAdj = ST7789_SYNC_ADJ_MAX;
                    st7789_match_audio_clock();
                    g_osdKind = OSD_SYNC;
                    g_osdFrames = ST7789_OSD_FRAMES;
                    g_osdSaveCountdown = ST7789_OSD_SAVE_FRAMES;
                    printf("syncSpeedAdj: %d\n", settings.syncSpeedAdj);
                }
                // SELECT+RIGHT = volume up (step 1).
                else if (i == 0 && settings.audioGain < ST7789_AUDIO_GAIN_MAX)
                {
                    settings.audioGain++;
                    g_osdKind = OSD_VOL;
                    g_osdFrames = ST7789_OSD_FRAMES;
                    g_osdSaveCountdown = ST7789_OSD_SAVE_FRAMES;
                    printf("Audio gain: %d\n", settings.audioGain);
                }
            }
#elif ENABLE_VU_METER
            else if (pushed & RIGHT)
            {
                settings.flags.enableVUMeter = !settings.flags.enableVUMeter;
                //FrensSettings::savesettings();
                // printf("VU Meter %s\n", settings.flags.enableVUMeter ? "enabled" : "disabled");
                turnOffAllLeds();
            }
#endif
        }

        prevButtons[i] = v;
    }

    /* NSF track controls (player 1 only, checked after button state update) */
    if (IsNSF)
    {
        static constexpr int LEFT = 1 << 6;
        static constexpr int RIGHT = 1 << 7;
        static constexpr int START = 1 << 3;
        static constexpr int SELECT = 1 << 2;
        static constexpr int A_BTN = 1 << 0;
        static constexpr int B_BTN = 1 << 1;

        /* Recalculate pushed for player 1: edges since last frame. */
        static DWORD nsfPrevPad = 0;
        DWORD nsfPushed = prevButtons[0] & ~nsfPrevPad;
        nsfPrevPad = prevButtons[0];

        if (!(prevButtons[0] & START) && !(prevButtons[0] & SELECT))
        {
            if (nsfPushed & RIGHT)
            {
                nsfNextTrack();
                /* Trigger a re-init by resetting the CPU state */
                resetGame = true;
            }
            else if (nsfPushed & LEFT)
            {
                nsfPrevTrack();
                resetGame = true;
            }
        }

        /* A = play, B = stop */
        if (nsfPushed & A_BTN)
            nsfStartPlayback();
        if (nsfPushed & B_BTN)
            nsfStopPlayback();

        if ((prevButtons[0] & SELECT) && (nsfPushed & START))
        {
            /* Select+Start: exit NSF playback, return to menu */
            reset = true;
        }
    }

    if (reset && !IsNSF)
    {
        saveNVRAM();
    }
    *pdwSystem = (reset || resetGame) ? PAD_SYS_QUIT : 0;
}

void InfoNES_MessageBox(const char *pszMsg, ...)
{
    printf("[MSG]");
    va_list args;
    va_start(args, pszMsg);
    vprintf(pszMsg, args);
    va_end(args);
    printf("\n");
}

void InfoNES_Error(const char *pszMsg, ...)
{
    printf("[Error]");
    va_list args;
    va_start(args, pszMsg);
    vsnprintf(ErrorMessage, ERRORMESSAGESIZE, pszMsg, args);
    printf("%s", ErrorMessage);
    va_end(args);
    printf("\n");
}
bool parseROM(const uint8_t *nesFile)
{
#if PICO_RP2350
    // Famicom Disk System dispatch. The disk image was loaded into memory
    // (PSRAM or flash); look up its size from the file on SD so we can
    // determine side count and strip any fwNES header.
    if (fdsIsFdsFilename(romName))
    {
        FILINFO fno;
        if (f_stat(romName, &fno) != FR_OK)
        {
            snprintf(ErrorMessage, ERRORMESSAGESIZE, "Cannot stat FDS file %s", romName);
            printf("%s\n", ErrorMessage);
            return false;
        }
        if (!fdsParse((BYTE *)nesFile, (size_t)fno.fsize))
        {
            // fdsParse already populated ErrorMessage via InfoNES_Error.
            return false;
        }
        // Disk image lives in memory at nesFile; PRG/CHR-RAM live in dedicated
        // FDS_* buffers. ROM/VROM are wired up by Mapper 20 init (phase 3).
        ROM = nullptr;
        VROM = nullptr;
        // Phase 5: expose FDS options in the in-game settings menu.
        g_settings_visibility_nes[MOPT_FDS_DISK_SWAP] = 1;
        g_settings_visibility_nes[MOPT_AUTO_SWAP_FDS_DISK] = 1;
        g_settings_visibility_nes[MOPT_AUTO_INSERT_FDS_DISK_A] = 1;
        menuSetFdsHooks(&fdsMenuHooks);
        return true;
    }
#endif

    // NSF (Nintendo Sound Format) detection — check magic or file extension.
    if (checkNSFMagic(nesFile))
    {
        // If already in NSF mode (e.g. track change via resetGame), skip re-parse
        // so that NsfCurrentTrack is preserved.
        if (IsNSF)
            return true;

        // Determine file size via f_stat (works for both SD and flash-based files).
        size_t fileSize = 0;
        FILINFO fno;
        if (f_stat(romName, &fno) == FR_OK)
            fileSize = (size_t)fno.fsize;
        if (fileSize == 0)
        {
            snprintf(ErrorMessage, ERRORMESSAGESIZE, "Cannot determine NSF file size");
            return false;
        }
        if (!nsfParse(nesFile, fileSize))
        {
            snprintf(ErrorMessage, ERRORMESSAGESIZE, "NSF parse error");
            return false;
        }
        ROM = nullptr;
        VROM = nullptr;
        return true;
    }

    memcpy(&NesHeader, nesFile, sizeof(NesHeader));
    if (!checkNESMagic(NesHeader.byID))
    {
        return false;
    }

    nesFile += sizeof(NesHeader);

    memset(SRAM, 0, SRAM_SIZE);

    if (NesHeader.byInfo1 & 4)
    {
        memcpy(&SRAM[0x1000], nesFile, 512);
        nesFile += 512;
    }

    auto romSize = NesHeader.byRomSize * 0x4000;
    auto vromSize = NesHeader.byVRomSize * 0x2000;

    // Detect ROMs where the header overstates PRG size and CHR data is
    // embedded inside the declared PRG area (e.g. Galaxian (J) has 8KB PRG +
    // 8KB CHR packed as 16KB with a header claiming 16KB PRG + 8KB CHR).
    // The reset vector at the end of the declared PRG will be invalid while
    // the vector at (romSize - vromSize) is valid.
    if (romSize > 0 && vromSize > 0 && romSize > vromSize)
    {
        uint16_t resetVec = nesFile[romSize - 4] | ((uint16_t)nesFile[romSize - 3] << 8);
        if (resetVec < 0x8000)
        {
            auto actualPrgSize = romSize - vromSize;
            uint16_t fixedResetVec = nesFile[actualPrgSize - 4] |
                                     ((uint16_t)nesFile[actualPrgSize - 3] << 8);
            if (fixedResetVec >= 0x8000)
            {
                printf("ROM header fix: PRG %dK -> %dK, CHR found at PRG offset %d\n",
                       romSize / 1024, actualPrgSize / 1024, actualPrgSize);
                ROM = (BYTE *)nesFile;
                VROM = (BYTE *)(nesFile + actualPrgSize);
                NesHeader.byRomSize = actualPrgSize / 0x4000;
                return true;
            }
        }
    }

    ROM = (BYTE *)nesFile;
    nesFile += romSize;

    if (NesHeader.byVRomSize > 0)
    {
        VROM = (BYTE *)nesFile;
        nesFile += vromSize;
    }

    return true;
}

void InfoNES_ReleaseRom()
{
    ROM = nullptr;
    VROM = nullptr;
    if (IsNSF)
    {
        /* On a track-change reset (resetGame), keep NSF state alive so
           the current track number is preserved across the re-init. */
        if (!resetGame)
            nsfRelease();
        return;
    }
#if PICO_RP2350
    if (IsFDS)
    {
        fdsRelease();
        IsFDS = false;
        g_settings_visibility_nes[MOPT_FDS_DISK_SWAP] = 0;
        menuSetFdsHooks(nullptr);
    }
#endif
}

void InfoNES_SoundInit()
{
}

int InfoNES_SoundOpen(int samples_per_sync, int sample_rate)
{
    printf("InfoNES_SoundOpen: samples_per_sync=%d, sample_rate=%d\n", samples_per_sync, sample_rate);
    return 0;
}

void InfoNES_SoundClose()
{
}

int __not_in_flash_func(InfoNES_GetSoundBufferSize)()
{
    // Prefer early return to avoid duplicated branches.
#if EXT_AUDIO_IS_ENABLED
    if (settings.flags.useExtAudio)
    {
        return audio_i2s_get_freebuffer_size();
    }
#endif

#if HSTX
    // Compute free HDMI Data Island audio packet capacity and convert to samples.
    int level = hstx_di_queue_get_level();
    int free_packets = HSTX_AUDIO_DI_HIGH_WATERMARK - level;
    if (free_packets <= 0)
        return 0;
    // Each DI packet carries 4 audio samples; use shift for fast multiply.
    return free_packets << 2;
#elif USE_ST7789
    // ST7789 backend: no DVI audio ring. I2S audio is added in a later phase.
    return 0;
#else
    // Non-HSTX path: return available ring buffer capacity directly.
    return dvi_->getAudioRingBuffer().getFullWritableSize();
#endif
}



static inline int16_t apply_dvi_gain_i32(int x)
{
    int64_t v = (int64_t)x * (int64_t)g_dvi_audio_gain_q8; // Q8 scale
    v >>= 8;
    if (v > 32767) v = 32767;
    else if (v < -32768) v = -32768;
    return (int16_t)v;
}

static inline int16_t apply_record_gain_i32(int x)
{
    int64_t v = (int64_t)x * (int64_t)g_record_gain_q8; // Q8 scale
    v >>= 8;
    if (v > 32767) v = 32767;
    else if (v < -32768) v = -32768;
    return (int16_t)v;
}

static inline void set_dviaudio_gain_q8(int q8)
{
    if (q8 < 0) q8 = 0;
    if (q8 > 1024) q8 = 1024; // up to 4.0x
    g_dvi_audio_gain_q8 = q8;
}
static inline void recordSampleToSoundRecorder(int l, int r)
{
#if PICO_RP2350
    if (SoundRecorder::isRecording())
    {
        // int16_t cl = (l > 32767 ? 32767 : (l < -32768 ? -32768 : l));
        // int16_t cr = (r > 32767 ? 32767 : (r < -32768 ? -32768 : r));
        int16_t cl = apply_record_gain_i32(l);
        int16_t cr = apply_record_gain_i32(r);
        int16_t stereo[2] = {cl, cr};
        SoundRecorder::recordFrame(stereo, 2);
    }
#endif
}

void __not_in_flash_func(InfoNES_SoundOutput)(int samples, BYTE *wave1, BYTE *wave2, BYTE *wave3, BYTE *wave4, BYTE *wave5, BYTE *wave6)
{
#if !HSTX
#if EXT_AUDIO_IS_ENABLED
    if (settings.flags.useExtAudio)
    {
        for (int i = 0; i < samples; ++i)
        {
            int w1 = wave1[i];
            int w2 = wave2[i];
            int w3 = wave3[i];
            int w4 = wave4[i];
            int w5 = wave5[i];
            int w6 = wave6 ? wave6[i] : 0;

            int l = w1 * 6 + w2 * 3 + w3 * 5 + w4 * 3 * 17 + w5 * 2 * 40 + w6 * 18;
            int r = w1 * 3 + w2 * 6 + w3 * 5 + w4 * 3 * 17 + w5 * 2 * 40 + w6 * 18;
#if PICO_RP2350
            recordSampleToSoundRecorder(l, r);
#endif
#if USE_ST7789
            // The NES APU mix is very low level (≈0..2445 of 32767). Scale it up
            // toward full 16-bit range so it isn't buried in DAC/amp noise. Clamp.
            {
                int gl = l * settings.audioGain; if (gl > 32767) gl = 32767;
                int gr = r * settings.audioGain; if (gr > 32767) gr = 32767;
                EXT_AUDIO_ENQUEUE_SAMPLE(gl, gr);
            }
#else
            EXT_AUDIO_ENQUEUE_SAMPLE(l, r);
#endif
#if ENABLE_VU_METER
            if (settings.flags.enableVUMeter)
            {
                addSampleToVUMeter(l);
            }
#endif
        }
        return;
    }
#endif
#if USE_ST7789
    // ST7789 backend: no DVI audio ring. I2S audio is added in a later phase.
    (void)samples; (void)wave1; (void)wave2; (void)wave3; (void)wave4; (void)wave5; (void)wave6;
#else
    while (samples)
    {
        auto &ring = dvi_->getAudioRingBuffer();
        auto n = std::min<int>(samples, ring.getWritableSize());
        if (!n)
        {
            return;
        }

        auto p = ring.getWritePointer();

        int ct = n;
        while (ct--)
        {
            int w1 = *wave1++;
            int w2 = *wave2++;
            int w3 = *wave3++;
            int w4 = *wave4++;
            int w5 = *wave5++;
            int w6 = wave6 ? *wave6++ : 0;

            int l = w1 * 6 + w2 * 3 + w3 * 5 + w4 * 3 * 17 + w5 * 2 * 40 + w6 * 18;
            int r = w1 * 3 + w2 * 6 + w3 * 5 + w4 * 3 * 17 + w5 * 2 * 40 + w6 * 18;
#if PICO_RP2350
            recordSampleToSoundRecorder(l, r);
#endif
            l = apply_dvi_gain_i32(l);
            r = apply_dvi_gain_i32(r);
            *p++ = {static_cast<short>(l), static_cast<short>(r)};
#if ENABLE_VU_METER
            if (settings.flags.enableVUMeter)
            {
                addSampleToVUMeter(l);
            }
#endif
        }

        ring.advanceWritePointer(n);
        samples -= n;
    }
#endif // USE_ST7789
#else
#if EXT_AUDIO_IS_ENABLED
    bool audioJackConnected = Frens::isHeadPhoneJackConnected();
#endif
    for (int i = 0; i < samples; ++i)
    {
        int w1 = wave1[i];
        int w2 = wave2[i];
        int w3 = wave3[i];
        int w4 = wave4[i];
        int w5 = wave5[i];
          /* w6: expansion audio 
                - VRC6 (Konami Mapper 24)
                - Famicom Disk System (Mapper 20)  
                - Sunsoft 5B (Mapper 69)
                - null when no expansion cart is loaded. */
        int w6 = wave6 ? wave6[i] : 0;

        // Mix your channels to a 12-bit value (example mix, adjust as needed)
        // This works but some effects are silent:
        // int sample12 =  (w1 + w2 + w3 + w4 + w5); // Range depends on input
        // Below is a more complex mix that gives a better sound

        int l = w1 * 6 + w2 * 3 + w3 * 5 + w4 * 3 * 17 + w5 * 2 * 40 + w6 * 18;
        int r = w1 * 3 + w2 * 6 + w3 * 5 + w4 * 3 * 17 + w5 * 2 * 40 + w6 * 18;
        const int l0 = l;
        const int r0 = r;

    #if PICO_RP2350
        recordSampleToSoundRecorder(l0, r0);
    #endif
    #if ENABLE_VU_METER
        if (settings.flags.enableVUMeter)
        {
            addSampleToVUMeter(l0);
        }
    #endif

    #if EXT_AUDIO_IS_ENABLED
        if (settings.flags.useExtAudio || audioJackConnected)
        {        
            EXT_AUDIO_ENQUEUE_SAMPLE(l0, r0);
            continue;
        }
    #endif
        
        int gl = apply_dvi_gain_i32(l0);
        int gr = apply_dvi_gain_i32(r0);
        hstx_push_audio_sample(gl, gr);
        
        // outBuffer[outIndex++] = sample8;
    }
#endif
}

extern WORD PC;

// Region-aware frame pacing.
//   NTSC: forwards to Frens::PaceFrames60fps (preserves the existing HSTX
//         vsync-wait and non-HSTX vsync/line-buffer behavior verbatim).
//   PAL : 50 Hz pacing via sleep_until. The HDMI/DVI panel still scans at
//         60 Hz, so 1 in every 6 displayed frames will be a duplicate; the
//         emulator itself runs at correct PAL speed. Works on HSTX (replaces
//         hstx_paceFrame's 60 Hz vsync wait) and on non-HSTX framebuffer
//         mode (replaces the vsync busy-wait). PAL is rejected upstream
//         when no framebuffer is available — see fallback at the call site.
static void paceFrame(bool init)
{
    if (!InfoNES_IsPal())
    {
        Frens::PaceFrames60fps(init);
        return;
    }

    static absolute_time_t next_frame_time;
    static bool initialized = false;
    if (init || !initialized)
    {
        next_frame_time = make_timeout_time_us(0);
        initialized = true;
    }

    // Slack-aware: if we overran the target by more than one frame (e.g.
    // returning from the menu after an idle pause), snap forward instead of
    // bursting through the backlog. Mirrors hstx_paceFrame's resync logic.
    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(now, next_frame_time) <= -20000)
    {
        next_frame_time = now;
    }

    sleep_until(next_frame_time);
    next_frame_time = delayed_by_us(next_frame_time, 20000); // 1/50s = 20000us
#if DOUBLEFRAMEBUFFER
    Frens::swapFrameBuffers();
#endif
}

int InfoNES_LoadFrame()
{
#if USE_ST7789
    // Pace the emulator to 60fps so game speed and audio rate stay correct (the
    // display runs in parallel on core1). If a frame ran long, don't wait (catch
    // up); resync after a long stall (menu / state load). The period is nudged by
    // settings.syncSpeedAdj (SELECT+RIGHT faster / LEFT slower) to match the panel's
    // free-running refresh and park the tear off-screen (anti-tearing sync tuner).
    {
        // syncSpeedAdj is in TENTHS of a us. Accumulate the fractional part and only ever add
        // whole us to the deadline, carrying the remainder -> the frame period DITHERS between
        // adjacent whole-us values so its AVERAGE hits the sub-us target. That parks the seam at
        // a fractional sweet spot the 1us pacer couldn't reach; the ±0.5us jitter is invisible.
        static uint64_t deadline = 0;
        static int32_t  frac_tenths = 0;
        uint64_t now = time_us_64();
        if (deadline == 0) deadline = now;
        frac_tenths += (166670 - settings.syncSpeedAdj);   // frame period in tenths-of-us
        uint32_t whole_us = (uint32_t)(frac_tenths / 10);
        frac_tenths -= (int32_t)(whole_us * 10);            // carry 0..9 tenths to next frame
        deadline += whole_us;
        if (now < deadline)
            while (time_us_64() < deadline) tight_loop_contents();
        else if (now > deadline + 50000)
            { deadline = time_us_64(); frac_tenths = 0; }
    }

    // Volume control housekeeping (once per frame): fade out the on-screen
    // indicator, and persist a changed gain to the settings file once the user
    // has stopped adjusting (debounced, so we don't write the SD every press).
    if (g_osdFrames > 0)
        g_osdFrames--;
    if (g_osdSaveCountdown > 0 && --g_osdSaveCountdown == 0)
        FrensSettings::savesettings();
#endif
//      if (pendingLoadState) {         // perform at frame start
//         pendingLoadState = false;
//         printf("Loading state...\n");
//         if (Emulator_LoadState("/slot0.state") == 0) {
//             printf("State loaded.\n");
// #if FRAMEBUFFERISPOSSIBLE
//             if (Frens::isFrameBufferUsed()) {
//                 memset(Frens::framebuffer, 0, sizeof(Frens::framebuffer));
//             }
// #endif
//         } else {
//             printf("State load failed.\n");
//         }
//     }
    paceFrame(false);
    //Frens::waitForVSync();
    Frens::pollHeadPhoneJack();
    EXT_AUDIO_POLL_HEADPHONE();

    /* NSF: update VU meter levels once per frame */
    if (IsNSF)
    {
        nsfUpdateVuLevels();
        /* Check for auto-advance (silence detection / max duration) */
        if (nsfUpdatePlayback())
            resetGame = true;
    }
#if NES_PIN_CLK != -1
    nespad_read_start();
#endif
    auto count =
#if !HSTX && !USE_ST7789
        dvi_->getFrameCounter();
#elif USE_ST7789
        []{ static uint32_t fc = 0; return fc++; }();
#else
        hstx_getframecounter();
#endif
    long onOff = hw_divider_s32_quotient_inlined(count, 60) & 1;
    Frens::blinkLed(onOff);
#if NES_PIN_CLK != -1
    nespad_read_finish(); // Sets global nespad_state var
#endif
#if USE_ST7789
    gpio_buttons_update(); // sample the GP2..GP9 buttons into gamepad 0
#endif
#if ST7789_USB_DEVICE
    tud_task();            // pump the USB CDC device stack (serial diagnostics)
#else
    tuh_task();            // pump the USB host stack (USB controller = player 2)
#endif
    // Frame rate calculation
    if (settings.flags.displayFrameRate)
    {
        // calculate fps and round to nearest value (instead of truncating/floor)
        uint32_t tick_us = Frens::time_us() - start_tick_us;
        fps = (1000000 - 1) / tick_us + 1;
        start_tick_us = Frens::time_us();
    }

#if ST7789_USB_DEVICE
    // DIAG: accumulate per-frame stats and print a summary once/sec. This shows
    // whether OUR output is the problem (jitter / dropped frames => fixable) or
    // whether we are dead-steady at 60.0 fps and the "shifty" motion is purely the
    // panel's free-running refresh beating against us (=> needs frame-rate matching
    // or a TE wire). Run while playing a SCROLLING game so the load is realistic.
    //   fps      : frames counted in the last second (want ~60, rock steady)
    //   avg/min/max/jitter : core0 frame interval in us (want ~16667, tight spread)
    //   late     : frames whose interval exceeded 17000us (a visible hitch each)
    //   c1_disp  : us/sec core1 was actually driving the display (1e6 - idle); the
    //              true per-frame display cost = c1_disp/fps. Lower = more headroom
    //              for a wider (aspect-correct) image. Measured WITHOUT per-line
    //              timer reads, so it reflects the real (shipping-build) cost.
    {
        static uint64_t win_start = 0, prev = 0;
        static uint32_t frames = 0, late = 0, mn = 0xffffffff, mx = 0, sum = 0;
        static uint32_t idle_prev = 0, block_prev = 0;
        uint64_t now = time_us_64();
        if (prev != 0)
        {
            uint32_t dt = (uint32_t)(now - prev);
            frames++; sum += dt;
            if (dt < mn) mn = dt;
            if (dt > mx) mx = dt;
            if (dt > 17000) late++;
        }
        prev = now;
        if (win_start == 0) win_start = now;
        uint32_t win = (uint32_t)(now - win_start);
        if (win >= 1000000)
        {
            uint32_t idle_now = st7789_core1_idle_us;
            uint32_t idle_d = idle_now - idle_prev;
            uint32_t disp = (idle_d < win) ? (win - idle_d) : 0; /* active us over the window */
            uint32_t block_now = st7789_acquire_block_us;
            uint32_t c0block = block_now - block_prev; /* us core0 waited on core1 */
            uint32_t avg = frames ? sum / frames : 0;
            // ScreenMode enum order: 0=SCANLINE_8_7 1=NOSCANLINE_8_7 2=SCANLINE_1_1 3=NOSCANLINE_1_1
            static const char *kModeName[4] = { "8:7+SL", "8:7", "1:1+SL", "1:1" };
            int modeIdx = (int)settings.screenMode & 3;
            printf("diag: %lu fps mode=%s pace=%d c1_disp=%lu/f c0block=%lu/f  calib=%lu us/%u words  clk_peri=%lu Hz  spi_baud=%lu Hz\n",
                   (unsigned long)frames, kModeName[modeIdx], (int)settings.dmaPaceY,
                   (unsigned long)(frames ? disp / frames : 0),
                   (unsigned long)(frames ? c0block / frames : 0),
                   (unsigned long)st7789_calib_us, (unsigned)(ST7789_OUT_WIDTH * 32u),
                   (unsigned long)st7789_calib_clkperi, (unsigned long)st7789_calib_baud);
            idle_prev = idle_now; block_prev = block_now;
            win_start = now; frames = 0; late = 0; mn = 0xffffffff; mx = 0; sum = 0;
        }
    }
#endif

#if !HSTX
#else
    // hstx_waitForVSync();
#endif
#if WII_PIN_SDA >= 0 and WII_PIN_SCL >= 0
    // Poll Wii pad once per frame (function called once per rendered frame)
    wiipad_raw_cached = wiipad_read();
#endif
#if ENABLE_VU_METER
    if (isVUMeterToggleButtonPressed())
    {
        settings.flags.enableVUMeter = !settings.flags.enableVUMeter;
        FrensSettings::savesettings();
        // printf("VU Meter %s\n", settings.flags.enableVUMeter ? "enabled" : "disabled");
        turnOffAllLeds();
    }
#endif
   
    if (showSettings && !IsNSF)
    {
        showSettings = false;
#if USE_ST7789
        st7789_ring_flush();            // quiesce core1 so core0 can draw the overlay
        st7789_set_color_mode(false);   // overlay uses the menu's 16-bit RGB565 path
#endif
        int rval = showSettingsMenu(true);
#if USE_ST7789
        st7789_set_color_mode(true);    // back to 12-bit for the game
#endif
        if (rval == 3)
        {
            reset = true;
            if (isAutoSaveStateConfigured() ){
                loadSaveStateMenu = true;
                quickSaveAction = SaveStateTypes::SAVE_AND_EXIT;
            }
        }
        if ( rval == 4) {
            loadSaveStateMenu = true;
            quickSaveAction = SaveStateTypes::NONE;
           
        }
        if (rval == 5) {
           resetGame = true;
        }
    }
    if (loadSaveStateMenu && !IsNSF) {
        if (quickSaveAction == SaveStateTypes::LOAD_AND_START) {
            if (framesbeforeAutoStateIsLoaded > 0) {
                --framesbeforeAutoStateIsLoaded;  // let the emulator run for a few frames before loading state
            }   
        }  else {
            framesbeforeAutoStateIsLoaded = 0;
        } 
        if (framesbeforeAutoStateIsLoaded == 0) {
           
            char msg[24];
            snprintf(msg, sizeof(msg), "Mapper %03d CRC %08X", MapperNo, Frens::getCrcOfLoadedRom());
#if USE_ST7789
            st7789_ring_flush();            // quiesce core1 so core0 can draw the overlay
            st7789_set_color_mode(false);   // overlay uses the menu's 16-bit RGB565 path
#endif
            bool kept = showSaveStateMenu(Emulator_SaveState, Emulator_LoadState, msg, quickSaveAction);
#if USE_ST7789
            st7789_set_color_mode(true);    // back to 12-bit for the game
#endif
            if ( kept == false ) {
                reset = true;
            };
            loadSaveStateMenu = false;
        }
    }

    return count;
}

namespace
{
#if !HSTX
#if USE_ST7789
    WORD *currentLineBuffer_{nullptr};
    WORD *currentLineBuf{nullptr};
#else
    dvi::DVI::LineBuffer *currentLineBuffer_{};
    WORD *currentLineBuf{nullptr};
#endif
#else
    WORD *currentLineBuffer_{nullptr};
#endif
}
#if !HSTX
void __not_in_flash_func(drawWorkMeterUnit)(int timing,
                                            [[maybe_unused]] int span,
                                            uint32_t tag)
{
    if (timing >= 0 && timing < 640)
    {
#if USE_ST7789
        WORD *p = currentLineBuffer_;
#else
        auto p = currentLineBuffer_->data();
#endif
        p[timing] = tag; // tag = color
    }
}

void __not_in_flash_func(drawWorkMeter)(int line)
{
    if (!currentLineBuffer_)
    {
        return;
    }

#if USE_ST7789
    WORD *lb = currentLineBuffer_;
#else
    WORD *lb = currentLineBuffer_->data();
#endif
    memset(lb, 0, 64);
    memset(&lb[320 - 32], 0, 64);
    lb[160] = 0;
    if (line == 4)
    {
        for (int i = 1; i < 10; ++i)
        {
            lb[16 * i] = 31;
        }
    }

    constexpr uint32_t clocksPerLine = 800 * 10;
    constexpr uint32_t meterScale = 160 * 65536 / (clocksPerLine * 2);
    util::WorkMeterEnum(meterScale, 1, drawWorkMeterUnit);
    //    util::WorkMeterEnum(160, clocksPerLine * 2, drawWorkMeterUnit);
}
#endif

/*-------------------------------------------------------------------*/
/*  NSF display helper: draw a text string on a scanline             */
/*-------------------------------------------------------------------*/
static void nsfDrawText(WORD *buf, int x, int line, const char *text, int textRow, WORD fgc, WORD bgc)
{
    x+=2; // 2 pixel padding on the left
    for (int i = 0; text[i] != '\0'; i++)
    {
        char fontSlice = getcharslicefrom8x8font(text[i], textRow);
        for (int bit = 0; bit < 8; bit++)
        {
            buf[x + i * 8 + bit] = (fontSlice & 1) ? fgc : bgc;
            fontSlice >>= 1;
        }
    }
}

/*-------------------------------------------------------------------*/
/*  NSF display: render VU meter UI on a scanline                    */
/*                                                                   */
/*  Layout (320 pixels wide, lines 4-235):                           */
/*   Lines  20-27:  Song name                                        */
/*   Lines  36-43:  Artist name                                      */
/*   Lines  52-59:  Copyright                                        */
/*   Lines  72-79:  Track N / Total                                  */
/*   Lines  96-103: "Pulse 1" bar label                              */
/*   Lines 104-111: Pulse 1 VU bar                                   */
/*   Lines 116-123: "Pulse 2" bar label                              */
/*   Lines 124-131: Pulse 2 VU bar                                   */
/*   Lines 136-143: "Triangle" bar label                             */
/*   Lines 144-151: Triangle VU bar                                  */
/*   Lines 156-163: "Noise" bar label                                */
/*   Lines 164-171: Noise VU bar                                     */
/*   Lines 176-183: "DPCM" bar label                                 */
/*   Lines 184-191: DPCM VU bar                                      */
/*-------------------------------------------------------------------*/
static void nsfRenderLine(WORD *buf, int line)
{
    /* NES palette indices for colors */
    WORD bgc = NesPalette[0x0F];   /* Black */
    WORD fgc = NesPalette[0x30];   /* White */

    /* Fill background */
    for (int i = 0; i < 320; i++)
        buf[i] = bgc;

    /* Channel bar colors (NES palette) */
    static const BYTE barColors[5] = { 0x16, 0x12, 0x1A, 0x14, 0x17 };
    static const char *chanNames[5] = { "Pulse 1", "Pulse 2", "Triangle", "Noise", "DPCM" };

    /* Song name (lines 20-27) */
    if (line >= 20 && line < 28)
    {
        int textRow = line - 20;
        nsfDrawText(buf, 32, line, NsfHeader.szSongName, textRow, fgc, bgc);
    }
    /* Artist (lines 36-43) */
    else if (line >= 36 && line < 44)
    {
        int textRow = line - 36;
        nsfDrawText(buf, 32, line, NsfHeader.szArtistName, textRow, NesPalette[0x21], bgc);
    }
    /* Copyright (lines 52-59) */
    else if (line >= 52 && line < 60)
    {
        int textRow = line - 52;
        nsfDrawText(buf, 32, line, NsfHeader.szCopyright, textRow, NesPalette[0x21], bgc);
    }
    /* Track info (lines 72-79) */
    else if (line >= 72 && line < 80)
    {
        char trackStr[40];
        /* Show elapsed time MM:SS and play/stop status */
        int totalSec = NsfFrameCounter / 60;
        int mm = totalSec / 60;
        int ss = totalSec % 60;
        snprintf(trackStr, sizeof(trackStr), "Track %d / %d  %d:%02d %s",
                 NsfCurrentTrack + 1, NsfHeader.byTotalSongs,
                 mm, ss, NsfIsPlaying ? ">" : "||");
        int textRow = line - 72;
        nsfDrawText(buf, 32, line, trackStr, textRow, NesPalette[0x30], bgc);
    }
    /* VU bars: 5 channels, each occupies 20 lines (8 label + 8 bar + 4 gap) */
    else if (line >= 96 && line < 196)
    {
        for (int ch = 0; ch < 5; ch++)
        {
            int labelStart = 96 + ch * 20;
            int barStart = labelStart + 8;

            /* Channel label */
            if (line >= labelStart && line < labelStart + 8)
            {
                int textRow = line - labelStart;
                nsfDrawText(buf, 32, line, chanNames[ch], textRow, NesPalette[barColors[ch]], bgc);
            }
            /* VU bar */
            else if (line >= barStart && line < barStart + 8)
            {
                /* Bar width proportional to VU level (0-255 → 0-240 pixels) */
                int barWidth = (NsfVuLevels[ch] * 240) / 255;
                WORD barColor = NesPalette[barColors[ch]];
                for (int x = 40; x < 40 + barWidth && x < 280; x++)
                    buf[x] = barColor;
            }
        }
    }
    /* Progress bar (lines 210-215) */
    else if (line >= 210 && line < 216)
    {
        BYTE progress = nsfGetProgress();
        int barWidth = (progress * 240) / 255;
        WORD borderColor = NesPalette[0x10]; /* Dark grey */
        WORD fillColor = NesPalette[0x30];   /* White */

        /* Draw border on top/bottom lines, fill on interior */
        if (line == 210 || line == 215)
        {
            for (int x = 39; x <= 280; x++)
                buf[x] = borderColor;
        }
        else
        {
            buf[39] = borderColor;
            buf[280] = borderColor;
            for (int x = 40; x < 40 + barWidth && x < 280; x++)
                buf[x] = fillColor;
        }
    }
}

void __not_in_flash_func(InfoNES_PreDrawLine)(int line)
{
#if !HSTX

    WORD *buff;
// b.size --> 640
// printf("Pre Draw%d\n", b->size());
// WORD = 2 bytes
// b->size = 640
// printf("%d\n", b->size());
#if FRAMEBUFFERISPOSSIBLE
    if (Frens::isFrameBufferUsed())
    {
        currentLineBuf = &Frens::framebuffer[line * 320];
        InfoNES_SetLineBuffer(currentLineBuf + 32, 320);
    }
    else
    {
#endif
#if USE_ST7789
        // Acquire a ring slot and render the RAW NES scanline straight into it (no core0
        // scaling pass — that is what frees core0 to hit 60 fps). core1 scales the slot
        // 256->OUT_WIDTH (or, for NSF, converts its full-width UI) as it streams the line
        // to the panel. Side bars are cleared once at game start.
        currentLineBuffer_ = st7789_ring_acquire();
        st7789_set_nsf_mode(IsNSF);
        InfoNES_SetLineBuffer(currentLineBuffer_, ST7789_NES_WIDTH);
#else
        util::WorkMeterMark(0xaaaa);
        auto b = dvi_->getLineBuffer();
        util::WorkMeterMark(0x5555);
        InfoNES_SetLineBuffer(b->data() + 32, b->size());
        currentLineBuffer_ = b;
#endif
#if FRAMEBUFFERISPOSSIBLE
    }
#endif
    //    (*b)[319] = line + dvi_->getFrameCounter();

#else
    currentLineBuffer_ = hstx_getlineFromFramebuffer(line + 4); // Top Margin of 4 lines
    InfoNES_SetLineBuffer(currentLineBuffer_ + 32, 640);

#endif
}

void __not_in_flash_func(InfoNES_PostDrawLine)(int line)
{
#if !HSTX
#if !defined(NDEBUG)
    util::WorkMeterMark(0xffff);
    drawWorkMeter(line);
#endif
#endif

    /* NSF mode: overwrite scanline with VU meter UI */
    if (IsNSF)
    {
        WORD *buf =
#if !HSTX && !USE_ST7789
            currentLineBuf == nullptr ? currentLineBuffer_->data() : currentLineBuf;
#else
            currentLineBuffer_;
#endif
        nsfRenderLine(buf, line);
    }

    // Display frame rate
    if (settings.flags.displayFrameRate && line >= 8 && line < 16)
    {
        char fpsString[2];
        WORD *fpsBuffer =
#if USE_ST7789
            currentLineBuffer_ + 8;   // ring slot, NES column 8 (core1 scales it)
#elif !HSTX
            currentLineBuf == nullptr ? currentLineBuffer_->data() + 40 : currentLineBuf + 40;
#else
            currentLineBuffer_ + 40;
#endif
        WORD fgc = NesPalette[48];
        WORD bgc = NesPalette[15];
        fpsString[0] = '0' + (fps / 10);
        fpsString[1] = '0' + (fps % 10);

        int rowInChar = line % 8;
        for (auto i = 0; i < 2; i++)
        {
            char firstFpsDigit = fpsString[i];
            char fontSlice = getcharslicefrom8x8font(firstFpsDigit, rowInChar);
            for (auto bit = 0; bit < 8; bit++)
            {
                if (fontSlice & 1)
                {
                    *fpsBuffer++ = fgc;
                }
                else
                {
                    *fpsBuffer++ = bgc;
                }
                fontSlice >>= 1;
            }
        }
    }

#if USE_ST7789
    // Volume OSD: for a short while after a SELECT+LEFT/RIGHT change, draw "VOL n" near
    // the top of the picture (8x8 font, one 8px-tall band at rows 16..23). Written into
    // the ring slot in RGB555 (NesPalette) so core1 converts and packs it along with the
    // rest of the scanline.
    if (g_osdFrames > 0 && line >= 16 && line < 24)
    {
        char osdStr[16];
        int n;
        if (g_osdKind == OSD_SYNC)
        {
            int t = settings.syncSpeedAdj, at = t < 0 ? -t : t; // tenths-of-us -> signed N.n
            n = snprintf(osdStr, sizeof(osdStr), "SYNC %s%d.%d", t < 0 ? "-" : "", at / 10, at % 10);
        }
        else if (g_osdKind == OSD_PACE)
            n = snprintf(osdStr, sizeof(osdStr), "PACE %d", settings.dmaPaceY);
        else
            n = snprintf(osdStr, sizeof(osdStr), "VOL %d", settings.audioGain);
        WORD fgc = NesPalette[48];
        WORD bgc = NesPalette[15];
        WORD *p = currentLineBuffer_ + 8; // ring slot: NES column 8
        int rowInChar = line % 8;
        for (int i = 0; i < n; i++)
        {
            char slice = getcharslicefrom8x8font(osdStr[i], rowInChar);
            for (int bit = 0; bit < 8; bit++)
            {
                *p++ = (slice & 1) ? fgc : bgc;
                slice >>= 1;
            }
        }
    }
#endif

#if !HSTX
#if FRAMEBUFFERISPOSSIBLE
    if (!Frens::isFrameBufferUsed())
    {
#endif
        assert(currentLineBuffer_);
#if USE_ST7789
        st7789_ring_commit(line); // hand the raw line to core1 (it scales + streams it)
#else
        dvi_->setLineBuffer(line, currentLineBuffer_);
#endif
        currentLineBuffer_ = nullptr;
#if FRAMEBUFFERISPOSSIBLE
    }
#endif
#endif
}

bool loadAndReset()
{
    auto rom = romSelector_.getCurrentROM();
    if (!rom)
    {
        printf("ROM does not exists.\n");
        return false;
    }

    if (!parseROM(rom))
    {
        printf("NES file parse error.\n");
        return false;
    }
    if (loadNVRAM() == false)
    {
        return false;
    }

    if (InfoNES_Reset() < 0)
    {
        printf("NES reset error.\n");
        return false;
    }
    return true;
}

int InfoNES_Menu()
{
    // InfoNES_Main() のループで最初に呼ばれる
    return loadAndReset() ? 0 : -1;
    // return 0;
}

int main()
{
    char selectedRom[FF_MAX_LFN];

    romName = selectedRom;
    ErrorMessage[0] = selectedRom[0] = 0;

#if USE_ST7789
    // 292 MHz overclock for the wider 8:7 image needs a bit more core voltage than
    // the 280 MHz builds use; 1.25 V gives stability margin (still well under max).
    Frens::setClocksAndStartStdio(CPUFreqKHz, VREG_VOLTAGE_1_25);
#else
    Frens::setClocksAndStartStdio(CPUFreqKHz, VREG_VOLTAGE_1_20);
#endif

#if USE_ST7789
    // Source clk_peri from clk_sys so the display SPI0 and the SD-card spi1 both
    // run at full rate (the InfoNES RP2040 path leaves clk_peri capped). Done
    // HERE, before initAll, so the SD mounts at the final peripheral clock —
    // changing clk_peri after the SD mount would corrupt its SPI divisor and
    // break file reads.
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                    clock_get_hz(clk_sys), clock_get_hz(clk_sys));
#endif

#if USE_ST7789 && ST7789_TEST_PATTERN
    // EARLY bring-up test: runs immediately after clocks, BEFORE Frens::initAll,
    // to isolate the ST7789 driver from SD/USB/menu init. Blinks the onboard LED
    // (GP25) each colour so we can tell the loop is alive even if the panel is
    // not, and cycles solid colours through the real next_linebuf->send_line path.
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    st7789_init();
    {
        static const uint16_t cols[] = {0xF800, 0x07E0, 0x001F, 0xFFFF, 0xFFE0, 0x07FF, 0x0000};
        for (unsigned f = 0;; ++f)
        {
            gpio_put(25, f & 1);
            uint16_t c = cols[f % (sizeof(cols) / sizeof(cols[0]))];
            for (int row = 0; row < (int)ST7789_HEIGHT; ++row)
            {
                uint16_t *lb = st7789_next_linebuf();
                for (unsigned x = 0; x < ST7789_WIDTH; ++x)
                    lb[x] = c;
                st7789_send_line(row, lb);
            }
            st7789_wait_idle();
            sleep_ms(600);
        }
    }
#endif

    printf("==========================================================================================\n");
    printf("Pico-InfoNES+ %s\n", SWVERSION);
    printf("Build date: %s\n", __DATE__);
    printf("Build time: %s\n", __TIME__);
    printf("CPU freq: %d kHz\n", clock_get_hz(clk_sys) / 1000);
#if HSTX
    printf("HSTX freq: %d\n", clock_get_hz(clk_hstx) / 1000);
#endif
    printf("Stack size: %d bytes\n", PICO_STACK_SIZE);
    printf("==========================================================================================\n");
    printf("Starting up...\n");
#if PICO_RP2350
    printf("Mapper 5 and Mapper 85 are enabled\n");
#else
    printf("Mapper 5 and Mapper 85 are disabled\n");
#endif
    FrensSettings::initSettings(FrensSettings::emulators::NES);
    // Note:
    //     - When using framebuffer, AUDIOBUFFERSIZE must be increased to 1024
    //     - Top and bottom margins are reset to zero
    isFatalError = !Frens::initAll(selectedRom, CPUFreqKHz, 4, 4, AUDIOBUFFERSIZE, false, true);
#if USE_ST7789 && !ST7789_TEST_PATTERN
    // Bring up the ST7789 SPI panel (DVI is disabled in this build). Clocks are
    // already configured by setClocksAndStartStdio() above. (In ST7789_TEST_PATTERN
    // builds the panel is brought up earlier, before initAll.)
    st7789_init();
    gpio_buttons_init();
    st7789ApplyPaletteSaturation(); // one-time gentle palette saturation boost
    // Hand the real (saturated) palette to the driver as an index->RGB565 LUT, then switch
    // NesPalette to identity so InfoNES, the NSF UI and the OSD overlays all write 1-byte
    // palette INDICES into the ring (half the RAM of RGB555 => twice the scatter window).
    st7789_set_palette((const uint16_t *)NesPalette, 64);
    {
        WORD *pal = (WORD *)NesPalette;
        for (int i = 0; i < 64; i++)
            pal[i] = (WORD)i; // backdrop pixels still get |0x8000 in PalTable (sprite priority)
    }
#if ST7789_USB_DEVICE
    st7789_calibrate_spi(); // measure raw SPI throughput before core1 starts
#endif
    st7789_start_display_core1(); // core1 drives the panel; idle until a game commits lines
#endif
#if USE_ST7789
    // This hardware has no DVI/HDMI audio path — always route sound to the I2S DAC,
    // overriding any external-audio setting loaded from the SD card's settings file.
    settings.flags.useExtAudio = 1;
    // Clamp a persisted gain into the valid range (e.g. if the ceiling was lowered
    // since the value was saved) so the loaded volume can never exceed the maximum.
    if (settings.audioGain > ST7789_AUDIO_GAIN_MAX)
        settings.audioGain = ST7789_AUDIO_GAIN_MAX;
#endif

    scaleMode8_7_ = Frens::applyScreenMode(settings.screenMode);
#if USE_ST7789
    st7789_set_scanlines(ST7789_SCANLINES_ON()); // apply scanlines + aspect per the saved screen mode
    st7789_set_aspect(ST7789_ASPECT_1_1());
#endif
    bool showSplash = true;
#if PICO_RP2350
    g_settings_visibility_nes[MOPT_AUTO_SWAP_FDS_DISK] = 1;
    g_settings_visibility_nes[MOPT_AUTO_INSERT_FDS_DISK_A] = 1;
#else
    g_settings_visibility_nes[MOPT_AUTO_SWAP_FDS_DISK] =   0;
    g_settings_visibility_nes[MOPT_AUTO_INSERT_FDS_DISK_A] = 0;
#endif
    g_settings_visibility = g_settings_visibility_nes;
    g_available_screen_modes = g_available_screen_modes_nes;
    while (true)
    {
#if EMBEDDED_NES_ROM
        ROM_FILE_ADDR = (uintptr_t)embedded_nes_rom;
        strcpy(selectedRom, "Embedded");
        isFatalError = false; // SD card failure is not fatal when ROM is embedded
        *ErrorMessage = 0;
        Frens::PaceFrames60fps(true);
#else
        if (strlen(selectedRom) == 0)
        {
#if PICO_RP2350
            const char *romExtensions = ".nes .fds .nsf";
#else
            const char *romExtensions = ".nes .nsf";
#endif
            menu("Pico-InfoNES+", ErrorMessage, isFatalError, showSplash, romExtensions, selectedRom); // With no psram this never returns, but reboots upon selecting a game
            printf("Playing selected ROM from menu: %s\n", selectedRom);
          
        }
        if (Frens::getCrcOfLoadedRom() == 0x743387FF && !Frens::isPsramEnabled())
        { 
            // Lagrange Point  needs PSRAM for its memory requirements.
            strcpy(ErrorMessage, "Lagrange Point needs PSRAM to run.");
            selectedRom[0] = 0;
            continue;
        }
#endif
        reset = resetGame = loadSaveStateMenu = false;
        //EXT_AUDIO_MUTE_INTERNAL_SPEAKER(settings.flags.fruitJamEnableInternalSpeaker == 0);
        EXT_AUDIO_SETVOLUME(settings.fruitjamVolumeLevel);
        *ErrorMessage = 0;
        if (!Frens::isPsramEnabled())
        {
            printf("Now playing: %s\n", selectedRom);
        }
       
        if (isAutoSaveStateConfigured() && !IsNSF)
        {
            char tmpPath[40];
            getAutoSaveStatePath(tmpPath, sizeof(tmpPath));
            printf("Auto-save is configured found for this ROM (%s)\n", tmpPath);
            if (Frens::fileExists(tmpPath) ) {
                printf("Auto-save state found for this ROM (%s)\n", tmpPath);
                printf("Loading auto-save state...\n");
                loadSaveStateMenu = true;
                quickSaveAction = SaveStateTypes::LOAD_AND_START;
                framesbeforeAutoStateIsLoaded = 120; // wait 2 seconds before loading auto state
            } else {
                printf("No auto-save state found for this ROM.\n");
            }
        } else {
            printf("No auto-save configured for this ROM.\n");
        }
        do {
            resetGame = false;
#if PICO_RP2350
            if (fdsIsFdsFilename(romName))
            {
                romSelector_.initRaw(ROM_FILE_ADDR);
            }
            else
#endif
            if (checkNSFMagic(reinterpret_cast<const uint8_t *>(ROM_FILE_ADDR)))
            {
                romSelector_.initRaw(ROM_FILE_ADDR);
            }
            else if (!romSelector_.init(ROM_FILE_ADDR) ) {
                strcpy(ErrorMessage, "Not a NES ROM file.");
                break;
            }

            // isRomPal: 0 = NTSC, 1 = PAL, 2 = Dendy.
            int region = InfoNES_DetectRegion(ROM_FILE_ADDR, Frens::getCrcOfLoadedRom(), selectedRom);
            static const char *regionNames[] = { "NTSC", "PAL", "Dendy" };
            const char *regionName = regionNames[region & 3];

            // PAL/Dendy require a framebuffer-based video pipeline. In non-HSTX
            // line-streaming mode (RP2040, no framebuffer) the line-buffer
            // queue is hardware-locked to the DVI 60 Hz scanline rate, so
            // pacing the emulator at 50 Hz starves the queue (flicker) and
            // the audio ring buffer (silence). Force NTSC there.
#if !HSTX
            if (region != INFONES_REGION_NTSC && !Frens::isFrameBufferUsed())
            {
                printf("%s not supported in line-streaming mode (no framebuffer); running as NTSC.\n", regionName);
                region = INFONES_REGION_NTSC;
                regionName = regionNames[0];
            }
#endif
            printf("Region: %s\n", regionName);
            // After a non-PSRAM reboot the monitor needs time to sync with the
            // fresh HDMI signal.  Without a delay the FDS BIOS intro animation
            // plays while the display is still dark.  Only needed on the very
            // first launch (showSplash is true); resets keep the link up.
            // This also benefits RP2040/RP2350: .nsf files don't clip sound at the start, roms that 
            // start with sound also don't clip sound.
            if (showSplash && !Frens::isPsramEnabled())
            {
                showSplash = false;
                printf("Feeding blank frames for display sync...\n");
                menuPumpBlankFrames(180);
            }
            paceFrame(true); // reset pacing to avoid burst of frames if resetGame is true
#if USE_ST7789
            st7789_fill(0x0000); // clear the static side bars to black (game sends only the 256 centre)
            st7789_set_color_mode(true); // game streams 12-bit RGB444 (saves SPI/line vs the menu's 16-bit)
            st7789_match_audio_clock();  // lock the I2S DAC rate to the (seam-parked) frame rate
            st7789_set_dma_pace((uint16_t)settings.dmaPaceY); // apply the saved tear-angle pace

#endif
            InfoNES_Main(region);
#if USE_ST7789
            st7789_ring_flush(); // drain core1's in-flight lines before core0 redraws
            st7789_set_color_mode(false); // back to 16-bit RGB565 for the menu
#endif

        } while (resetGame);
#if !EMBEDDED_NES_ROM
        selectedRom[0] = 0;
#endif
        showSplash = false;
    }

    return 0;
}
