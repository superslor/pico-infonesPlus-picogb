#pragma once
// Screensaver image format. The DVI/TMDS path packs RGB444; the HSTX path and the
// ST7789 path both work in RGB555 (the ST7789 driver converts 555->565 at blit, so
// feeding it the 444 image would mis-decode the colours). Select 555 for both.
#if HSTX || USE_ST7789
extern const unsigned char DefaultSS160_555[];
extern const unsigned int DefaultSS160_555_len;
#define DEFAULT_SS DefaultSS160_555
#define DEFAULT_SS_LEN DefaultSS160_555_len
#else
extern const unsigned char DefaultSS160_444[];
extern const unsigned int DefaultSS160_444_len;
#define DEFAULT_SS DefaultSS160_444
#define DEFAULT_SS_LEN DefaultSS160_444_len
#endif