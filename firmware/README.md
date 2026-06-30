# PicoNES Portable — ST7789 firmware (RP2040 / Pico-GB hardware, 320×240 landscape)

Flash by holding **BOOTSEL** while plugging in the Pico, then copy a `.uf2` onto the `RPI-RP2` drive.

| File | Build | USB | Serial diag |
|------|-------|-----|-------------|
| `piconesPlus_320_host.uf2`       | Shipping | USB controller = Player 2 | none |
| `piconesPlus_320_serialdiag.uf2` | Diagnostics | none | USB CDC on `/dev/ttyACM0` |

golden64 scatter anti-tear build, 60 fps. ST7789 Portable Port by **Slor**.
