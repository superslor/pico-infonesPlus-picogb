/*
 * gpio_buttons.cpp - direct GPIO button input for the PicoGB hardware.
 * See gpio_buttons.h for the overview.
 */
#include "gpio_buttons.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "gamepad.h"

#if ST7789_USB_DEVICE
// In the serial-diagnostics variant hid_app.cpp (which normally defines these) is
// excluded, but menu.cpp / FrensHelpers.h still reference abSwapped (A/B order), so
// provide them here. The Manta flags are USB-pad detection state, 0 for GPIO input.
// The default (host) build keeps hid_app.cpp, so these must NOT be redefined there.
int abSwapped = 1;              // match hid_app.cpp's ABSWAPPED default
int isManta[2] = {0, 0};
int isMantaVariant[2] = {0, 0};
#endif

namespace
{
    struct ButtonMap
    {
        unsigned gpio;
        int bit; // io::GamePadState::Button bit (some are 1<<31, i.e. negative as int)
    };

    // GB pin map (active-low). UP=2 DOWN=3 LEFT=4 RIGHT=5 A=6 B=7 SELECT=8 START=9.
    const ButtonMap kButtons[] = {
        {2, io::GamePadState::Button::UP},
        {3, io::GamePadState::Button::DOWN},
        {4, io::GamePadState::Button::LEFT},
        {5, io::GamePadState::Button::RIGHT},
        {6, io::GamePadState::Button::A},
        {7, io::GamePadState::Button::B},
        {8, io::GamePadState::Button::SELECT},
        {9, io::GamePadState::Button::START},
    };
}

void gpio_buttons_init(void)
{
    for (const auto &b : kButtons)
    {
        gpio_init(b.gpio);
        gpio_set_dir(b.gpio, GPIO_IN);
        gpio_pull_up(b.gpio); // buttons short the pin to GND when pressed
    }
}

void gpio_buttons_update(void)
{
    uint32_t buttons = 0;
    for (const auto &b : kButtons)
    {
        if (!gpio_get(b.gpio)) // pressed = low
            buttons |= (uint32_t)b.bit;
    }

    auto &gp = io::getCurrentGamePadState(0);
    gp.buttons = buttons;
    gp.flagConnected(true); // present this as a connected controller 1
}
