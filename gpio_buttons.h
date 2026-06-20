/*
 * gpio_buttons.h - direct GPIO button input for the PicoGB hardware.
 *
 * Reads the 8 face/d-pad buttons wired to GP2..GP9 (active-low, internal
 * pull-ups), matching the pico1-gb-320 build, and feeds them into the
 * emulator's unified gamepad state (io::getCurrentGamePadState(0)) so both the
 * menu and the running game see them as controller 1. Used in place of the
 * USB-host / NES shift-register controller support on this hardware.
 *
 *   UP=2  DOWN=3  LEFT=4  RIGHT=5  A=6  B=7  SELECT=8  START=9
 */
#ifndef GPIO_BUTTONS_H
#define GPIO_BUTTONS_H

#ifdef __cplusplus
extern "C" {
#endif

// Configure the button GPIOs as inputs with pull-ups. Call once at startup.
void gpio_buttons_init(void);

// Sample the buttons and publish them to gamepad 0. Call once per frame (from
// both the menu loop and the game loop).
void gpio_buttons_update(void);

#ifdef __cplusplus
}
#endif

#endif /* GPIO_BUTTONS_H */
