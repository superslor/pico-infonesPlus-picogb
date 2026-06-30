
cmake_minimum_required(VERSION 3.13)
set(LWMEM_OPTS_FILE ${CMAKE_CURRENT_LIST_DIR}/lwmem_opts_ppp.h)
add_library( pico_shared INTERFACE)
target_sources(pico_shared INTERFACE
menu.cpp
RomLister.cpp
settings.cpp
FrensHelpers.cpp
gamepad.cpp
nespad.cpp
wiipad.cpp
FrensFonts.cpp
#ffwrappers.cpp
crc32.cpp
vumeter.cpp
soundrecorder.cpp
samplesound.c
wavplayer.cpp
FlashParams.cpp
#PicoPlusPsram.cpp
)
# hid_app.cpp is the USB-HOST controller handler. It is excluded ONLY for the
# ST7789 serial-diagnostics variant (ST7789_USB_DEVICE=1), which puts USB in CDC
# device mode — there the host tuh_* APIs aren't built (CFG_TUH_ENABLED=0) and its
# abSwapped/isManta globals are provided by gpio_buttons.cpp instead. The normal
# (host) build keeps it so a USB controller can be player 2.
if(NOT ST7789_USB_DEVICE)
    target_sources(pico_shared INTERFACE hid_app.cpp)
endif()
add_subdirectory(drivers/pico_fatfs)     
add_subdirectory(drivers/pico_audio_i2s)
add_subdirectory(drivers/lwmem)
# add_subdirectory(drivers/pico_hstx)
# add_subdirectory(drivers/pico_audio_mcp4822) # SPI Audio using MCP4822 DAC. Works but not used
add_subdirectory(drivers/pico_psram)
add_subdirectory(drivers/pico_hdmi)

target_include_directories(pico_shared INTERFACE
        .
)
target_compile_definitions(pico_shared INTERFACE 
 NOHSTXCLOCK=1 # Do not set the hstx clock in the pico_hdmi driver, as it is already set FrensHelpers.cpp
)
pico_generate_pio_header(${projectname} ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)
target_link_libraries(pico_shared 
                      INTERFACE 
                      pico_rand
                      pico_fatfs 
                      pico_audio_i2s 
                      lwmem 
                      #pico_hstx 
                      #pico_audio_mcp4822 
                      pico_psram
                      pico_hdmi)

