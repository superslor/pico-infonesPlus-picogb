# pico-infonesPlus
# IMPORTANT: Use the latest version of tinyusb and pico pio USB.

cmake_minimum_required(VERSION 3.13)

set(projectname piconesPlus)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# ---------------------------------------------------------------------------
# Custom build type: RELEASENODEBUG
# usage: cmake -DCMAKE_BUILD_TYPE=RELEASENODEBUG ..
# ---------------------------------------------------------------------------
message("* Adding build types...")

set(GCC_RELEASENODEBUG_FLAGS "-O2 -DNDEBUG")

set(CMAKE_ASM_FLAGS_RELEASENODEBUG "${GCC_RELEASENODEBUG_FLAGS}"
    CACHE STRING "Flags used by the ASM compiler during REL_O2 builds." FORCE)
set(CMAKE_CXX_FLAGS_RELEASENODEBUG "${GCC_RELEASENODEBUG_FLAGS}"
    CACHE STRING "Flags used by the C++ compiler during REL_O2 builds." FORCE)
set(CMAKE_C_FLAGS_RELEASENODEBUG   "${GCC_RELEASENODEBUG_FLAGS}"
    CACHE STRING "Flags used by the C compiler during REL_O2 builds." FORCE)
set(CMAKE_EXE_LINKER_FLAGS_RELEASENODEBUG ""
    CACHE STRING "Flags used for linking binaries during REL_O2 builds." FORCE)
set(CMAKE_SHARED_LINKER_FLAGS_RELEASENODEBUG ""
    CACHE STRING "Flags used by the shared libraries linker during REL_O2 builds." FORCE)

mark_as_advanced(
    CMAKE_CXX_FLAGS_RELEASENODEBUG
    CMAKE_ASM_FLAGS_RELEASENODEBUG
    CMAKE_C_FLAGS_RELEASENODEBUG
    CMAKE_EXE_LINKER_FLAGS_RELEASENODEBUG
    CMAKE_SHARED_LINKER_FLAGS_RELEASENODEBUG)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug
        CACHE STRING "Choose the type of build : None Debug Release RelWithDebInfo MinSizeRel REL_O2."
        FORCE)
endif()
message("* Current build type is : ${CMAKE_BUILD_TYPE}")

# ---------------------------------------------------------------------------
# Hardware / board selection (project-specific)
# ---------------------------------------------------------------------------
if(NOT HW_CONFIG)
    set(HW_CONFIG 8 CACHE STRING "Select the hardware configuration for your board")
endif()

if(NOT USE_HSTX)
    set(USE_HSTX 1 CACHE STRING "Use HSTX for display output when possible")
endif()

include("pico_shared/BoardConfigs.cmake")

if(NOT PICO_BOARD)
    set(PICO_BOARD pico2 CACHE STRING "Board type")
    message("PICO_BOARD not set, using default: ${PICO_BOARD}")
endif()

add_definitions(-DNDEBUG)

# ---------------------------------------------------------------------------
# TinyUSB family support + Pico SDK init
# Include family_support.cmake BEFORE pico_sdk_init() so that TinyUSB picks up
# the board-specific configuration (PIO USB VBUSEN pin, DP pin, etc.) from the
# BOARD variable set by BoardConfigs.cmake.  The Pico SDK's own TinyUSB
# integration forcibly sets BOARD="pico_sdk" which loses these board-specific
# defines; including family_support.cmake first (with its include_guard) prevents
# the SDK from overriding the board configuration.
# ---------------------------------------------------------------------------
set(FAMILY rp2040)
include($ENV{PICO_SDK_PATH}/lib/tinyusb/hw/bsp/family_support.cmake)

# Some boards (e.g. adafruit_metro_rp2350) force CFG_TUH_RPI_PIO_USB=1 in their
# board.cmake, which TinyUSB's rp2040/family.cmake then bakes into
# tinyusb_common_base. When PIO USB is disabled for this build, strip that
# interface define so it doesn't fight our -DCFG_TUH_RPI_PIO_USB=0 and drag
# hcd_pio_usb.c / pio_usb.h into the compile.
if(ENABLE_PIO_USB EQUAL 0 AND TARGET tinyusb_common_base)
    get_target_property(_tu_defs tinyusb_common_base INTERFACE_COMPILE_DEFINITIONS)
    if(_tu_defs)
        list(REMOVE_ITEM _tu_defs "CFG_TUH_RPI_PIO_USB=1")
        set_target_properties(tinyusb_common_base PROPERTIES
            INTERFACE_COMPILE_DEFINITIONS "${_tu_defs}")
    endif()
endif()

project(${projectname} C CXX ASM)

pico_sdk_init()

# check_and_add_pico_pio_usb_support() needs pico_generate_pio_header which is
# only available after pico_sdk_init().  The first call inside family.cmake
# (before SDK init) was a no-op; this second call actually creates the
# tinyusb_pico_pio_usb target AND appends hcd_pio_usb.c / dcd_pio_usb.c to
# tinyusb_host_base / tinyusb_device_base. Skip it when PIO USB is disabled,
# otherwise those files get compiled into the project and fail looking for
# pio_usb.h (its include dir lives on tinyusb_pico_pio_usb, which we don't
# link in that case).
if(ENABLE_PIO_USB EQUAL 1)
    message("Checking for PIO USB support...")
    check_and_add_pico_pio_usb_support()
endif()
message("PIO USB support: ${ENABLE_PIO_USB}, using PIO: ${PIO_USB_USE_PIO}")

# ---------------------------------------------------------------------------
# ST7789 SPI display backend (PicoGB hardware). When USE_ST7789=1 the DVI/HDMI
# video output is disabled and the NES picture is driven to an ST7789 SPI panel
# instead. Defined globally (here, before add_subdirectory) so the flag also
# reaches the pico_shared submodule (FrensHelpers.cpp).
# ---------------------------------------------------------------------------
if(NOT DEFINED USE_ST7789)
    set(USE_ST7789 0)
endif()
add_compile_definitions(USE_ST7789=${USE_ST7789})
message("ST7789 SPI display backend: USE_ST7789=${USE_ST7789}")

# ---------------------------------------------------------------------------
# USB role for the ST7789 (PicoGB) build. The single RP2040 USB controller can
# be EITHER a host (TinyUSB xinput/hid -> a USB pad as player 2) OR a CDC device
# (serial /dev/ttyACM* for diagnostics) — not both. Default = HOST (the shipping
# build, USB controller works). Configure with -DST7789_USB_SERIAL=1 to build the
# diagnostics variant instead. Only meaningful when USE_ST7789=1.
# ---------------------------------------------------------------------------
if(NOT DEFINED ST7789_USB_SERIAL)
    set(ST7789_USB_SERIAL 0)
endif()
if(USE_ST7789 AND ST7789_USB_SERIAL)
    set(ST7789_USB_DEVICE 1)
else()
    set(ST7789_USB_DEVICE 0)
endif()
add_compile_definitions(ST7789_USB_SERIAL=${ST7789_USB_SERIAL})
# Single macro the C/C++ + tusb_config use: 1 only when the ST7789 build is in CDC
# device mode (so passing -DST7789_USB_SERIAL=1 without -DUSE_ST7789 stays host).
add_compile_definitions(ST7789_USB_DEVICE=${ST7789_USB_DEVICE})
message("ST7789 USB role: ${ST7789_USB_DEVICE} (1=CDC serial device, 0=USB host)")
if(USE_ST7789)
    # NES APU audio is unipolar (large DC offset). Enable the I2S driver's
    # PCM5000A DC-blocking filter so the DC bias doesn't distort on the DAC.
    add_compile_definitions(I2S_AUDIO_COMPENSATE_DC_OFFSET=1)
endif()
# Optional hardware bring-up test pattern (cmake -DST7789_TEST_PATTERN=1)
if(NOT DEFINED ST7789_TEST_PATTERN)
    set(ST7789_TEST_PATTERN 0)
endif()
add_compile_definitions(ST7789_TEST_PATTERN=${ST7789_TEST_PATTERN})

# ---------------------------------------------------------------------------
# Executable
# ---------------------------------------------------------------------------
add_executable(${projectname}
    main.cpp
    splash.cpp
    DefaultSS444.c
    DefaultSS555.c
    state.cpp
    st7789.c
    gpio_buttons.cpp
)

# Optional: embed a NES ROM directly into the firmware.
# Usage: cmake -DEMBED_NES_ROM=/path/to/rom.nes ..
if(EMBED_NES_ROM)
    if(NOT EXISTS "${EMBED_NES_ROM}")
        message(FATAL_ERROR "EMBED_NES_ROM file not found: ${EMBED_NES_ROM}")
    endif()
    set(EMBEDDED_ROM_C "${CMAKE_CURRENT_BINARY_DIR}/embedded_rom.c")
    add_custom_command(
        OUTPUT "${EMBEDDED_ROM_C}"
        COMMAND ${CMAKE_COMMAND} -DINPUT_FILE=${EMBED_NES_ROM} -DOUTPUT_FILE=${EMBEDDED_ROM_C}
                -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_embedded_rom.cmake
        DEPENDS "${EMBED_NES_ROM}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_embedded_rom.cmake"
        COMMENT "Embedding ROM: ${EMBED_NES_ROM}"
    )
    target_sources(${projectname} PRIVATE "${EMBEDDED_ROM_C}")
    target_compile_definitions(${projectname} PRIVATE EMBEDDED_NES_ROM=1)
    message(STATUS "Embedded ROM: ${EMBED_NES_ROM}")
endif()

pico_set_program_name(${projectname} "${projectname}")
pico_set_program_version(${projectname} "0.8")

# ---------------------------------------------------------------------------
# stdio routing
# ---------------------------------------------------------------------------
if(UART_ENABLED EQUAL 0)
    message("Disabling UART output for this board.")
    pico_enable_stdio_uart(${projectname} 0)
else()
    pico_enable_stdio_uart(${projectname} 1)
endif()
if(ST7789_USB_DEVICE)
    # CDC serial over USB for diagnostics (board is a USB device on this build).
    pico_enable_stdio_usb(${projectname} 1)
else()
    pico_enable_stdio_usb(${projectname} 0)
endif()

# ---------------------------------------------------------------------------
# What family_configure_target() used to do for us, inlined:
#   * RP2040-E5 USB device enumeration fix
#   * Silence harmless warnings from TinyUSB / pico-pio-usb sources
# ---------------------------------------------------------------------------
target_compile_definitions(${projectname} PRIVATE
    PICO_RP2040_USB_DEVICE_ENUMERATION_FIX=1
)

target_compile_options(${projectname} PRIVATE
    -Wno-unused-parameter
    -Wno-unused-function
    -Wno-cast-function-type
    -Wno-maybe-uninitialized
)

# ---------------------------------------------------------------------------
# Project compile definitions (board pins, features, etc.)
# ---------------------------------------------------------------------------
target_compile_definitions(${projectname} PRIVATE
    DVICONFIG=${DVICONFIG}
    SDCARD_PIN_CS=${SD_CS}
    SDCARD_PIN_SCK=${SD_SCK}
    SDCARD_PIN_MOSI=${SD_MOSI}
    SDCARD_PIN_MISO=${SD_MISO}
    SDCARD_PIO=pio1                 # Fallback when SPI initialization fails
    SDCARD_SPI=${SD_SPI}
    NES_PIN_CLK=${NES_CLK}
    NES_PIN_DATA=${NES_DATA}
    NES_PIN_LAT=${NES_LAT}
    NES_PIO=${NES_PIO}
    NES_PIN_CLK_1=${NES_CLK_1}
    NES_PIN_DATA_1=${NES_DATA_1}
    NES_PIN_LAT_1=${NES_LAT_1}
    NES_PIO_1=${NES_PIO_1}
    WII_PIN_SDA=${WII_SDA}
    WII_PIN_SCL=${WII_SCL}
    LED_GPIO_PIN=${LED_GPIO_PIN}
    HW_CONFIG=${HW_CONFIG}
    WIIPAD_I2C=${WIIPAD_I2C}
    USE_I2S_AUDIO=${USE_I2S_AUDIO}
    PICO_AUDIO_I2S_DATA_PIN=${PICO_AUDIO_I2S_DATA_PIN}
    PICO_AUDIO_I2S_CLOCK_PIN_BASE=${PICO_AUDIO_I2S_CLOCK_PIN_BASE}
    PICO_AUDIO_I2S_PIO=${PICO_AUDIO_I2S_PIO}
    PICO_AUDIO_I2S_DMA_IRQ=1
    PICO_AUDIO_I2S_CLOCK_PINS_SWAPPED=${PICO_AUDIO_I2S_CLOCK_PINS_SWAPPED}
    PICO_AUDIO_I2S_RESET_PIN=${PICO_AUDIO_I2S_RESET_PIN}
    PICO_AUDIO_I2S_INTERRUPT_PIN=${PICO_AUDIO_I2S_INTERRUPT_PIN}
    PICO_AUDIO_I2S_INTERRUPT_IS_BUTTON=${PICO_AUDIO_I2S_INTERRUPT_IS_BUTTON}
    CFG_TUH_RPI_PIO_USB=${ENABLE_PIO_USB}
    PIO_USB_USE_PIO=${PIO_USB_USE_PIO}
    PSRAM_CS_PIN=${PSRAM_CS_PIN}
    GPIOHSTXD0=${GPIOHSTXD0}
    GPIOHSTXD1=${GPIOHSTXD1}
    GPIOHSTXD2=${GPIOHSTXD2}
    GPIOHSTXCK=${GPIOHSTXCK}
    GPIOHSTXINVERTED=${GPIOHSTXINVERTED}
    ENABLE_VU_METER=${ENABLE_VU_METER}
    WIIPAD_DELAYED_START=${WIIPAD_DELAYED_START}
    ENABLEDVI=1
    DOUBLEFRAMEBUFFER=0
    #HSTX_DEBUG=1
)

if(NOT PICO_PLATFORM STREQUAL "rp2040")
    message(STATUS "Using PICO_PIO_USE_GPIO_BASE=1 for non-rp2040 platforms.")
    target_compile_definitions(${projectname} PRIVATE
        PICO_PIO_USE_GPIO_BASE=1
        PICO_STACK_SIZE=3072
    )
endif()

if(DEFINED PICO_DEFAULT_UART_TX_PIN)
    message(STATUS "Using PICO_DEFAULT_UART_TX_PIN: ${PICO_DEFAULT_UART_TX_PIN}")
    target_compile_definitions(${projectname} PRIVATE
        PICO_DEFAULT_UART_TX_PIN=${PICO_DEFAULT_UART_TX_PIN}
        PICO_DEFAULT_UART=0
    )
endif()

if(DEFINED PICO_DEFAULT_UART_RX_PIN)
    message(STATUS "Using PICO_DEFAULT_UART_RX_PIN: ${PICO_DEFAULT_UART_RX_PIN}")
    target_compile_definitions(${projectname} PRIVATE
        PICO_DEFAULT_UART_RX_PIN=${PICO_DEFAULT_UART_RX_PIN}
    )
endif()

if(DEFINED PICO_DEFAULT_UART)
    message(STATUS "Using PICO_DEFAULT_UART: ${PICO_DEFAULT_UART}")
    target_compile_definitions(${projectname} PRIVATE
        PICO_DEFAULT_UART=${PICO_DEFAULT_UART}
    )
endif()

# ---------------------------------------------------------------------------
# PIO USB (optional)
# tinyusb_pico_pio_usb is a target the pico-sdk exposes when TinyUSB is present.
# ---------------------------------------------------------------------------
if(ENABLE_PIO_USB EQUAL 1)
    if(NOT PIO_DP_PLUS_PIN EQUAL -1)
        message(STATUS "Using PIO USB DP pin: ${PIO_DP_PLUS_PIN}")
        target_compile_definitions(${projectname} PRIVATE
            PICO_DEFAULT_PIO_USB_DP_PIN=${PIO_DP_PLUS_PIN}
        )
    endif()
    message("Adding target link libraries for PIO USB support.")
    target_link_libraries(${projectname} PRIVATE tinyusb_pico_pio_usb)
endif()

# ---------------------------------------------------------------------------
# Link
# ---------------------------------------------------------------------------
target_link_libraries(${projectname} PRIVATE
    pico_stdlib
    pico_multicore
    hardware_dma
    hardware_spi
    hardware_pio
    hardware_i2c
    hardware_interp
    hardware_timer
    hardware_clocks
    hardware_pwm
    hardware_flash
    dvi
    util
    infones
    tinyusb_board
    pico_shared
)

# USB role. The PicoGB build (USE_ST7789) takes input from the GPIO buttons, not
# a USB controller, so the single RP2040 USB controller is configured as a CDC
# DEVICE: the board enumerates as /dev/ttyACM* and printf() goes out over the USB
# cable for diagnostics. All other boards use the TinyUSB HOST stack for USB pads.
if(ST7789_USB_DEVICE)
    target_link_libraries(${projectname} PRIVATE pico_stdio_usb)
    # pico_stdio_usb provides the CDC descriptors, but only auto-enables them when
    # the app isn't otherwise pulling in TinyUSB device (LIB_TINYUSB_DEVICE). Our
    # tusb_config.h enables the device stack, so force the default descriptors on.
    # Also drop the vendor-specific reset-to-BOOTSEL interface: we don't enable
    # CFG_TUD_VENDOR, so that extra interface makes TinyUSB STALL SET_CONFIG
    # (host: "can't set config #1, error -32"). A plain CDC-only device enumerates.
    target_compile_definitions(${projectname} PRIVATE
        PICO_STDIO_USB_USE_DEFAULT_DESCRIPTORS=1
        PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=0
        # NON-BLOCKING stdout: when no host is reading, the CDC FIFO fills and printf would
        # otherwise SPIN until this timeout, stalling core0 once/sec (a visible seam "pulse").
        # 0 => drop chars when the FIFO is full and return immediately. Diag lines still come
        # through whenever a reader is attached (FIFO has space then).
        PICO_STDIO_USB_STDOUT_TIMEOUT_US=0
    )
else()
    target_link_libraries(${projectname} PRIVATE tinyusb_host xinput_host)
endif()

if(PICO_BOARD STREQUAL "pico_w" OR PICO_BOARD STREQUAL "pico2_w")
    target_link_libraries(${projectname} PRIVATE pico_cyw43_arch_none)
endif()

target_include_directories(${projectname} PRIVATE
    pico_lib
    pico_shared
    infones
    .
)
if(NOT ST7789_USB_DEVICE)
    target_include_directories(${projectname} PRIVATE tusb_xinput)
endif()

add_subdirectory(infones)
add_subdirectory(pico_lib/dvi)
add_subdirectory(pico_lib/util)
add_subdirectory(pico_shared)
if(NOT ST7789_USB_DEVICE)
    add_subdirectory(tusb_xinput xinput_host)
endif()

target_link_options(${projectname} PRIVATE -Xlinker --print-memory-usage --data-sections)

pico_add_extra_outputs(${projectname})
