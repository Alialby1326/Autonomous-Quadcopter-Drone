# Command-line / CI build for the drone firmware.
#
# This builds and flashes the firmware without the IDE. STM32CubeIDE is still
# the debugger; its generated makefile lives in Debug/ and is gitignored.
#
# The flags below mirror what CubeIDE generates, with one deliberate addition:
# -u _printf_float. newlib-nano's printf omits float conversion unless that
# symbol is forced in, so "%f" silently produces nothing without it. If you
# change flags here, mirror them in .cproject (and vice versa) or the two build
# paths will disagree.
#
#   cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake
#   cmake --build build

cmake_minimum_required(VERSION 3.20)

project(drone_code C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)  # -std=gnu11, as CubeIDE uses

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

set(MCU_FLAGS -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F401CCUX_FLASH.ld)

file(GLOB HAL_SOURCES ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/*.c)

add_executable(${PROJECT_NAME}.elf
    Core/Src/main.c
    Core/Src/imu.c
    Core/Src/telemetry.c
    Core/Src/timebase.c
    Core/Src/motors.c
    Core/Src/stm32f4xx_hal_msp.c
    Core/Src/stm32f4xx_it.c
    Core/Src/syscalls.c
    Core/Src/sysmem.c
    Core/Src/system_stm32f4xx.c
    Core/Startup/startup_stm32f401ccux.s
    ${HAL_SOURCES}
)

target_include_directories(${PROJECT_NAME}.elf PRIVATE
    Core/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
)

target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
    USE_HAL_DRIVER
    STM32F401xC
    $<$<CONFIG:Debug>:DEBUG>
)

target_compile_options(${PROJECT_NAME}.elf PRIVATE
    ${MCU_FLAGS}
    --specs=nano.specs
    -Wall
    -ffunction-sections
    -fdata-sections
    $<$<CONFIG:Debug>:-Og>
    $<$<CONFIG:Debug>:-g3>
    $<$<CONFIG:Release>:-Os>
)

target_link_options(${PROJECT_NAME}.elf PRIVATE
    ${MCU_FLAGS}
    -T${LINKER_SCRIPT}
    --specs=nosys.specs
    --specs=nano.specs
    -static
    -u _printf_float          # without this, "%f" prints nothing under nano
    -Wl,--gc-sections
    -Wl,-Map=${PROJECT_NAME}.map,--cref
    -Wl,--start-group -lc -lm -Wl,--end-group
)

# Relink when the linker script changes.
set_target_properties(${PROJECT_NAME}.elf PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})

# Report flash/RAM usage after every build -- the F401CC has 256K flash and
# 64K RAM, and it is useful to watch that number move.
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
)

# Flashable artifacts alongside the .elf.
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
)

# ---------------------------------------------------------------------------
# Flashing: cmake --build build --target flash
#
# Builds if needed, then writes the .elf with STM32CubeProgrammer's CLI,
# verifies it and resets the chip. Override with -D at configure time:
#
#   STM32_PROGRAMMER_CLI  path to STM32_Programmer_CLI(.exe); found automatically
#                         from a standalone CubeProgrammer install or the newest
#                         copy bundled with STM32CubeIDE
#   FLASH_PORT            SWD (ST-Link, default) or usb1 (the chip's USB DFU
#                         bootloader: hold BOOT0, tap NRST, release BOOT0)
#
# Under WSL the Windows .exe is used as-is: Windows owns the USB device, so no
# usbipd passthrough is needed. The .elf path is converted with wslpath.
# ---------------------------------------------------------------------------
set(FLASH_PORT SWD CACHE STRING "STM32_Programmer_CLI port: SWD or usb1")

file(GLOB _cubeide_programmers
    /mnt/c/ST/STM32CubeIDE_*/STM32CubeIDE/plugins/*cubeprogrammer*/tools/bin/STM32_Programmer_CLI.exe
)
list(SORT _cubeide_programmers COMPARE NATURAL ORDER DESCENDING)  # newest first

find_program(STM32_PROGRAMMER_CLI
    NAMES STM32_Programmer_CLI STM32_Programmer_CLI.exe
    PATHS
        "$ENV{HOME}/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin"
        "/mnt/c/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin"
    NO_CACHE
)
if(NOT STM32_PROGRAMMER_CLI AND _cubeide_programmers)
    list(GET _cubeide_programmers 0 _found)
    set(STM32_PROGRAMMER_CLI "${_found}" CACHE FILEPATH "STM32_Programmer_CLI executable")
endif()

if(STM32_PROGRAMMER_CLI)
    set(_flash_image "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.elf")
    if(STM32_PROGRAMMER_CLI MATCHES "\\.exe$")
        execute_process(COMMAND wslpath -w "${_flash_image}"
            OUTPUT_VARIABLE _flash_image OUTPUT_STRIP_TRAILING_WHITESPACE)
    endif()
    message(STATUS "flash target: ${STM32_PROGRAMMER_CLI} (port=${FLASH_PORT})")
    add_custom_target(flash
        COMMAND "${STM32_PROGRAMMER_CLI}" -c port=${FLASH_PORT} -w "${_flash_image}" -v -rst
        DEPENDS ${PROJECT_NAME}.elf
        USES_TERMINAL
        VERBATIM
    )
else()
    add_custom_target(flash
        COMMAND ${CMAKE_COMMAND} -E echo "STM32_Programmer_CLI not found; configure with -DSTM32_PROGRAMMER_CLI=<path>"
        COMMAND ${CMAKE_COMMAND} -E false
        VERBATIM
    )
endif()
