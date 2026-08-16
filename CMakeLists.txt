# Command-line / CI build for the drone firmware.
#
# STM32CubeIDE remains the debugger and flasher; its generated makefile lives in
# Debug/ and is gitignored. This file exists so the firmware can be compiled
# without the IDE.
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
    Core/Src/adc.c
    Core/Src/dma.c
    Core/Src/gpio.c
    Core/Src/i2c.c
    Core/Src/tim.c
    Core/Src/usart.c
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
