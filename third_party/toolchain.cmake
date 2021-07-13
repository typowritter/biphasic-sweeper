set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER "arm-none-eabi-gcc")
set(CMAKE_C_COMPILER_WORKS 1)

set(arm_compile_link_options
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-d16
    -mfloat-abi=hard
)

add_compile_options(
    ${arm_compile_link_options}
    -fdata-sections
    -ffunction-sections
    -DLV_CONF_INCLUDE_SIMPLE
)

add_link_options(
    ${arm_compile_link_options}
    -lc
    -lnosys
    -specs=nosys.specs
    -Wl,--gc-sections
)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
