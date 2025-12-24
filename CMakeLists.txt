#
# CMakeLists.txt
#
#  Created on: 24 dec. 2025
#      Author: Ludo
#

# Minimum CMake version.
cmake_minimum_required(VERSION 3.23)

# Project creation.
project(meteofox-spsws)
add_executable(${PROJECT_NAME})

# Use object build mode in all sub-directories.
set(BUILD_MODE "OBJECT")

# Macro to convert options to variables.
macro(add_compilation_flag FLAG_NAME FLAG_DESCRIPTION FLAG_DEFAULT_VALUE)
    option(FLAG_NAME FLAG_DESCRIPTION FLAG_DEFAULT_VALUE)
    set(${FLAG_NAME} ${FLAG_DEFAULT_VALUE} CACHE STRING ${FLAG_DESCRIPTION})
    list(APPEND COMPILATION_FLAGS_LIST ${FLAG_NAME})
endmacro()

# Hardware compilation flags.
set(SPSWS_HW_VERSION "NONE" CACHE STRING "Hardware version")

# Software compilation flags.
add_compilation_flag(SPSWS_MODE_CLI "Enable command line mode." OFF)
add_compilation_flag(SPSWS_WIND_RAINFALL_MEASUREMENTS "Enable wind and rainfall measurements." ON)
add_compilation_flag(SPSWS_WIND_VANE_ULTIMETER "Use Ultimeter wind vane." OFF)
add_compilation_flag(SPSWS_SEN15901_EMULATOR "Enable SEN15901 emulator mode." OFF)

# Hardware specific settings.
# SPSWS HW1.0.
if(SPSWS_HW_VERSION STREQUAL "HW1_0")   
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m0plus -Os")
    set(SPSWS_MCU "stm32l0xx")
    set(PROJECT_LINKER_SCRIPT "stm32l041x6xx.ld")
# SPSWS HW2.0.
elseif(SPSWS_HW_VERSION STREQUAL "HW2_0")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m0plus -O0")
    set(SPSWS_MCU "stm32l0xx")
    set(PROJECT_LINKER_SCRIPT "stm32l081xbxx.ld")
# Unknown version.
else()
    message(FATAL_ERROR "Invalid hardware version.")
endif()
set(PROJECT_LINKER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/drivers/device/${SPSWS_MCU}-device/linker")

# Add hardware compilation flags.
add_compile_definitions(
    __SPSWS_FLAGS_H__
    ${SPSWS_HW_VERSION}
)

# Add software compilation flags.
foreach(FLAG ${COMPILATION_FLAGS_LIST})
    # Macros only defined.
    if(${FLAG} STREQUAL ON)
        add_compile_definitions(${FLAG})
    endif()
    # Macros with value.
    if((NOT ${FLAG} STREQUAL OFF) AND (NOT ${FLAG} STREQUAL ON))
        set(${FLAG} "(${${FLAG}})")
        add_compile_definitions(${FLAG}=${${FLAG}}) 
    endif()
endforeach()

# Add project sources files.
target_sources(${PROJECT_NAME}
    PRIVATE
        drivers/peripherals/src/mcu_mapping.c
        drivers/components/src/dps310_hw.c
        drivers/components/src/max11136_hw.c
        drivers/components/src/neom8x_hw.c
        drivers/components/src/sen15901_hw.c
        drivers/components/src/sensors_hw.c
        drivers/components/src/sht3x_hw.c
        drivers/components/src/si1133_hw.c
        drivers/components/src/sx1232_hw.c
        drivers/components/src/ultimeter_hw.c
        drivers/utils/src/terminal_hw.c
        middleware/analog/src/analog.c
        middleware/cli/src/cli.c
        middleware/gps/src/gps.c
        middleware/power/src/power.c
        middleware/sigfox/src/mcu_api.c
        middleware/sigfox/src/rf_api.c
        middleware/sigfox/src/rfe.c
        application/src/main.c
)

# Project include folders.
include_directories(${PROJECT_NAME}
    PRIVATE
        drivers/device/inc
        drivers/registers/inc
        drivers/registers/${SPSWS_MCU}-registers/inc
        drivers/peripherals/inc
        drivers/peripherals/${SPSWS_MCU}-drivers/inc
        drivers/utils/inc
        drivers/utils/embedded-utils/inc
        drivers/components/inc
        drivers/components/dps310-driver/inc
        drivers/components/max111xx-driver/inc
        drivers/components/neom8x-driver/inc
        drivers/components/sen15901-driver/inc
        drivers/components/sht3x-driver/inc
        drivers/components/si1133-driver/inc
        drivers/components/sx1232-driver/inc
        drivers/components/ultimeter-driver/inc
        middleware/analog/inc
        middleware/cli/inc
        middleware/gps/inc
        middleware/power/inc
        middleware/sigfox/inc
        middleware/sigfox/sigfox-ep-lib/inc
        middleware/sigfox/sigfox-ep-addon-rfp/inc
        application/inc
)

# Add submodules sources files.
add_subdirectory(drivers/device/${SPSWS_MCU}-device EXCLUDE_FROM_ALL)
add_subdirectory(drivers/peripherals/${SPSWS_MCU}-drivers EXCLUDE_FROM_ALL)
add_subdirectory(drivers/utils/embedded-utils EXCLUDE_FROM_ALL)
add_subdirectory(drivers/components/dps310-driver EXCLUDE_FROM_ALL)
add_subdirectory(drivers/components/max111xx-driver EXCLUDE_FROM_ALL)
add_subdirectory(drivers/components/neom8x-driver EXCLUDE_FROM_ALL)
add_subdirectory(drivers/components/sen15901-driver EXCLUDE_FROM_ALL)
add_subdirectory(drivers/components/sht3x-driver EXCLUDE_FROM_ALL)
add_subdirectory(drivers/components/si1133-driver EXCLUDE_FROM_ALL)
add_subdirectory(drivers/components/sx1232-driver EXCLUDE_FROM_ALL)
add_subdirectory(drivers/components/ultimeter-driver EXCLUDE_FROM_ALL)
add_subdirectory(middleware/sigfox/sigfox-ep-lib EXCLUDE_FROM_ALL)
add_subdirectory(middleware/sigfox/sigfox-ep-addon-rfp EXCLUDE_FROM_ALL)

# Link libraries.
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${SPSWS_MCU}-device
        ${SPSWS_MCU}-drivers
        embedded-utils
        dps310-driver
        max111xx-driver
        neom8x-driver
        sen15901-driver
        sht3x-driver
        si1133-driver
        sx1232-driver
        ultimeter-driver
        sigfox_ep_lib_obj
        sigfox_ep_addon_rfp_obj
        c_nano
        nosys
        gcc
)

# Linker and artifact.
include(script/cmake-arm-none-eabi/linker.cmake)
include(script/cmake-arm-none-eabi/artifact.cmake)
