set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(ARM_GCC_BIN_DIR "$ENV{ARM_GCC_BIN_DIR}" CACHE PATH "Path to the GNU Arm Embedded bin directory")

if(NOT ARM_GCC_BIN_DIR)
    if(EXISTS "$ENV{LOCALAPPDATA}/arduino15/packages/adafruit/tools/arm-none-eabi-gcc/9-2019q4/bin/arm-none-eabi-gcc.exe")
        set(ARM_GCC_BIN_DIR "$ENV{LOCALAPPDATA}/arduino15/packages/adafruit/tools/arm-none-eabi-gcc/9-2019q4/bin" CACHE PATH "Auto-detected GNU Arm Embedded bin directory" FORCE)
    elseif(EXISTS "$ENV{LOCALAPPDATA}/arduino15/packages/teensy/tools/teensy-compile/11.3.1/arm/bin/arm-none-eabi-gcc.exe")
        set(ARM_GCC_BIN_DIR "$ENV{LOCALAPPDATA}/arduino15/packages/teensy/tools/teensy-compile/11.3.1/arm/bin" CACHE PATH "Auto-detected GNU Arm Embedded bin directory" FORCE)
    else()
        message(FATAL_ERROR "ARM_GCC_BIN_DIR is not set. Point it to the folder containing arm-none-eabi-gcc(.exe).")
    endif()
endif()

set(TOOLCHAIN_PREFIX "${ARM_GCC_BIN_DIR}/arm-none-eabi")

if(WIN32)
    set(TOOLCHAIN_SUFFIX ".exe")
else()
    set(TOOLCHAIN_SUFFIX "")
endif()

set(CMAKE_C_COMPILER "${TOOLCHAIN_PREFIX}-gcc${TOOLCHAIN_SUFFIX}")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PREFIX}-g++${TOOLCHAIN_SUFFIX}")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_PREFIX}-gcc${TOOLCHAIN_SUFFIX}")
set(CMAKE_AR "${TOOLCHAIN_PREFIX}-ar${TOOLCHAIN_SUFFIX}")
set(CMAKE_OBJCOPY "${TOOLCHAIN_PREFIX}-objcopy${TOOLCHAIN_SUFFIX}")
set(CMAKE_OBJDUMP "${TOOLCHAIN_PREFIX}-objdump${TOOLCHAIN_SUFFIX}")
set(CMAKE_SIZE "${TOOLCHAIN_PREFIX}-size${TOOLCHAIN_SUFFIX}")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
