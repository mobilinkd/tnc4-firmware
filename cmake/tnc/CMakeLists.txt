cmake_minimum_required(VERSION 3.22)

project(tnc)
add_library(tnc)

# Enable CMake support for ASM and C languages
enable_language(CXX ASM)

target_compile_definitions(tnc PUBLIC 
    USE_HAL_DRIVER 
    STM32L4P5xx
    ARM_MATH_CM4
    $<$<CONFIG:Debug>:DEBUG>
    $<$<CONFIG:Debug>:KISS_LOGGING>
    $<$<CONFIG:Release>:NDEBUG>
    BOOST_DISABLE_ASSERTS
)

target_include_directories(tnc PUBLIC
    ../../Core/Inc
    ../../USB_DEVICE/App
    ../../USB_DEVICE/Target
    ../../Drivers/STM32L4xx_HAL_Driver/Inc
    ../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy
    ../../Middlewares/Third_Party/FreeRTOS/Source/include
    ../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    ../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ../../Drivers/CMSIS/Device/ST/STM32L4xx/Include
    ../../Drivers/CMSIS/Include
    ../../Drivers/CMSIS/DSP/Include
    ../../Core/TNC
    ../../blaze
)

target_sources(tnc PRIVATE
    ../../Core/TNC/Afsk1200Demodulator.cpp
    ../../Core/TNC/AfskDemodulator.cpp
    ../../Core/TNC/AFSKModulator.cpp
    ../../Core/TNC/AFSKTestTone.cpp
    ../../Core/TNC/AudioInput.cpp
    ../../Core/TNC/AudioLevel.cpp
    ../../Core/TNC/bm78.cpp
    ../../Core/TNC/bm78_eeprom.cpp
    ../../Core/TNC/DCD.cpp
    ../../Core/TNC/Demodulator.cpp
    ../../Core/TNC/Digipeater.cpp
    ../../Core/TNC/FilterCoefficients.cpp
    ../../Core/TNC/FirFilter.cpp
    ../../Core/TNC/Fsk9600Demodulator.cpp
    ../../Core/TNC/Fsk9600Modulator.cpp
    ../../Core/TNC/Goertzel.cpp
    ../../Core/TNC/Golay24.cpp
    ../../Core/TNC/HdlcDecoder.cpp
    ../../Core/TNC/HdlcFrame.cpp
    ../../Core/TNC/IOEventTask.cpp
    ../../Core/TNC/Kiss.cpp
    ../../Core/TNC/KissHardware.cpp
    ../../Core/TNC/KissTask.cpp
    ../../Core/TNC/LEDIndicator.cpp
    ../../Core/TNC/Log.cpp
    ../../Core/TNC/M17.cpp
    ../../Core/TNC/M17Demodulator.cpp
    ../../Core/TNC/M17Encoder.cpp
    ../../Core/TNC/M17Modulator.cpp
    ../../Core/TNC/ModulatorTask.cpp
    ../../Core/TNC/NullPort.cpp
    ../../Core/TNC/PortInterface.cpp
    ../../Core/TNC/power.cpp
    ../../Core/TNC/PowerMode.cpp
    ../../Core/TNC/SerialPort.cpp
    ../../Core/TNC/UsbPort.cpp
)

target_link_directories(tnc PUBLIC
    ../../Drivers/CMSIS/DSP/Lib/GCC
)

target_link_libraries(tnc PRIVATE
    arm_cortexM4lf_math
)

target_compile_options(tnc PRIVATE -fpermissive -fdata-sections -ffunction-sections -fpermissive -fsigned-char -fsingle-precision-constant -fno-inline-functions -ffast-math -fstack-usage --param=large-stack-frame=128 --param=large-stack-frame-growth=50 -fstrict-aliasing --specs=nano.specs -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit --specs=nosys.specs)

# Validate that TNC code is compatible with C++ standard
if(CMAKE_CXX_STANDARD LESS 20)
    message(ERROR "Generated code requires C++20 or higher")
endif()


