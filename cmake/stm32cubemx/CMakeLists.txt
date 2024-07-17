cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_HAL_DRIVER 
	STM32L4P5xx
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
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
    ../../Middlewares/ST/ARM/DSP/Inc
)

target_sources(stm32cubemx INTERFACE
    ../../Core/Src/main.c
    ../../Core/Src/freertos.c
    ../../Core/Src/stm32l4xx_it.c
    ../../Core/Src/stm32l4xx_hal_msp.c
    ../../Core/Src/stm32l4xx_hal_timebase_tim.c
    ../../USB_DEVICE/App/usb_device.c
    ../../USB_DEVICE/App/usbd_desc.c
    ../../USB_DEVICE/App/usbd_cdc_if.c
    ../../USB_DEVICE/Target/usbd_conf.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd_ex.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc_ex.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_comp.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_crc.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_crc_ex.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dac.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dac_ex.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_iwdg.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_opamp.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_opamp_ex.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rng.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rng_ex.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc_ex.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c
    ../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c
    ../../Core/Src/system_stm32l4xx.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/list.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/queue.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/timers.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_3.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    ../../Core/Src/sysmem.c
    ../../Core/Src/syscalls.c
    ../../startup_stm32l4p5xx.s
)

target_link_directories(stm32cubemx INTERFACE
    ../../Middlewares/ST/ARM/DSP/Lib
)

target_link_libraries(stm32cubemx INTERFACE 
	:libarm_cortexM4lf_math.a
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


