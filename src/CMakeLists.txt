set(CAN2040_SOURCE can2040/src)
set(CBUSDEFS cbusdefs)
set(INIH inih)
set(CANLOCK CANLock)

add_subdirectory(FatFsPicoSPI/FatFs_SPI build)

add_executable(CANLock
   # Source for CAN2040 PIO based CAN controller
   ${CAN2040_SOURCE}/can2040.c
   # Sources for CBUSPico libary 
   CBUSLED.cpp
   CBUSSwitch.cpp
   CBUSConfig.cpp
   CBUSParams.cpp
   SystemTick.cpp
   CBUSCircularBuffer.cpp
   CBUSLongMessage.cpp
   CBUSGridConnect.cpp
   CBUS.cpp
   ACAN2040.cpp
   CBUSACAN2040.cpp
   hw_config.c
   ${INIH}/ini.c
   # CANLock module using libary
   CBUSWiFi.cpp
   ${CANLOCK}/CANLock.cpp
)

target_include_directories(CANLock PRIVATE
   ${CMAKE_CURRENT_SOURCE_DIR}
   ${CAN2040_SOURCE}
   ${CBUSDEFS}
   ${INIH}
)

target_compile_options(CANLock PRIVATE
   # can2040 has an unused param, -Wpedantic is a step too far !
   -Wall -Wextra -Werror -Wno-unused-parameter 
)

# Custom linker scipt to put CAN2040 code into RAM
pico_set_linker_script(CANLock ${CMAKE_CURRENT_SOURCE_DIR}/memmap_locking.ld)

target_link_libraries(
   CANLock
   pico_stdlib
   #pico_stdio_semihosting
   pico_multicore
   cmsis_core
   hardware_i2c
   hardware_flash
   pico_cyw43_arch_lwip_threadsafe_background
   pico_lwip_http
   FatFs_SPI
)

pico_add_extra_outputs(CANLock)
