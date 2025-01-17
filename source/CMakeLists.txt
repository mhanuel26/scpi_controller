set(TARGET_NAME scpi_controller)

add_executable(${TARGET_NAME}
        ${TARGET_NAME}.c
        scpi/scpi-def.c
        ../libraries/scpi-parser-2.3/libscpi/src/parser.c
        ../libraries/scpi-parser-2.3/libscpi/src/lexer.c
        ../libraries/scpi-parser-2.3/libscpi/src/error.c
        ../libraries/scpi-parser-2.3/libscpi/src/ieee488.c
        ../libraries/scpi-parser-2.3/libscpi/src/minimal.c
        ../libraries/scpi-parser-2.3/libscpi/src/utils.c
        ../libraries/scpi-parser-2.3/libscpi/src/units.c
        ../libraries/scpi-parser-2.3/libscpi/src/fifo.c
        )

target_link_libraries(${TARGET_NAME} PRIVATE
        pico_stdlib
        hardware_spi
        hardware_dma
        ETHERNET_FILES
        IOLIBRARY_FILES
        DHCP_FILES
        DNS_FILES
        TIMER_FILES
        )


target_include_directories(${TARGET_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/scpi
        ../libraries/scpi-parser-2.3/libscpi/inc
)

pico_enable_stdio_usb(${TARGET_NAME} 1)
pico_enable_stdio_uart(${TARGET_NAME} 0)

pico_add_extra_outputs(${TARGET_NAME})
