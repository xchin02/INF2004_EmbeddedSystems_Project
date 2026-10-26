

add_executable(motion motion.c)

target_link_libraries(motion
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_clocks
    hardware_sync
    hardware_pio
)
pico_generate_pio_header(motion ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)
# printf over USB serial, not UART
pico_enable_stdio_usb(motion 1)
pico_enable_stdio_uart(motion 0)

# Generates robo_pico_encoder.uf2 for drag-and-drop flashing
pico_add_extra_outputs(motion)