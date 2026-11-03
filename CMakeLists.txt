add_executable(line-follow-test_v3 line-follow-test.c motion.c ir-line-sensor.c barcode.c)
target_link_libraries(line-follow-test_v3 pico_stdlib hardware_pwm hardware_pio hardware_clocks hardware_adc)

pico_generate_pio_header(line-follow-test_v3 ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)
# printf over USB serial, not UART
pico_enable_stdio_usb(line-follow-test_v3 1)
pico_enable_stdio_uart(line-follow-test_v3 0)

# Generates robo_pico_encoder.uf2 for drag-and-drop flashing
pico_add_extra_outputs(line-follow-test_v3)