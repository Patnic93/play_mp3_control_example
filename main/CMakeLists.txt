set(COMPONENT_SRCS
    ./play_mp3_control_example.c
    ./i2c_slave_ctrl.c
    ./uart_ctrl.c
)
set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()
