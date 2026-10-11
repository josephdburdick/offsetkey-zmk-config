if(CONFIG_ZMK_INPUT_PROCESSOR_MOTION_GATE)
    target_sources(app PRIVATE src/input_processor_motion_gate.c)
endif()
