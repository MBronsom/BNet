idf_component_register(SRCS "ppposClient_example.c"
                             INCLUDE_DIRS "")

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
