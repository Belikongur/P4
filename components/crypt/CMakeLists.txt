idf_component_register(
    SRCS "crypt.c"
    INCLUDE_DIRS "."
    REQUIRES "lownet"
    PRIV_REQUIRES "serial" "mbedtls"
)
