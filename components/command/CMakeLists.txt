idf_component_register(
	SRCS "command.c" "hash.c" "signature.c"
	INCLUDE_DIRS "."
	REQUIRES "lownet"
	PRIV_REQUIRES "ping" "serial" "utility" "mbedtls"
)
