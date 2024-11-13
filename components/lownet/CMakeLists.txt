idf_component_register(
	SRCS "lownet.c" "lownet_util.c" "lownet_crypt.c"
	INCLUDE_DIRS "."
	REQUIRES "serial" "utility" "esp_wifi" "nvs_flash" "esp_timer" "mbedtls"
)
