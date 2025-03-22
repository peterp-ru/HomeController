idf_component_register(SRCS "HTTPServerHC.c" "RingBufferHC.c" "HomeControl.c"
                    INCLUDE_DIRS "."
                    EMBED_FILES "favicon.ico" "upload_script.html" "FirmwareScript.html" "ConfigurationScript.html" "Secret.bin")

# spiffs_create_partition_image(storage ../spiffs_image FLASH_IN_PROJECT)