idf_component_register(
                    SRCS "app_main.cpp"
                    PRIV_REQUIRES
                    	spi_flash
                    	arduino
                    	Adafruit_NeoPixel
                    	CLI_Arduino
                    	esp_partition
                    	ESP32S3_Common_Libs
                    	ESP32S3_App
                    	nvs_flash
                    PRIV_INCLUDE_DIRS)
