idf_component_register(SRCS "vision_ai.cpp" "vision.c"
                        INCLUDE_DIRS "include"
                        PRIV_INCLUDE_DIRS "."
                        REQUIRES esp_lcd
                                 esp_hw_support
                                 esp_mm
                                 freertos
                                 heap
                                 esp_driver_i2c
                                 esp_driver_cam
                                 esp_driver_isp
                                 esp_driver_ppa
                                 esp_timer
                                 esp_cam_sensor
                                 esp-dl
                                 pedestrian_detect
                                 waveshare__esp_lcd_jd9365_10_1)
