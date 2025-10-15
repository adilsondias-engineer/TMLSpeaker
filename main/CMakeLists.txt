idf_component_register(SRCS main.c
                            img_bt_logo.c
                            audio_jack3.c
                            bt_app_core.c
                       REQUIRES bt esp_driver_i2s nvs_flash esp_ringbuf esp_driver_dac lvgl lvgl_esp32_drivers driver
                       INCLUDE_DIRS ".")

#bt_app_av.c