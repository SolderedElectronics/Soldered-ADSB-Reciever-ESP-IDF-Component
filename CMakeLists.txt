idf_component_register(SRCS "src/soldered_adsb_receiver.c" "src/adsb_receiver_csv.c" "src/adsb_receiver_mavlink.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_uart esp_driver_gpio esp_timer)
