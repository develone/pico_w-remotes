add_executable(picow_freertos_iperf_mqtt
        mqtt_example.c
		mqtt.c
		mqtt_example.h 
        )
target_compile_definitions(picow_freertos_iperf_mqtt PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )
target_include_directories(picow_freertos_iperf_mqtt PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts
        )
target_link_libraries(picow_freertos_iperf_mqtt
        pico_cyw43_arch_lwip_threadsafe_background
        pico_stdlib
        pico_lwip_iperf
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap	
        )
pico_add_extra_outputs(picow_freertos_iperf_mqtt)


 