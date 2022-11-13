add_executable(picow_tcpip_server_background
        picow_tcp_server.c
		crc.h
		crc16.h
		lifting.h
		klt.h
		xtransmit.h
		xreceive.h
		head-tail.h
        )
target_compile_definitions(picow_tcpip_server_background PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )
target_include_directories(picow_tcpip_server_background PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )
target_link_libraries(picow_tcpip_server_background
        pico_cyw43_arch_lwip_threadsafe_background
        pico_stdlib
		crc_crc16
		lifting
		klt
		littlefs
		xmodem
        )

pico_add_extra_outputs(picow_tcpip_server_background)

add_executable(picow_tcpip_server_poll
        picow_tcp_server.c
        )
target_compile_definitions(picow_tcpip_server_poll PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )
target_include_directories(picow_tcpip_server_poll PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )
target_link_libraries(picow_tcpip_server_poll
        pico_cyw43_arch_lwip_poll
        pico_stdlib
		crc_crc16
		lifting
		klt
		littlefs 
        )
pico_add_extra_outputs(picow_tcpip_server_poll)
