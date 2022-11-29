#!/bin/bash
openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program pico_w/freertos/mqtt/picow_freertos_iperf_mqtt.elf verify reset exit"
