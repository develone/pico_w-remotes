#!/bin/bash
rm -rf remote1  remote2 remote3 remote4 remote5 remote6

mkdir remote1

mkdir remote2

mkdir remote3

mkdir remote4

mkdir remote5

mkdir remote6

cd remote1

cmake -DHOSTNAME="remote1"  -DPICO_BOARD=pico_w  -DTEST_TCP_SERVER_IP="10.0.1.12"  -DFREERTOS_KERNEL_PATH:PATH=/home/devel/FreeRTOS-Kernel ..

cd ../remote2

cmake -DHOSTNAME="remote2"  -DPICO_BOARD=pico_w  -DTEST_TCP_SERVER_IP="10.0.1.12"  -DFREERTOS_KERNEL_PATH:PATH=/home/devel/FreeRTOS-Kernel ..

cd ../remote3

cmake -DHOSTNAME="remote3"   -DPICO_BOARD=pico_w  -DTEST_TCP_SERVER_IP="10.0.1.12"  -DFREERTOS_KERNEL_PATH:PATH=/home/devel/FreeRTOS-Kernel ..

cd ../remote4

cmake -DHOSTNAME="remote4"  -DPICO_BOARD=pico_w  -DTEST_TCP_SERVER_IP="10.0.1.12"  -DFREERTOS_KERNEL_PATH:PATH=/home/devel/FreeRTOS-Kernel ..

cd ../remote5

cmake -DHOSTNAME="remote5"  -DPICO_BOARD=pico_w  -DTEST_TCP_SERVER_IP="10.0.1.12"  -DFREERTOS_KERNEL_PATH:PATH=/home/devel/FreeRTOS-Kernel ..

cd ../remote6

cmake -DHOSTNAME="remote6"  -DPICO_BOARD=pico_w  -DTEST_TCP_SERVER_IP="10.0.1.12"  -DFREERTOS_KERNEL_PATH:PATH=/home/devel/FreeRTOS-Kernel ..

cd ../remote1

make &

cd ../remote2

make &


cd ../remote3

make &

cd ../remote4

make &


cd ../remote5

make &

cd ../remote6

make &


