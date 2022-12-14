/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
/*needed for GPIO from pico-examples/gpio/hello_7segment/hello_7segment.c
gpio will be an additional freertos task
*/
#include "hardware/gpio.h"

#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "lwip/apps/lwiperf.h"

#include "FreeRTOS.h"
#include "task.h"
//#include "pw_ssid.h"
#include "crc.h"
#include "crc16.h"
#include "lifting.h"
#include "klt.h"
#include "xtransmit.h"
#include "xreceive.h"
#include "lwip/apps/mqtt.h"
#include "mqtt_example.h"

u16_t mqtt_port = 9883;
//u16_t mqtt_port = 1883;

#if LWIP_TCP

/** Define this to a compile-time IP address initialization
 * to connect anything else than IPv4 loopback
 */
#ifndef LWIP_MQTT_EXAMPLE_IPADDR_INIT
#if LWIP_IPV4

/*192.168.1.229 0xc0a801e5 LWIP_MQTT_EXAMPLE_IPADDR_INIT */
#define LWIP_MQTT_EXAMPLE_IPADDR_INIT = IPADDR4_INIT(PP_HTONL(0xc0a801e5))
//#define LWIP_MQTT_EXAMPLE_IPADDR_INIT = IPADDR4_INIT(PP_HTONL(0xe501a8c0))
/*192.168.1.211 0xc0a801d3 LWIP_MQTT_EXAMPLE_IPADDR_INIT */
//#define LWIP_MQTT_EXAMPLE_IPADDR_INIT = IPADDR4_INIT(PP_HTONL(0xc0a801d3))
//#define LWIP_MQTT_EXAMPLE_IPADDR_INIT = IPADDR4_INIT(PP_HTONL(IPADDR_LOOPBACK))
#else
#define LWIP_MQTT_EXAMPLE_IPADDR_INIT
#endif
#endif

/*needed for GPIO from pico-examples/gpio/hello_7segment/hello_7segment.c
gpio will be an additional freertos task
*/
#define FIRST_GPIO 2
#define BUTTON_GPIO (FIRST_GPIO+7)

#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define TEST_TASK_PRIORITY				( tskIDLE_PRIORITY + 2UL )
#define BLINK_TASK_PRIORITY				( tskIDLE_PRIORITY + 1UL )
#define GPIO_TASK_PRIORITY				( tskIDLE_PRIORITY + 3UL )
#define MQTT_TASK_PRIORITY				( tskIDLE_PRIORITY + 4UL )
#if CLIENT_TEST && !defined(IPERF_SERVER_IP)
#error IPERF_SERVER_IP not defined
#endif


// This array converts a number 0-9 to a bit pattern to send to the GPIOs
int bits[10] = {
        0x3f,  // 0
        0x06,  // 1
        0x5b,  // 2
        0x4f,  // 3
        0x66,  // 4
        0x6d,  // 5
        0x7d,  // 6
        0x07,  // 7
        0x7f,  // 8
        0x67   // 9
};

// Report IP results and exit
static void iperf_report(void *arg, enum lwiperf_report_type report_type,
                         const ip_addr_t *local_addr, u16_t local_port, const ip_addr_t *remote_addr, u16_t remote_port,
                         u32_t bytes_transferred, u32_t ms_duration, u32_t bandwidth_kbitpsec) {
    static uint32_t total_iperf_megabytes = 0;
    uint32_t mbytes = bytes_transferred / 1024 / 1024;
    float mbits = bandwidth_kbitpsec / 1000.0;

    total_iperf_megabytes += mbytes;

    printf("Completed iperf transfer of %d MBytes @ %.1f Mbits/sec\n", mbytes, mbits);
    printf("Total iperf megabytes since start %d Mbytes\n", total_iperf_megabytes);
}

static ip_addr_t mqtt_ip LWIP_MQTT_EXAMPLE_IPADDR_INIT;
static mqtt_client_t* mqtt_client;

static const struct mqtt_connect_client_info_t mqtt_client_info =
{
  "pico_w",
  "testuser", /* user */
  "password123", /* pass */
  100,  /* keep alive */
  "update/memo", /* will_topic */
  NULL, /* will_msg */
  0,    /* will_qos */
  0     /* will_retain */
#if LWIP_ALTCP && LWIP_ALTCP_TLS
  , NULL
#endif
};

static void
mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
  const struct mqtt_connect_client_info_t* client_info = (const struct mqtt_connect_client_info_t*)arg;
  LWIP_UNUSED_ARG(data);

  LWIP_PLATFORM_DIAG(("MQTT client \"%s\" data cb: len %d, flags %d\n", client_info->client_id,
          (int)len, (int)flags));
}

static void
mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
  const struct mqtt_connect_client_info_t* client_info = (const struct mqtt_connect_client_info_t*)arg;

  LWIP_PLATFORM_DIAG(("MQTT client \"%s\" publish cb: topic %s, len %d\n", client_info->client_id,
          topic, (int)tot_len));
}

static void
mqtt_request_cb(void *arg, err_t err)
{
  const struct mqtt_connect_client_info_t* client_info = (const struct mqtt_connect_client_info_t*)arg;

  LWIP_PLATFORM_DIAG(("MQTT client \"%s\" request cb: err %d\n", client_info->client_id, (int)err));
}

static void
mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
  const struct mqtt_connect_client_info_t* client_info = (const struct mqtt_connect_client_info_t*)arg;
  LWIP_UNUSED_ARG(client);

  LWIP_PLATFORM_DIAG(("MQTT client \"%s\" connection cb: status %d\n", client_info->client_id, (int)status));

  if (status == MQTT_CONNECT_ACCEPTED) {
    mqtt_sub_unsub(client,
            "topic_qos1", 1,
            mqtt_request_cb, LWIP_CONST_CAST(void*, client_info),
            1);
    mqtt_sub_unsub(client,
            "topic_qos0", 0,
            mqtt_request_cb, LWIP_CONST_CAST(void*, client_info),
            1);
  }
}
#endif /* LWIP_TCP */

void
mqtt_example_init(void)
{
#if LWIP_TCP
  mqtt_client = mqtt_client_new();
  printf("mqtt_client 0x%x &mqtt_client 0x%x \n", mqtt_client,&mqtt_client);	
  
  mqtt_set_inpub_callback(mqtt_client,
          mqtt_incoming_publish_cb,
          mqtt_incoming_data_cb,
          LWIP_CONST_CAST(void*, &mqtt_client_info));
  printf("mqtt_set_inpub_callback 0x%x\n",mqtt_set_inpub_callback);
  
  
  mqtt_client_connect(mqtt_client,
          &mqtt_ip, mqtt_port,
          mqtt_connection_cb, LWIP_CONST_CAST(void*, &mqtt_client_info),
          &mqtt_client_info);
  printf("mqtt_client_connect 0x%x\n",mqtt_client_connect);
          
#endif /* LWIP_TCP */
}


void blink_task(__unused void *params) {
    bool on = false;
    printf("blink_task starts\n");
    while (true) {
#if 0 && configNUM_CORES > 1
        static int last_core_id;
        if (portGET_CORE_ID() != last_core_id) {
            last_core_id = portGET_CORE_ID();
            printf("blinking now from core %d\n", last_core_id);
        }
#endif
        cyw43_arch_gpio_put(0, on);
        on = !on;
        vTaskDelay(200);
    }
} 

void mqtt_task(__unused void *params) {
    //bool on = false;
    printf("mqtt_task starts\n");
    while (true) {
#if 0 && configNUM_CORES > 1
        static int last_core_id;
        if (portGET_CORE_ID() != last_core_id) {
            last_core_id = portGET_CORE_ID();
            printf("mqtt now from core %d\n", last_core_id);
        }
#endif
        //cyw43_arch_gpio_put(0, on);
        //on = !on;
  
        
        vTaskDelay(200);
    }
}

void gpio_task(__unused void *params) {
    //bool on = false;
    printf("gpio_task starts\n");
     while (true) {
#if 0 && configNUM_CORES > 1
        static int last_core_id;
        if (portGET_CORE_ID() != last_core_id) {
            last_core_id = portGET_CORE_ID();
            printf("gpio now from core %d\n", last_core_id);
        }
#endif
        //cyw43_arch_gpio_put(0, on);
        //on = !on;
        
// We could use gpio_set_dir_out_masked() here
    for (int gpio = FIRST_GPIO; gpio < FIRST_GPIO + 7; gpio++) {
        gpio_init(gpio);
        gpio_set_dir(gpio, GPIO_OUT);
        // Our bitmap above has a bit set where we need an LED on, BUT, we are pulling low to light
        // so invert our output
        gpio_set_outover(gpio, GPIO_OVERRIDE_INVERT);
    }

    gpio_init(BUTTON_GPIO);
    gpio_set_dir(BUTTON_GPIO, GPIO_IN);
    // We are using the button to pull down to 0v when pressed, so ensure that when
    // unpressed, it uses internal pull ups. Otherwise when unpressed, the input will
    // be floating.
    gpio_pull_up(BUTTON_GPIO);

    int val = 0;
    while (true) {
        // Count upwards or downwards depending on button input
        // We are pulling down on switch active, so invert the get to make
        // a press count downwards
        if (!gpio_get(BUTTON_GPIO)) {
            if (val == 9) {
                val = 0;
            } else {
                val++;
            }
        } else if (val == 0) {
            val = 9;
        } else {
            val--;
        }

        // We are starting with GPIO 2, our bitmap starts at bit 0 so shift to start at 2.
        int32_t mask = bits[val] << FIRST_GPIO;

        // Set all our GPIOs in one go!
        // If something else is using GPIO, we might want to use gpio_put_masked()
        gpio_set_mask(mask);
        sleep_ms(250);
        gpio_clr_mask(mask);
    }
        
        vTaskDelay(200);
    }
}

void main_task(__unused void *params) {

    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return;
    }
    cyw43_arch_enable_sta_mode();
    printf("Connecting to WiFi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms("nanotest", "12345678", CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        exit(1);
    } else {
        printf("Connected.\n");
    }

    xTaskCreate(blink_task, "BlinkThread", configMINIMAL_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, NULL);

    xTaskCreate(gpio_task, "GPIOThread", configMINIMAL_STACK_SIZE, NULL, GPIO_TASK_PRIORITY, NULL);

	xTaskCreate(mqtt_task, "MQTTThread", configMINIMAL_STACK_SIZE, NULL, MQTT_TASK_PRIORITY, NULL);

#if CLIENT_TEST
    printf("\nReady, running iperf client\n");
    ip_addr_t clientaddr;
    ip4_addr_set_u32(&clientaddr, ipaddr_addr(xstr(IPERF_SERVER_IP)));
    assert(lwiperf_start_tcp_client_default(&clientaddr, &iperf_report, NULL) != NULL);
#else
    printf("\nReady, running iperf server at %s\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));
    lwiperf_start_tcp_server_default(&iperf_report, NULL);
#endif

    while(true) {
        // not much to do as LED is in another task, and we're using RAW (callback) lwIP API
        vTaskDelay(100);
    }

    cyw43_arch_deinit();
}

void vLaunch( void) {
    TaskHandle_t task;
    xTaskCreate(main_task, "TestMainThread", configMINIMAL_STACK_SIZE, NULL, TEST_TASK_PRIORITY, &task);

#if NO_SYS && configUSE_CORE_AFFINITY && configNUM_CORES > 1
    // we must bind the main task to one core (well at least while the init is called)
    // (note we only do this in NO_SYS mode, because cyw43_arch_freertos
    // takes care of it otherwise)
    vTaskCoreAffinitySet(task, 1);
#endif

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}

int main( void )
{
    stdio_init_all();
    printf("adding gpio support to the program\n");
	unsigned char message[3] = {0xd3, 0x01, 0x00};
	buildCRCTable();
	printf("Back from buildCRCTable \n");
	message[2] = getCRC(message, 2); 
	printf("0x%x 0x%x 0x%x \n",message[0],message[1],message[2]);
    /* Configure the hardware ready to run the demo. */
    const char *rtos_name;

    
    
#if ( portSUPPORT_SMP == 1 )
    rtos_name = "FreeRTOS SMP";
#else
    rtos_name = "FreeRTOS";
#endif

#if ( portSUPPORT_SMP == 1 ) && ( configNUM_CORES == 2 )
    printf("Starting %s on both cores:\n", rtos_name);
    vLaunch();
#elif ( RUN_FREE_RTOS_ON_CORE == 1 )
    printf("Starting %s on core 1:\n", rtos_name);
    multicore_launch_core1(vLaunch);
    while (true);
#else
    printf("Starting %s on core 0:\n", rtos_name);
    vLaunch();
#endif
    return 0;
}
