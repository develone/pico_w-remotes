/*
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Dirk Ziegelmeier <dziegel@gmx.de>
 *
 */
#include <stdio.h> 
//#include "hardware/rtc.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "pico/util/datetime.h"
static volatile bool fired = false;

static void alarm_callback(void) {
    datetime_t t = {0};
    rtc_get_datetime(&t);
    char datetime_buf[256];
    char *datetime_str = &datetime_buf[0];
    datetime_to_str(datetime_str, sizeof(datetime_buf), &t);
    printf("Alarm Fired At %s\n", datetime_str);
    stdio_flush();
    fired = true;
}

/*needed for ntp*/
#include <string.h>
#include <time.h>
#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
typedef struct NTP_T_ {
    ip_addr_t ntp_server_address;
    bool dns_request_sent;
    struct udp_pcb *ntp_pcb;
    absolute_time_t ntp_test_time;
    alarm_id_t ntp_resend_alarm;
} NTP_T;

#define NTP_SERVER "pool.ntp.org"
#define NTP_MSG_LEN 48
#define NTP_PORT 123
#define NTP_DELTA 2208988800 // seconds between 1 Jan 1900 and 1 Jan 1970
#define NTP_TEST_TIME (30 * 1000)
#define NTP_RESEND_TIME (10 * 1000)


/*needed for rtc */
datetime_t t;
datetime_t alarm;
datetime_t t_ntp;
datetime_t *pt;
datetime_t *palarm;
datetime_t *pt_ntp;
u8_t rtc_set_flag = 0;
char datetime_buf[256];
char *datetime_str = &datetime_buf[0];
/*needed for rtc */
/*needed for ntp*/
 

/*needed for GPIO from pico-examples/gpio/hello_7segment/hello_7segment.c
gpio will be an additional freertos task
*/
#include "hardware/gpio.h"
#include "hardware/watchdog.h"

#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "lwip/apps/lwiperf.h"
#include "pw_ssid.h"
#include "FreeRTOS.h"
#include "task.h"
/*needed for GPIO from pico-examples/gpio/hello_7segment/hello_7segment.c
gpio will be an additional freertos task
*/
#define FIRST_GPIO 2
#define BUTTON_GPIO (FIRST_GPIO+7)
#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
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

#define TEST_TASK_PRIORITY				( tskIDLE_PRIORITY + 2UL )
#define BLINK_TASK_PRIORITY				( tskIDLE_PRIORITY + 1UL )
#define GPIO_TASK_PRIORITY				( tskIDLE_PRIORITY + 3UL )
#define MQTT_TASK_PRIORITY				( tskIDLE_PRIORITY + 4UL )
#define NTP_TASK_PRIORITY				( tskIDLE_PRIORITY + 5UL )
#define WATCHDOG_TASK_PRIORITY				( tskIDLE_PRIORITY + 6UL )
/*needed for ntp*/

/*needed for ntp*/
#ifndef USE_LED
#define USE_LED 1
#endif
#include "lwip/apps/mqtt.h"
#include "mqtt_example.h"
mqtt_request_cb_t pub_mqtt_request_cb_t; 
//u16_t mqtt_port = 9883;
u16_t mqtt_port = 1883;
#if LWIP_TCP

/** Define this to a compile-time IP address initialization
 * to connect anything else than IPv4 loopback
 */
#ifndef LWIP_MQTT_EXAMPLE_IPADDR_INIT
#if LWIP_IPV4

/* One of the defines below needs to uncommented to connect 
to mosqquitto broker */
/*192.168.1.245 0xc0a801f5 LWIP_MQTT_EXAMPLE_IPADDR_INIT pi4-60*/
//#define LWIP_MQTT_EXAMPLE_IPADDR_INIT = IPADDR4_INIT(PP_HTONL(0xc0a801f5))
/*192.168.1.212 0xc0a801d4 LWIP_MQTT_EXAMPLE_IPADDR_INIT pi4-50*/
//#define LWIP_MQTT_EXAMPLE_IPADDR_INIT = IPADDR4_INIT(PP_HTONL(0xc0a801d4))

//*192.168.1.157 0xc0a8019d LWIP_MQTT_EXAMPLE_IPADDR_INIT pi4-20*/
//#define LWIP_MQTT_EXAMPLE_IPADDR_INIT = IPADDR4_INIT(PP_HTONL(0xc0a8019d))
/*192.168.1.211 0xc0a801d3 LWIP_MQTT_EXAMPLE_IPADDR_INIT pi4-30*/
//#define LWIP_MQTT_EXAMPLE_IPADDR_INIT = IPADDR4_INIT(PP_HTONL(0xc0a801d3))

#else
#define LWIP_MQTT_EXAMPLE_IPADDR_INIT
#endif
#endif


char PUB_PAYLOAD[] = "this is a message from pico_w ctrl 0       ";
char PUB_PAYLOAD_SCR[] = "this is a message from pico_w ctrl 0       ";
char PUB_EXTRA_ARG[] = "test";
u16_t payload_size;

static ip_addr_t mqtt_ip LWIP_MQTT_EXAMPLE_IPADDR_INIT;
static mqtt_client_t* mqtt_client;
static mqtt_client_t* saved_mqtt_client = NULL;
static u8_t mqtt_connected = 1;
static u8_t check_mqtt_connected;
static u8_t check_wifi_connected;
static u8_t wifi_connected = 1;

static const struct mqtt_connect_client_info_t mqtt_client_info =
{
  CYW43_HOST_NAME,
  "testuser", /* user */
  "password123", /* pass */
  200,  /* keep alive */
  "topic_qos0", /* will_topic */
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
  if (len==19) printf("%s \n",data);
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
  saved_mqtt_client = mqtt_client;
  printf("saved_mqtt_client 0x%x mqtt_client 0x%x \n", saved_mqtt_client,mqtt_client);
  mqtt_set_inpub_callback(mqtt_client,
          mqtt_incoming_publish_cb,
          mqtt_incoming_data_cb,
          LWIP_CONST_CAST(void*, &mqtt_client_info));
  printf("mqtt_set_inpub_callback 0x%x\n",mqtt_set_inpub_callback);
  

  mqtt_connected = mqtt_client_connect(mqtt_client,
          &mqtt_ip, mqtt_port,
          mqtt_connection_cb, LWIP_CONST_CAST(void*, &mqtt_client_info),
          &mqtt_client_info);
  printf("mqtt_client_connect 0x%x\n",mqtt_client_connect);

 
  //printf("0x%x \n",LWIP_CONST_CAST(void*, &mqtt_client_info));
/*
  strcpy(PUB_PAYLOAD_SCR,PUB_PAYLOAD);
  strcat( PUB_PAYLOAD_SCR,CYW43_HOST_NAME);
  payload_size = sizeof(PUB_PAYLOAD_SCR) + 7;
  printf("%s  %d \n",PUB_PAYLOAD_SCR,sizeof(PUB_PAYLOAD_SCR));
  mqtt_publish(mqtt_client,"update/memo",PUB_PAYLOAD_SCR,payload_size,2,0,pub_mqtt_request_cb_t,PUB_EXTRA_ARG);
*/   
          
#endif /* LWIP_TCP */
}

#if CLIENT_TEST && !defined(IPERF_SERVER_IP)
#error IPERF_SERVER_IP not defined
#endif

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
#if CYW43_USE_STATS
    printf("packets in %u packets out %u\n", CYW43_STAT_GET(PACKET_IN_COUNT), CYW43_STAT_GET(PACKET_OUT_COUNT));
#endif
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
/*needed for ntp*/
void ntp_task(__unused void *params) {
    //bool on = false;
    printf("ntp_task starts\n");
	run_ntp_test();
    while (true) {
#if 0 && configNUM_CORES > 1
        static int last_core_id;
        if (portGET_CORE_ID() != last_core_id) {
            last_core_id = portGET_CORE_ID();
            printf("ntp now from core %d\n", last_core_id);
        }
#endif
        //cyw43_arch_gpio_put(0, on);
        //on = !on;
        
        vTaskDelay(200);
    }
}
/*needed for ntp*/

void watchdog_task(__unused void *params) {
    //bool on = false;

    while (true) {
	 
	if (wifi_connected == 0) watchdog_update();
 
       vTaskDelay(200);
    }
}

void mqtt_task(__unused void *params) {
    //bool on = false;
    printf("mqtt_task starts\n");
mqtt_subscribe(mqtt_client,"pub_time", 2,pub_mqtt_request_cb_t,PUB_EXTRA_ARG);

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
        printf("in mqtt\n");
  strcpy(PUB_PAYLOAD_SCR,PUB_PAYLOAD);
  strcat( PUB_PAYLOAD_SCR,CYW43_HOST_NAME);
  payload_size = sizeof(PUB_PAYLOAD_SCR) + 7;
  printf("%s  %d \n",PUB_PAYLOAD_SCR,sizeof(PUB_PAYLOAD_SCR));
  check_mqtt_connected = mqtt_client_is_connected(saved_mqtt_client);
  if (check_mqtt_connected == 0) {
	mqtt_client_free(saved_mqtt_client);
	mqtt_example_init();
  } 	
  /*
  mqtt_client_is_connected 1 if connected to server, 0 otherwise 
  */
  printf("saved_mqtt_client 0x%x check_mqtt_connected %d \n", saved_mqtt_client,check_mqtt_connected);

  mqtt_publish(mqtt_client,"update/memo",PUB_PAYLOAD_SCR,payload_size,2,0,pub_mqtt_request_cb_t,PUB_EXTRA_ARG);
	
        vTaskDelay(25000);
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
	printf("watchdog_task starts\n");
	watchdog_enable(10000, 1);
	while (wifi_connected) {
    	cyw43_arch_enable_sta_mode();
    	printf("Connecting to WiFi...\n");
   		if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        	printf("failed to connect.\n");
        	exit(1);
    	} else {
        	printf("Connected.\n");
        	printf("mqtt_port = %d &mqtt_port 0x%x\n",mqtt_port,&mqtt_port);
        	printf("mqtt_ip = 0x%x &mqtt_ip = 0x%x\n",mqtt_ip,&mqtt_ip);
        	printf("IPADDR_LOOPBACK = 0x%x \n",IPADDR_LOOPBACK);
			printf("saved_mqtt_client 0x%x %d \n", saved_mqtt_client,mqtt_connected);
        	mqtt_example_init();
			printf("saved_mqtt_client 0x%x %d \n", saved_mqtt_client,mqtt_connected);
			wifi_connected=0;
			printf("wifi_connected %d\n",wifi_connected);
    	}
	}
    xTaskCreate(watchdog_task, "WatchdogThread", configMINIMAL_STACK_SIZE, NULL, WATCHDOG_TASK_PRIORITY, NULL);

    xTaskCreate(blink_task, "BlinkThread", configMINIMAL_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, NULL);

    xTaskCreate(gpio_task, "GPIOThread", configMINIMAL_STACK_SIZE, NULL, GPIO_TASK_PRIORITY, NULL);

	xTaskCreate(mqtt_task, "MQTTThread", configMINIMAL_STACK_SIZE, NULL, MQTT_TASK_PRIORITY, NULL);

	/*needed for ntp*/
	xTaskCreate(ntp_task, "NTPThread", configMINIMAL_STACK_SIZE, NULL, NTP_TASK_PRIORITY, NULL);
	/*needed for ntp*/

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


void set_rtc(datetime_t *pt, datetime_t *pt_ntp,datetime_t *palarm) {
	if(rtc_set_flag==0) {
		pt->year = pt_ntp->year;
		pt->month = pt_ntp->month; 
		pt->day = pt_ntp->day;
		pt->dotw = 0;
		pt->hour = pt_ntp->hour;
		pt->min = pt_ntp->min;
		pt->sec = pt_ntp->sec;
		palarm->year = pt_ntp->year;
		palarm->month = pt_ntp->month;
		palarm->day = pt_ntp->day;
		palarm->dotw = 0;
		palarm->hour = pt_ntp->hour;
		palarm->min = pt_ntp->min + 1;
		palarm->sec = pt_ntp->sec;
		rtc_set_flag=1;
    	// Start the RTC
    	rtc_init();
    	rtc_set_datetime(&t);
		sleep_us(64);
		rtc_set_alarm(&alarm, &alarm_callback);

	}
        rtc_get_datetime(&t);
        datetime_to_str(datetime_str, sizeof(datetime_buf), &t);
        printf("\r%s      ", datetime_str);
}
/*needed for ntp*/
// Called with results of operation
static void ntp_result(NTP_T* state, int status, time_t *result) {
    if (status == 0 && result) {
        struct tm *utc = gmtime(result);
		t_ntp.year = utc->tm_year + 1900;
		t_ntp.month = utc->tm_mon + 1; 
		t_ntp.day = utc->tm_mday;
		t_ntp.hour = utc->tm_hour;
		t_ntp.min = utc->tm_min;
		t_ntp.sec = utc->tm_sec;
		pt = &t;
		pt_ntp = &t_ntp;
		palarm = &alarm;
		printf("0x%x 0x%x 0x%x\n",pt,pt_ntp,palarm);
		set_rtc(pt,pt_ntp,palarm);
		
		//printf("%02d:%02d:%02d\n",pt->hour,pt->min,pt->sec); 
        printf("got ntp response: %02d/%02d/%04d %02d:%02d:%02d\n", utc->tm_mday, utc->tm_mon + 1, utc->tm_year + 1900,
               utc->tm_hour, utc->tm_min, utc->tm_sec);
    }

    if (state->ntp_resend_alarm > 0) {
        cancel_alarm(state->ntp_resend_alarm);
        state->ntp_resend_alarm = 0;
    }
    state->ntp_test_time = make_timeout_time_ms(NTP_TEST_TIME);
    state->dns_request_sent = false;
}

static int64_t ntp_failed_handler(alarm_id_t id, void *user_data);

// Make an NTP request
static void ntp_request(NTP_T *state) {
    // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
    // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
    // these calls are a no-op and can be omitted, but it is a good practice to use them in
    // case you switch the cyw43_arch type later.
    cyw43_arch_lwip_begin();
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, NTP_MSG_LEN, PBUF_RAM);
    uint8_t *req = (uint8_t *) p->payload;
    memset(req, 0, NTP_MSG_LEN);
    req[0] = 0x1b;
    udp_sendto(state->ntp_pcb, p, &state->ntp_server_address, NTP_PORT);
    pbuf_free(p);
    cyw43_arch_lwip_end();
}

static int64_t ntp_failed_handler(alarm_id_t id, void *user_data)
{
    NTP_T* state = (NTP_T*)user_data;
    printf("ntp request failed\n");
    ntp_result(state, -1, NULL);
	wifi_connected=1;
    return 0;
}

// Call back with a DNS result
static void ntp_dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    NTP_T *state = (NTP_T*)arg;
    if (ipaddr) {
        state->ntp_server_address = *ipaddr;
        printf("ntp address %s\n", ip4addr_ntoa(ipaddr));
        ntp_request(state);
    } else {
        printf("ntp dns request failed\n");
        ntp_result(state, -1, NULL);
    }
}

// NTP data received
static void ntp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    NTP_T *state = (NTP_T*)arg;
    uint8_t mode = pbuf_get_at(p, 0) & 0x7;
    uint8_t stratum = pbuf_get_at(p, 1);

    // Check the result
    if (ip_addr_cmp(addr, &state->ntp_server_address) && port == NTP_PORT && p->tot_len == NTP_MSG_LEN &&
        mode == 0x4 && stratum != 0) {
        uint8_t seconds_buf[4] = {0};
        pbuf_copy_partial(p, seconds_buf, sizeof(seconds_buf), 40);
        uint32_t seconds_since_1900 = seconds_buf[0] << 24 | seconds_buf[1] << 16 | seconds_buf[2] << 8 | seconds_buf[3];
        uint32_t seconds_since_1970 = seconds_since_1900 - NTP_DELTA;
        time_t epoch = seconds_since_1970;
        ntp_result(state, 0, &epoch);
    } else {
        printf("invalid ntp response\n");
        ntp_result(state, -1, NULL);
    }
    pbuf_free(p);
}

// Perform initialisation
static NTP_T* ntp_init(void) {
    NTP_T *state = calloc(1, sizeof(NTP_T));
    if (!state) {
        printf("failed to allocate state\n");
        return NULL;
    }
    state->ntp_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (!state->ntp_pcb) {
        printf("failed to create pcb\n");
        free(state);
        return NULL;
    }
    udp_recv(state->ntp_pcb, ntp_recv, state);
    return state;
}

// Runs ntp test forever
void run_ntp_test(void) {
    NTP_T *state = ntp_init();
    if (!state)
        return;
    while(true) {
        if (absolute_time_diff_us(get_absolute_time(), state->ntp_test_time) < 0 && !state->dns_request_sent) {

            // Set alarm in case udp requests are lost
            state->ntp_resend_alarm = add_alarm_in_ms(NTP_RESEND_TIME, ntp_failed_handler, state, true);

            // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
            // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
            // these calls are a no-op and can be omitted, but it is a good practice to use them in
            // case you switch the cyw43_arch type later.
            cyw43_arch_lwip_begin();
            int err = dns_gethostbyname(NTP_SERVER, &state->ntp_server_address, ntp_dns_found, state);
            cyw43_arch_lwip_end();

            state->dns_request_sent = true;
            if (err == ERR_OK) {
                ntp_request(state); // Cached result
            } else if (err != ERR_INPROGRESS) { // ERR_INPROGRESS means expect a callback
                printf("dns request failed\n");
                ntp_result(state, -1, NULL);
            }
        }
#if PICO_CYW43_ARCH_POLL
        // if you are using pico_cyw43_arch_poll, then you must poll periodically from your
        // main loop (not from a timer) to check for WiFi driver or lwIP work that needs to be done.
        cyw43_arch_poll();
        sleep_ms(1);
#else
        // if you are not using pico_cyw43_arch_poll, then WiFI driver and lwIP work
        // is done via interrupt in the background. This sleep is just an example of some (blocking)
        // work you might be doing.
        sleep_ms(1000);
#endif
    }
    free(state);
}
/*needed for ntp*/
int main() {
    stdio_init_all();
    printf("RTC Alarm!\n");


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
