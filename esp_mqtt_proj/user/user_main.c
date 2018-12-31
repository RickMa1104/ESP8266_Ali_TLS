/* main.c -- MQTT client example
*
* Copyright (c) 2014-2015, Tuan PM <tuanpm at live dot com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Redis nor the names of its contributors may be used
* to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
#include "ets_sys.h"
#include "driver/uart.h"
#include "osapi.h"
#include "mqtt.h"
#include "wifi.h"
#include "config.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
#include "mem.h"

MQTT_Client mqttClient;

static ETSTimer WenDuTimer,sntp_timer;

static void ICACHE_FLASH_ATTR sntpfn()
{
	unsigned long ts = 0;
    ts = sntp_get_current_timestamp();
    os_printf("current time : %s\n", sntp_get_real_time(ts));
    if (ts == 0) {
        os_printf("did not get a valid time from sntp server\n");
    } else {
            os_timer_disarm(&sntp_timer);
            MQTT_Connect(&mqttClient);
    }
}

static void ICACHE_FLASH_ATTR wendu_send()
{
	char *randwendu="11";
	os_sprintf(randwendu,"%d",os_random()%100);
	INFO("WenDu:%s\r\n",randwendu);
	MQTT_Publish(&mqttClient, "/NgFcEKAJRGf/Esp8266/WenDu", randwendu, os_strlen(randwendu), 0, 0);
}

void wifiConnectCb(uint8_t status)
{
	if(status == STATION_GOT_IP){
        sntp_setservername(0, "pool.ntp.org");        // set sntp server after got ip address
        sntp_init();
        os_timer_disarm(&sntp_timer);
        os_timer_setfn(&sntp_timer, (os_timer_func_t *)sntpfn, NULL);
        os_timer_arm(&sntp_timer, 1000, 1);//1s
	} else {
		MQTT_Disconnect(&mqttClient);
	}
}
void mqttConnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Connected\r\n");

	MQTT_Subscribe(client, "/NgFcEKAJRGf/Esp8266/KaiGuan", 0);

	os_timer_disarm(&WenDuTimer);
	os_timer_setfn(&WenDuTimer, (os_timer_func_t *)wendu_send, NULL);
	os_timer_arm(&WenDuTimer, 10000, 1);
}

void mqttDisconnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Disconnected\r\n");
	os_timer_disarm(&WenDuTimer);
}

void mqttPublishedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Published\r\n");
}

void mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len)
{
	char *topicBuf = (char*)os_zalloc(topic_len+1),
			*dataBuf = (char*)os_zalloc(data_len+1);

	MQTT_Client* client = (MQTT_Client*)args;

	os_memcpy(topicBuf, topic, topic_len);
	topicBuf[topic_len] = 0;

	os_memcpy(dataBuf, data, data_len);
	dataBuf[data_len] = 0;

	INFO("Receive topic: %s, data: %s \r\n", topicBuf, dataBuf);


	os_free(topicBuf);
	os_free(dataBuf);
}


/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
 *******************************************************************************/
uint32 ICACHE_FLASH_ATTR
user_rf_cal_sector_set(void)
{
    enum flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        case FLASH_SIZE_64M_MAP_1024_1024:
            rf_cal_sec = 2048 - 5;
            break;
        case FLASH_SIZE_128M_MAP_1024_1024:
            rf_cal_sec = 4096 - 5;
            break;
        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}


void user_init(void)
{
	int randtimestamp=os_random()%1000;

	uint8_t mydigest[20],str[40],dst[41],randclient[60],message[80]={"clientIdDVES_001deviceNameEsp8266productKeyNgFcEKAJRGftimestamp789"},secret[60]={"pQy3OwciEXt5Xg2zcudEIpDn4Xp45IL3"};
	int i;

	uart_init(115200,115200);
	os_delay_us(60000);

	os_sprintf(message,"clientIdDVES_001deviceNameEsp8266productKeyNgFcEKAJRGftimestamp%d",randtimestamp);
	os_sprintf(randclient,"DVES_001|securemode=2,signmethod=hmacsha1,timestamp=%d|",randtimestamp);

	os_printf("\r\n%s\r\n",message);
	os_printf("\r\n%s\r\n",randclient);

//	CFG_Load();
//¹þÎ÷É¢ÁÐ¼ÓÃÜ
	ssl_hmac_sha1(message,os_strlen(message),secret,os_strlen(secret),mydigest);
	for(i=0;i<20;i++)
	{
		str[2*i]=mydigest[i]>>4;
		str[2*i+1]=mydigest[i]&0xf;
	}
	for(i=0;i<40;i++)
	{
		os_sprintf(&dst[i],"%x",str[i]);
	}
	dst[40]='\0';
	os_printf("\r\n%s\r\n",dst);

//	CFG_Load();

	MQTT_InitConnection(&mqttClient, MQTT_HOST, MQTT_PORT, DEFAULT_SECURITY);
	//MQTT_InitConnection(&mqttClient, "192.168.11.122", 1880, 0);

	MQTT_InitClient(&mqttClient, randclient, MQTT_USER, dst, MQTT_KEEPALIVE, 1);
	//MQTT_InitClient(&mqttClient, "client_id", "user", "pass", 120, 1);

	MQTT_InitLWT(&mqttClient, "/lwt", "offline", 0, 0);
	MQTT_OnConnected(&mqttClient, mqttConnectedCb);
	MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
	MQTT_OnPublished(&mqttClient, mqttPublishedCb);
	MQTT_OnData(&mqttClient, mqttDataCb);

	WIFI_Connect(STA_SSID, STA_PASS, wifiConnectCb);

	INFO("\r\nSystem started ...\r\n");
}
