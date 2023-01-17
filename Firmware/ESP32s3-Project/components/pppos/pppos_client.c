#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_netif.h"
#include "netif/ppp/pppos.h"
#include "netif/ppp/ppp.h"
#include "netif/ppp/pppapi.h"
#if LWIP_DNS
#include "lwip/dns.h"
#endif

#include "pppos_client.h"


// === Module configuration that you can set via 'make menuconfig'. ===
#define UART_GPIO_TX CONFIG_MODULE_TX
#define UART_GPIO_RX CONFIG_MODULE_RX
#define UART_BDRATE CONFIG_MODULE_BDRATE

#ifdef CONFIG_PPPOS_DEBUG
#define PPPOS_DEBUG 1
#else
#define PPPOS_DEBUG 0
#endif
#define BUF_SIZE (1024)
#define MODULE_OK_Str "OK"
#define PPPOSMUTEX_TIMEOUT 1000 / portTICK_PERIOD_MS

#define PPPOS_CLIENT_STACK_SIZE 1024*3


// shared variables, use mutex to access them
static uint8_t pppos_client_status = PPPOS_CLIENT_STATE_FIRSTINIT;
static int do_pppos_connect = 1;
static uint32_t pppos_rx_count;
static uint32_t pppos_tx_count;
static uint8_t pppos_task_started = 0;
static uint8_t module_rfOff = 0;

// local variables
static QueueHandle_t pppos_mutex = NULL;
const char *PPP_User = CONFIG_MODULE_INTERNET_USER;
const char *PPP_Pass = CONFIG_MODULE_INTERNET_PASSWORD;
static int uart_num = UART_NUM_1;

static uint8_t esp_netif_initialized = 0;

// The PPP control block
static ppp_pcb *ppp = NULL;

// The PPP IP interface
struct netif ppp_netif;

static const char *TAG = "[PPPOS CLIENT]";

typedef struct
{
	char		*cmd;
	uint16_t	cmdSize;
	char		*cmdResponseOnOk;
	uint16_t	timeoutMs;
	uint16_t	delayMs;
	uint8_t		skip;
}MODULE_Cmd;

static MODULE_Cmd cmd_AT =
{
	.cmd = "AT\r\n",
	.cmdSize = sizeof("AT\r\n")-1,
	.cmdResponseOnOk = MODULE_OK_Str,
	.timeoutMs = 300,
	.delayMs = 0,
	.skip = 0,
};

static MODULE_Cmd cmd_NoSMSInd =
{
	.cmd = "AT+CNMI=0,0,0,0,0\r\n",
	.cmdSize = sizeof("AT+CNMI=0,0,0,0,0\r\n")-1,
	.cmdResponseOnOk = MODULE_OK_Str,
	.timeoutMs = 1000,
	.delayMs = 0,
	.skip = 0,
};

static MODULE_Cmd cmd_Reset =
{
	.cmd = "ATZ\r\n",
	.cmdSize = sizeof("ATZ\r\n")-1,
	.cmdResponseOnOk = MODULE_OK_Str,
	.timeoutMs = 300,
	.delayMs = 0,
	.skip = 0,
};

static MODULE_Cmd cmd_RFOn =
{
	.cmd = "AT+CFUN=1\r\n",
	.cmdSize = sizeof("ATCFUN=1,0\r\n")-1,
	.cmdResponseOnOk = MODULE_OK_Str,
	.timeoutMs = 10000,
	.delayMs = 1000,
	.skip = 0,
};

static MODULE_Cmd cmd_EchoOff =
{
	.cmd = "ATE0\r\n",
	.cmdSize = sizeof("ATE0\r\n")-1,
	.cmdResponseOnOk = MODULE_OK_Str,
	.timeoutMs = 300,
	.delayMs = 0,
	.skip = 0,
};

static MODULE_Cmd cmd_Pin =
{
	.cmd = "AT+CPIN?\r\n",
	.cmdSize = sizeof("AT+CPIN?\r\n")-1,
	.cmdResponseOnOk = "CPIN: READY",
	.timeoutMs = 5000,
	.delayMs = 0,
	.skip = 0,
};

static MODULE_Cmd cmd_Reg =
{
	.cmd = "AT+CREG?\r\n",
	.cmdSize = sizeof("AT+CREG?\r\n")-1,
	.cmdResponseOnOk = "CREG: 0,1",
	.timeoutMs = 3000,
	.delayMs = 2000,
	.skip = 0,
};

static MODULE_Cmd cmd_APN =
{
	.cmd = NULL,
	.cmdSize = 0,
	.cmdResponseOnOk = MODULE_OK_Str,
	.timeoutMs = 8000,
	.delayMs = 0,
	.skip = 0,
};

static MODULE_Cmd cmd_Connect =
{
	.cmd = "ATD*99#\r\n",
	.cmdSize = sizeof("ATD*99#\r\n")-1,
	.cmdResponseOnOk = "CONNECT",
	.timeoutMs = 30000,
	.delayMs = 1000,
	.skip = 0,
};

static MODULE_Cmd *MODULE_Init[] =
{
		&cmd_AT,
		&cmd_Reset,
		&cmd_EchoOff,
		&cmd_RFOn,
		&cmd_NoSMSInd,
		&cmd_Pin,
		&cmd_Reg,
		&cmd_APN,
		&cmd_Connect,
};

#define MODULE_InitCmdsSize  (sizeof(MODULE_Init)/sizeof(MODULE_Cmd *))


// PPP status callback
//--------------------------------------------------------------
static void ppp_status_cb(ppp_pcb *pcb, int err_code, void *ctx)
{
	struct netif *pppif = ppp_netif(pcb);
	LWIP_UNUSED_ARG(ctx);

	switch(err_code) {
		case PPPERR_NONE: {
			#if PPPOS_DEBUG
			ESP_LOGI(TAG,"status_cb: Connected");
			#if PPP_IPV4_SUPPORT
			ESP_LOGI(TAG,"   ipaddr    = %s", ipaddr_ntoa(&pppif->ip_addr));
			ESP_LOGI(TAG,"   gateway   = %s", ipaddr_ntoa(&pppif->gw));
			ESP_LOGI(TAG,"   netmask   = %s", ipaddr_ntoa(&pppif->netmask));
			#endif

			#if PPP_IPV6_SUPPORT
			ESP_LOGI(TAG,"   ip6addr   = %s", ip6addr_ntoa(netif_ip6_addr(pppif, 0)));
			#endif

			#if LWIP_DNS
            const ip_addr_t *ns;
            ns = dns_getserver(0);
            ESP_LOGI(TAG, "   dns1    = %s\n", ipaddr_ntoa(ns));
            ns = dns_getserver(1);
            ESP_LOGI(TAG, "   dns2    = %s\n", ipaddr_ntoa(ns));
        	#endif /* LWIP_DNS */
			#endif
			xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
			pppos_client_status = PPPOS_CLIENT_STATE_CONNECTED;
			xSemaphoreGive(pppos_mutex);
			break;
		}
		case PPPERR_PARAM: {
			#if PPPOS_DEBUG
			ESP_LOGE(TAG,"status_cb: Invalid parameter");
			#endif
			break;
		}
		case PPPERR_OPEN: {
			#if PPPOS_DEBUG
			ESP_LOGE(TAG,"status_cb: Unable to open PPP session");
			#endif
			break;
		}
		case PPPERR_DEVICE: {
			#if PPPOS_DEBUG
			ESP_LOGE(TAG,"status_cb: Invalid I/O device for PPP");
			#endif
			break;
		}
		case PPPERR_ALLOC: {
			#if PPPOS_DEBUG
			ESP_LOGE(TAG,"status_cb: Unable to allocate resources");
			#endif
			break;
		}
		case PPPERR_USER: {
			/* ppp_free(); -- can be called here */
			#if PPPOS_DEBUG
			ESP_LOGW(TAG,"status_cb: User interrupt (disconnected)");
			#endif
			xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
			pppos_client_status = PPPOS_CLIENT_STATE_DISCONNECTED;
			xSemaphoreGive(pppos_mutex);
			break;
		}
		case PPPERR_CONNECT: {
			#if PPPOS_DEBUG
			ESP_LOGE(TAG,"status_cb: Connection lost");
			#endif
			xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
			pppos_client_status = PPPOS_CLIENT_STATE_DISCONNECTED;
			xSemaphoreGive(pppos_mutex);
			break;
		}
		case PPPERR_AUTHFAIL: {
			#if PPPOS_DEBUG
			ESP_LOGE(TAG,"status_cb: Failed authentication challenge");
			#endif
			break;
		}
		case PPPERR_PROTOCOL: {
			#if PPPOS_DEBUG
			ESP_LOGE(TAG,"status_cb: Failed to meet protocol");
			#endif
			break;
		}
		case PPPERR_PEERDEAD: {
			#if PPPOS_DEBUG
			ESP_LOGE(TAG,"status_cb: Connection timeout");
			#endif
			break;
		}
		case PPPERR_IDLETIMEOUT: {
			#if PPPOS_DEBUG
			ESP_LOGE(TAG,"status_cb: Idle Timeout");
			#endif
			break;
		}
		case PPPERR_CONNECTTIME: {
			#if PPPOS_DEBUG
			ESP_LOGE(TAG,"status_cb: Max connect time reached");
			#endif
			break;
		}
		case PPPERR_LOOPBACK: {
			#if PPPOS_DEBUG
			ESP_LOGE(TAG,"status_cb: Loopback detected");
			#endif
			break;
		}
		default: {
			#if PPPOS_DEBUG
			ESP_LOGE(TAG,"status_cb: Unknown error code %d", err_code);
			#endif
			break;
		}
	}
}

// === Handle sending data to MODULE modem ===
//------------------------------------------------------------------------------
static u32_t ppp_output_callback(ppp_pcb *pcb, u8_t *data, u32_t len, void *ctx)
{
	uint32_t ret = uart_write_bytes(uart_num, (const char*)data, len);
    uart_wait_tx_done(uart_num, 10 / portTICK_PERIOD_MS);
    if (ret > 0) {
		xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
    	pppos_rx_count += ret;
		xSemaphoreGive(pppos_mutex);
    }
    return ret;
}

//---------------------------------------------------------
static void infoCommand(char *cmd, int cmdSize, char *info)
{
	char buf[cmdSize+2];
	memset(buf, 0, cmdSize+2);

	for (int i=0; i<cmdSize;i++) {
		if ((cmd[i] != 0x00) && ((cmd[i] < 0x20) || (cmd[i] > 0x7F))) buf[i] = '.';
		else buf[i] = cmd[i];
		if (buf[i] == '\0') break;
	}
	ESP_LOGI(TAG,"%s [%s]", info, buf);
}

//----------------------------------------------------------------------------------------------------------------------
static int atCmd_waitResponse(char * cmd, char *resp, char * resp1, int cmdSize, int timeout, char **response, int size)
{
	char sresp[256] = {'\0'};
	char data[256] = {'\0'};
    int len, res = 1, idx = 0, tot = 0, timeoutCnt = 0;

	// ** Send command to MODULE
	vTaskDelay(100 / portTICK_PERIOD_MS);
	uart_flush(uart_num);

	if (cmd != NULL) {
		if (cmdSize == -1) cmdSize = strlen(cmd);
		#if PPPOS_DEBUG
		infoCommand(cmd, cmdSize, "AT COMMAND:");
		#endif
		uart_write_bytes(uart_num, (const char*)cmd, cmdSize);
		uart_wait_tx_done(uart_num, 100 / portTICK_PERIOD_MS);
	}

	if (response != NULL) {
		// Read MODULE response into buffer
		char *pbuf = *response;
		len = uart_read_bytes(uart_num, (uint8_t*)data, 256, timeout / portTICK_PERIOD_MS);
		while (len > 0) {
			if ((tot+len) >= size) {
				char *ptemp = realloc(pbuf, size+512);
				if (ptemp == NULL) return 0;
				size += 512;
				pbuf = ptemp;
			}
			memcpy(pbuf+tot, data, len);
			tot += len;
			response[tot] = '\0';
			len = uart_read_bytes(uart_num, (uint8_t*)data, 256, 100 / portTICK_PERIOD_MS);
		}
		*response = pbuf;
		return tot;
	}

    // ** Wait for and check the response
	idx = 0;
	while(1)
	{
		memset(data, 0, 256);
		len = 0;
		len = uart_read_bytes(uart_num, (uint8_t*)data, 256, 10 / portTICK_PERIOD_MS);
		if (len > 0) {
			for (int i=0; i<len;i++) {
				if (idx < 256) {
					if ((data[i] >= 0x20) && (data[i] < 0x80)) sresp[idx++] = data[i];
					else sresp[idx++] = 0x2e;
				}
			}
			tot += len;
		}
		else {
			if (tot > 0) {
				// Check the response
				if (strstr(sresp, resp) != NULL) {
					#if PPPOS_DEBUG
					ESP_LOGI(TAG,"AT RESPONSE: [%s]", sresp);
					#endif
					break;
				}
				else {
					if (resp1 != NULL) {
						if (strstr(sresp, resp1) != NULL) {
							#if PPPOS_DEBUG
							ESP_LOGI(TAG,"AT RESPONSE (1): [%s]", sresp);
							#endif
							res = 2;
							break;
						}
					}
					// no match
					#if PPPOS_DEBUG
					ESP_LOGI(TAG,"AT BAD RESPONSE: [%s]", sresp);
					#endif
					res = 0;
					break;
				}
			}
		}

		timeoutCnt += 10;
		if (timeoutCnt > timeout) {
			// timeout
			#if PPPOS_DEBUG
			ESP_LOGE(TAG,"AT: TIMEOUT");
			#endif
			res = 0;
			break;
		}
	}

	return res;
}

//------------------------------------
static void _disconnect(uint8_t rfOff)
{
	int res = atCmd_waitResponse("AT\r\n", MODULE_OK_Str, NULL, 4, 1000, NULL, 0);
	if (res == 1) {
		if (rfOff) {
			cmd_Reg.timeoutMs = 10000;
			res = atCmd_waitResponse("AT+CFUN=4\r\n", MODULE_OK_Str, NULL, 11, 10000, NULL, 0); // disable RF function
		}
		return;
	}

	#if PPPOS_DEBUG
	ESP_LOGI(TAG,"ONLINE, DISCONNECTING...");
	#endif
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	uart_flush(uart_num);
	uart_write_bytes(uart_num, "+++", 3);
    uart_wait_tx_done(uart_num, 10 / portTICK_PERIOD_MS);
	vTaskDelay(1100 / portTICK_PERIOD_MS);

	int n = 0;
	res = atCmd_waitResponse("ATH\r\n", MODULE_OK_Str, "NO CARRIER", 5, 3000, NULL, 0);
	while (res == 0) {
		n++;
		if (n > 10) {
			#if PPPOS_DEBUG
			ESP_LOGI(TAG,"STILL CONNECTED.");
			#endif
			n = 0;
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			uart_flush(uart_num);
			uart_write_bytes(uart_num, "+++", 3);
		    uart_wait_tx_done(uart_num, 10 / portTICK_PERIOD_MS);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
		vTaskDelay(100 / portTICK_PERIOD_MS);
		res = atCmd_waitResponse("ATH\r\n", MODULE_OK_Str, "NO CARRIER", 5, 3000, NULL, 0);
	}
	vTaskDelay(100 / portTICK_PERIOD_MS);
	if (rfOff) {
		cmd_Reg.timeoutMs = 10000;
		res = atCmd_waitResponse("AT+CFUN=4\r\n", MODULE_OK_Str, NULL, 11, 3000, NULL, 0);
	}
	#if PPPOS_DEBUG
	ESP_LOGI(TAG,"DISCONNECTED.");
	#endif
}

//----------------------------
static void enableAllInitCmd()
{
	for (int idx = 0; idx < MODULE_InitCmdsSize; idx++) {
		MODULE_Init[idx]->skip = 0;
	}
}

/*
 * PPPoS TASK
 * Handles MODULE initialization, disconnects and MODULE modem responses
 */
//-----------------------------
static void pppos_client_task()
{
	xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
	pppos_task_started = 1;
	xSemaphoreGive(pppos_mutex);

    // Allocate receive buffer
    char* data = (char*) malloc(BUF_SIZE);
    if (data == NULL) {
		#if PPPOS_DEBUG
		ESP_LOGE(TAG,"Failed to allocate data buffer.");
		#endif
    	goto exit;
    }

    if (gpio_set_direction(UART_GPIO_TX, GPIO_MODE_OUTPUT)) goto exit;
	if (gpio_set_direction(UART_GPIO_RX, GPIO_MODE_INPUT)) goto exit;
	if (gpio_set_pull_mode(UART_GPIO_RX, GPIO_PULLUP_ONLY)) goto exit;

	char PPP_ApnATReq[sizeof(CONFIG_MODULE_APN)+24];
	
	uart_config_t uart_config = {
			.baud_rate = UART_BDRATE,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.source_clk = UART_SCLK_DEFAULT,
	};

	//Configure UART1 parameters
	if (uart_param_config(uart_num, &uart_config)) goto exit;
	//Set UART1 pins(TX, RX, RTS, CTS)
	if (uart_set_pin(uart_num, UART_GPIO_TX, UART_GPIO_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)) goto exit;
	if (uart_driver_install(uart_num, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0)) goto exit;

	// Set APN from config
	sprintf(PPP_ApnATReq, "AT+CGDCONT=1,\"IP\",\"%s\"\r\n", CONFIG_MODULE_APN);
	cmd_APN.cmd = PPP_ApnATReq;
	cmd_APN.cmdSize = strlen(PPP_ApnATReq);

	_disconnect(1); // Disconnect if connected

	xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
    pppos_tx_count = 0;
    pppos_rx_count = 0;
	pppos_client_status = PPPOS_CLIENT_STATE_FIRSTINIT;
	xSemaphoreGive(pppos_mutex);

	enableAllInitCmd();

	while(1)
	{
		#if PPPOS_DEBUG
		ESP_LOGI(TAG,"MODULE initialization start");
		#endif
		vTaskDelay(500 / portTICK_PERIOD_MS);

		int moduleCmdIter = 0;
		int nfail = 0;
		// * MODULE Initialization loop
		while(moduleCmdIter < MODULE_InitCmdsSize)
		{
			if (MODULE_Init[moduleCmdIter]->skip) {
				#if PPPOS_DEBUG
				infoCommand(MODULE_Init[moduleCmdIter]->cmd, MODULE_Init[moduleCmdIter]->cmdSize, "Skip command:");
				#endif
				moduleCmdIter++;
				continue;
			}
			if (atCmd_waitResponse(MODULE_Init[moduleCmdIter]->cmd,
					MODULE_Init[moduleCmdIter]->cmdResponseOnOk, NULL,
					MODULE_Init[moduleCmdIter]->cmdSize,
					MODULE_Init[moduleCmdIter]->timeoutMs, NULL, 0) == 0)
			{
				// * No response or not as expected, start from first initialization command
				#if PPPOS_DEBUG
				ESP_LOGW(TAG,"Wrong response, restarting...");
				#endif

				nfail++;
				if (nfail > 20) goto exit;

				vTaskDelay(3000 / portTICK_PERIOD_MS);
				moduleCmdIter = 0;
				continue;
			}

			if (MODULE_Init[moduleCmdIter]->delayMs > 0) vTaskDelay(MODULE_Init[moduleCmdIter]->delayMs / portTICK_PERIOD_MS);
			MODULE_Init[moduleCmdIter]->skip = 1;
			if (MODULE_Init[moduleCmdIter] == &cmd_Reg) MODULE_Init[moduleCmdIter]->delayMs = 0;
			// Next command
			moduleCmdIter++;
		}

		#if PPPOS_DEBUG
		ESP_LOGI(TAG,"MODULE initialized.");
		#endif

		xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
		if (pppos_client_status == PPPOS_CLIENT_STATE_FIRSTINIT) {
			xSemaphoreGive(pppos_mutex);
			// ** After first successful initialization create PPP control block
			ppp = pppapi_pppos_create(&ppp_netif,
					ppp_output_callback, ppp_status_cb, NULL);

			if (ppp == NULL) {
				#if PPPOS_DEBUG
				ESP_LOGE(TAG, "Error initializing PPPoS");
				#endif
				break; // end task
			}
			//netif_set_default(&ppp_netif);
		}
		else xSemaphoreGive(pppos_mutex);

		pppapi_set_default(ppp);
		ppp_set_auth(ppp, PPPAUTHTYPE_PAP, PPP_User, PPP_Pass);
		ppp_set_usepeerdns(ppp, 1);
		//pppapi_set_auth(ppp, PPPAUTHTYPE_NONE, PPP_User, PPP_Pass);

		xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
		pppos_client_status = PPPOS_CLIENT_STATE_IDLE;
		xSemaphoreGive(pppos_mutex);
		pppapi_connect(ppp, 0);

		// *** LOOP: Handle MODULE modem responses & disconnects ***
		while(1) {
			// === Check if disconnect requested ===
			xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
			if (do_pppos_connect <= 0) {
				int end_task = do_pppos_connect;
				do_pppos_connect = 1;
				xSemaphoreGive(pppos_mutex);
				#if PPPOS_DEBUG
				printf("\r\n");
				ESP_LOGI(TAG, "Disconnect requested.");
				#endif

				pppapi_close(ppp, 0);
				int gstat = 1;
				while (pppos_client_status != PPPOS_CLIENT_STATE_DISCONNECTED) {
					// Handle data received from MODULE
					memset(data, 0, BUF_SIZE);
					int len = uart_read_bytes(uart_num, (uint8_t*)data, BUF_SIZE, 30 / portTICK_PERIOD_MS);
					if (len > 0)	{
						pppos_input_tcpip(ppp, (u8_t*)data, len);
						xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
					    pppos_tx_count += len;
						xSemaphoreGive(pppos_mutex);
					}
					xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
					gstat = pppos_client_status;
					xSemaphoreGive(pppos_mutex);
				}
				vTaskDelay(1000 / portTICK_PERIOD_MS);

				xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
				uint8_t rfoff = module_rfOff;
				xSemaphoreGive(pppos_mutex);
				_disconnect(rfoff); // Disconnect MODULE if still connected

				#if PPPOS_DEBUG
				ESP_LOGI(TAG, "Disconnected.");
				#endif

				moduleCmdIter = 0;
				enableAllInitCmd();
				xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
				pppos_client_status = PPPOS_CLIENT_STATE_IDLE;
				do_pppos_connect = 0;
				xSemaphoreGive(pppos_mutex);

				if (end_task < 0) goto exit;

				// === Wait for reconnect request ===
				gstat = 0;
				while (gstat == 0) {
					vTaskDelay(100 / portTICK_PERIOD_MS);
					xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
					gstat = do_pppos_connect;
					xSemaphoreGive(pppos_mutex);
				}
				#if PPPOS_DEBUG
				printf("\r\n");
				ESP_LOGI(TAG, "Reconnect requested.");
				#endif
				break;
			}

			// === Check if disconnected ===
			if (pppos_client_status == PPPOS_CLIENT_STATE_DISCONNECTED) {
				xSemaphoreGive(pppos_mutex);
				#if PPPOS_DEBUG
				printf("\r\n");
				ESP_LOGE(TAG, "Disconnected, trying again...");
				#endif
				pppapi_close(ppp, 0);

				enableAllInitCmd();
				moduleCmdIter = 0;
				pppos_client_status = PPPOS_CLIENT_STATE_IDLE;
				vTaskDelay(10000 / portTICK_PERIOD_MS);
				break;
			}
			else xSemaphoreGive(pppos_mutex);

			// === Handle data received from MODULE ===
			memset(data, 0, BUF_SIZE);
			int len = uart_read_bytes(uart_num, (uint8_t*)data, BUF_SIZE, 30 / portTICK_PERIOD_MS);
			if (len > 0)	{
				pppos_input_tcpip(ppp, (u8_t*)data, len);
				xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
			    pppos_tx_count += len;
				xSemaphoreGive(pppos_mutex);
			}

		}  // Handle MODULE modem responses & disconnects loop
	}  // main task loop

exit:
	if (data) free(data);  // free data buffer
	if (ppp) ppp_free(ppp);

	xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
	pppos_task_started = 0;
	pppos_client_status = PPPOS_CLIENT_STATE_FIRSTINIT;
	xSemaphoreGive(pppos_mutex);
	#if PPPOS_DEBUG
	ESP_LOGE(TAG, "PPPoS TASK TERMINATED");
	#endif
	vTaskDelete(NULL);
}

//=============
int ppposClientInit()
{
	if (pppos_mutex != NULL) xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
	do_pppos_connect = 1;
	int gstat = 0;
	int task_s = pppos_task_started;
	if (pppos_mutex != NULL) xSemaphoreGive(pppos_mutex);

	if (task_s == 0) {
		if (pppos_mutex == NULL) pppos_mutex = xSemaphoreCreateMutex();
		if (pppos_mutex == NULL) return 0;

		if (esp_netif_initialized == 0) {
			esp_netif_init();
			esp_netif_initialized = 1;
		}
		xTaskCreate(&pppos_client_task, "pppos_client_task", PPPOS_CLIENT_STACK_SIZE, NULL, 10, NULL);
		while (task_s == 0) {
			vTaskDelay(10 / portTICK_PERIOD_MS);
			xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
			task_s = pppos_task_started;
			xSemaphoreGive(pppos_mutex);
		}
	}

	while (gstat != 1) {
		vTaskDelay(10 / portTICK_PERIOD_MS);
		xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
		gstat = pppos_client_status;
		task_s = pppos_task_started;
		xSemaphoreGive(pppos_mutex);
		if (task_s == 0) return 0;
	}

	return 1;
}

//===================================================
void ppposClientDisconnect(uint8_t end_task, uint8_t rfoff)
{
	xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
	int gstat = pppos_client_status;
	xSemaphoreGive(pppos_mutex);

	if (gstat == PPPOS_CLIENT_STATE_IDLE) return;

	gstat = 0;

	vTaskDelay(2000 / portTICK_PERIOD_MS);
	xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
	if (end_task) do_pppos_connect = -1;
	else do_pppos_connect = 0;
	module_rfOff = rfoff;
	xSemaphoreGive(pppos_mutex);

	while (gstat == 0) {
		xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
		gstat = do_pppos_connect;
		xSemaphoreGive(pppos_mutex);
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
	while (gstat != 0) {
		vTaskDelay(100 / portTICK_PERIOD_MS);
		xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
		gstat = do_pppos_connect;
		xSemaphoreGive(pppos_mutex);
	}
}

//===================
int ppposClientStatus()
{
	xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
	int gstat = pppos_client_status;
	xSemaphoreGive(pppos_mutex);

	return gstat;
}

//========================================================
void getRxTxCount(uint32_t *rx, uint32_t *tx, uint8_t rst)
{
	xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
	*rx = pppos_rx_count;
	*tx = pppos_tx_count;
	if (rst) {
		pppos_rx_count = 0;
		pppos_tx_count = 0;
	}
	xSemaphoreGive(pppos_mutex);
}

//===================
void resetRxTxCount()
{
	xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
	pppos_rx_count = 0;
	pppos_tx_count = 0;
	xSemaphoreGive(pppos_mutex);
}
