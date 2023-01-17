#ifndef _PPPOS_CLIENT_H_
#define _PPPOS_CLIENT_H_

#define PPPOS_CLIENT_STATE_DISCONNECTED	0
#define PPPOS_CLIENT_STATE_CONNECTED		1
#define PPPOS_CLIENT_STATE_IDLE			2
#define PPPOS_CLIENT_STATE_FIRSTINIT		3

extern struct netif ppp_netif;

/*
 * Create PPPoS task if not already created
 * Initialize module and connect to Internet
 * Handle all PPPoS requests
 * Disconnect/Reconnect from/to Internet on user request
 */
//==============
int ppposClientInit();

/*
 * Disconnect from Internet
 * If 'end_task' = 1 also terminate PPPoS task
 * If 'rfoff' = 1, turns off GSM RF section to preserve power
 * If already disconnected, this function does nothing
 */
//====================================================
void ppposClientDisconnect(uint8_t end_task, uint8_t rfoff);

/*
 * Get transmitted and received bytes count
 * If 'rst' = 1, resets the counters
 */
//=========================================================
void getRxTxCount(uint32_t *rx, uint32_t *tx, uint8_t rst);

/*
 * Resets transmitted and received bytes counters
 */
//====================
void resetRxTxCount();

/*
 * Get Task status
 *
 * Result:
 * PPPOS_CLIENT_STATE_DISCONNECTED	(0)		Disconnected from Internet
 * PPPOS_CLIENT_STATE_CONNECTED		(1)		Connected to Internet
 * PPPOS_CLIENT_STATE_IDLE			(2)	Disconnected from Internet, Task idle, waiting for reconnect request
 * PPPOS_CLIENT_STATE_FIRSTINIT		(3)	Task started, initializing PPPoS
 */
//================
int ppposClientStatus();

#endif /* _PPPOS_CLIENT_H_ */
