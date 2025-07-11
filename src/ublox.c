#include <linux/cohda/llc/llc-api.h>
#include <linux/cohda/llc/llc.h>
#include "ublox/debug-levels.h"
#include "ublox/mk2mac-api-types.h"
#include "ublox/ublox.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <time.h>
#include <poll.h>
#include <unistd.h>
#include <errno.h>

D_LEVEL_DECLARE();

static tMKxStatus LLC_RxAlloc(struct MKx* pMKx, int BufLen, uint8_t** ppBuf, void** ppPriv) {
    int result = MKXSTATUS_FAILURE_INTERNAL_ERROR;
    *ppBuf = malloc(BufLen);
    if (*ppBuf != NULL)
        result = MKXSTATUS_SUCCESS;
    return result;
}

static tMKxStatus LLC_RxInd(struct MKx* pMKx, tMKxRxPacket* pMsg, void* pPriv) {
    int result = MKXSTATUS_FAILURE_INTERNAL_ERROR;
    it2s_ublox_t *ublox = (it2s_ublox_t*) pMKx->pPriv;
    ublox->on_packet(ublox, ublox->userData, pMsg->RxPacketData.RxFrame, pMsg->RxPacketData.RxFrameLength);
    free(pMsg);
    return result;
}

static void* MKxRecvFn(it2s_ublox_t *ublox) {
#define RX_IF_MKX (0)
#define RX_IF_CNT (RX_IF_MKX + 1)
    struct pollfd FDs[RX_IF_CNT] = {{
        -1,
    }};

    FDs[RX_IF_MKX].fd = ublox->Fd;
    int errorCode = 0;
    if (FDs[RX_IF_MKX].fd >= 0) {
        const int POLL_INPUT = (POLLIN | POLLPRI);
        const int POLL_ERROR = (POLLERR | POLLHUP | POLLNVAL);
        FDs[RX_IF_MKX].events = POLL_INPUT;

        while (ublox->Exit == false) {
            int Data = poll(FDs, RX_IF_CNT, 1000);
            if (Data < 0) {
                errorCode = -errno;
                ublox->Exit = true;
            }
            else {
                if (FDs[RX_IF_MKX].revents & POLL_ERROR) {
                }
                if (FDs[RX_IF_MKX].revents & POLL_INPUT) {
                    errorCode = MKx_Recv(ublox->pMKx);
                }
            }
        }
    }

    return NULL;
}

it2s_ublox_t *it2s_ublox_init(int channel, char radio, int channel_config, int antenna, char* mcs, int power) {
    char cmd[256];
    sprintf(cmd, "/opt/cohda/bin/llc chconfig -s -w CCH -c %d -r%c -a3 -bMK2BW_10MHz -p%d -e0x8947 -m%s -iwave-raw > /dev/null", channel, radio, power, mcs);
    system(cmd);

    D_LEVEL_INIT();

    it2s_ublox_t *ublox = calloc(1, sizeof(it2s_ublox_t));

    ublox->config.channel = channel;
    if (radio == 'a') {
        ublox->config.radio = 0;
    } else {
        ublox->config.radio = 1;
    }
    ublox->config.channel_config = channel_config;
    ublox->config.tx_antenna = antenna;

    if (!strcmp(mcs, "MK2MCS_R12BPSK")) {
        ublox->config.mcs = MK2MCS_R12BPSK;
    } else {
        ublox->config.mcs = 0; 
    }
    ublox->config.power = power;

    int result = MKx_Init(0, &(ublox->pMKx));
    ublox->Fd = MKx_Fd(ublox->pMKx);
    ublox->pMKx->API.Callbacks.RxInd = LLC_RxInd;
    ublox->pMKx->API.Callbacks.RxAlloc = LLC_RxAlloc;
    ublox->pMKx->pPriv = (void*)ublox;
    ublox->Exit = 0;

    return ublox;
}

void* it2s_ublox_loop(void *arg) {
    it2s_ublox_t* ublox = (it2s_ublox_t*) arg;
    MKxRecvFn(ublox);

    while (ublox->Exit == false) {
        sleep(1);
    }

    return NULL;
}

void it2s_ublox_stop(it2s_ublox_t *ublox) {
    ublox->Exit = true;
}

void it2s_ublox_user_data_set(it2s_ublox_t *ublox, void *obj) {
    ublox->userData = obj;
}

void it2s_ublox_rx_packet_callback_set(it2s_ublox_t *ublox, void (* on_packet) (struct it2s_ublox*, void *obj, uint8_t* packet, uint16_t packet_len)) {
    ublox->on_packet = on_packet;
}

void it2s_ublox_free(it2s_ublox_t *ublox) {
    free(ublox);
}

int it2s_ublox_tx_packet(it2s_ublox_t *ublox, it2s_ublox_config_t* config, uint8_t* packet, uint16_t packet_len) {
    struct MKxTxPacket *pMsg = calloc(1, sizeof(struct MKxTxPacket) + packet_len);

    if (!config) {
        pMsg->TxPacketData.MCS = MK2MCS_DEFAULT;
        pMsg->TxPacketData.TxPower = MKX_POWER_TX_DEFAULT;
        pMsg->TxPacketData.TxAntenna = ublox->config.tx_antenna;
        pMsg->TxPacketData.ChannelID = ublox->config.channel_config;
        pMsg->TxPacketData.RadioID = ublox->config.radio;
    } else {
        pMsg->TxPacketData.RadioID = config->radio;
        pMsg->TxPacketData.ChannelID = config->channel_config;
        pMsg->TxPacketData.MCS = config->mcs;
        pMsg->TxPacketData.TxAntenna = config->tx_antenna;
        pMsg->TxPacketData.TxPower = config->power;
    }

    memcpy(pMsg->TxPacketData.TxFrame, packet, packet_len);
    pMsg->TxPacketData.TxFrameLength = packet_len;

    int Res = ublox->pMKx->API.Functions.TxReq(ublox->pMKx, pMsg, ublox);
    if (Res) {
        printf("[ublox] error transmitting packet");
    }

    free(pMsg);
    return 0;
}
