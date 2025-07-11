#include <stdint.h>

typedef struct it2s_ublox_config {
    int channel;
    char radio;
    int channel_config;
    int tx_antenna;
    int mcs;
    int power;
} it2s_ublox_config_t;

typedef struct it2s_ublox {
    struct MKx* pMKx;
    int Fd;
    int Exit;
    void (*on_packet)(struct it2s_ublox *, void *obj, uint8_t* packet, uint16_t packet_len);
    uint8_t srcMac[6];
    void *userData;

    it2s_ublox_config_t config;
} it2s_ublox_t;

it2s_ublox_t *it2s_ublox_init(int channel, char radio, int channel_config, int antenna, char* mcs, int power);
int it2s_ublox_loop(it2s_ublox_t *ublox);
void it2s_ublox_user_data_set(it2s_ublox_t *ublox, void *obj);
void it2s_ublox_rx_packet_callback_set(it2s_ublox_t *ublox, void(* on_packet) (struct it2s_ublox*, void *obj, uint8_t* packet, uint16_t packet_len));
int it2s_ublox_tx_packet(it2s_ublox_t *ublox, it2s_ublox_config_t *config, uint8_t* packet, uint16_t packet_len);
void it2s_ublox_free(it2s_ublox_t *ublox);
