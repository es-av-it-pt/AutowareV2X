#ifndef COHDA_LINK_HPP_IXOCQ5RH
#define COHDA_LINK_HPP_IXOCQ5RH

#include "autoware_v2x/link_layer.hpp"
#include "ublox/ublox.h"

class CohdaLink : public LinkLayer {
public:
  CohdaLink();
  void request(const vanetza::access::DataRequest&, std::unique_ptr<vanetza::ChunkPacket>) override;
  void indicate(IndicationCallback callback) override;

private:
  it2s_ublox_t* ublox;
  pthread_t rx_thread;
  void rx_callback(it2s_ublox_t *amqp, void* user_data, unsigned char* buf, int packet_len);
  IndicationCallback indicate_to_router_;
};

#endif /* COHDA_LINK_HPP_IXOCQ5RH */
