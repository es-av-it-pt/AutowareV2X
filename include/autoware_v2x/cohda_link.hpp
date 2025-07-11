#ifndef COHDA_LINK_HPP_IXOCQ5RH
#define COHDA_LINK_HPP_IXOCQ5RH

#include "autoware_v2x/link_layer.hpp"

extern "C" {
#include "ublox/ublox.h"
}

class CohdaLink : public LinkLayer {
public:
  CohdaLink();
  void request(const vanetza::access::DataRequest&, std::unique_ptr<vanetza::ChunkPacket>) override;
  void indicate(IndicationCallback callback) override;

protected:
  virtual boost::optional<vanetza::EthernetHeader> parse_ethernet_header(vanetza::CohesivePacket&) const;

private:
  it2s_ublox_t* ublox;
  pthread_t rx_thread;
  static void rx_callback(it2s_ublox_t *ublox, void* user_data, uint8_t* buf, uint16_t packet_len);
  IndicationCallback indicate_to_router_;
};

#endif /* COHDA_LINK_HPP_IXOCQ5RH */
