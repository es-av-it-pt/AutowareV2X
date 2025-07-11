#include "autoware_v2x/cohda_link.hpp"

#include <pthread.h>
#include <vanetza/access/access_category.hpp>
#include <vanetza/access/data_request.hpp>
#include <vanetza/access/g5_link_layer.hpp>
#include <vanetza/common/byte_buffer.hpp>
#include <vanetza/common/clock.hpp>
#include <vanetza/common/serialization_buffer.hpp>
#include <vanetza/dcc/mapping.hpp>
#include <vanetza/net/chunk_packet.hpp>
#include <vanetza/net/cohesive_packet.hpp>
#include <vanetza/net/ethernet_header.hpp>

#include <boost/optional.hpp>
#include <boost/optional/optional.hpp>

#include <linux/cohda/llc/llc-api.h>

#include <cassert>
#include <iostream>
#include <memory>

using namespace vanetza;

void CohdaLink::rx_callback(it2s_ublox_t *amqp, void* user_data, unsigned char* buf, int packet_len) {
  // DEBUG 
  char *rx_hex = malloc(packet_len*2+2);
  char *buf_ptr = rx_hex;
  uint8_t *pkt_ptr = packet;
  for (int i = 0; i < packet_len; i++)
      buf_ptr += sprintf(buf_ptr, "%02x", pkt_ptr[i]);
  printf("[ublox]<- received packet |size:%dB data:%.*s", packet_len, packet_len*2, rx_hex);
  free(rx_hex);

  /*
  // mac decap (look for vanetaza access methods)
  // llc decap (look for vanetaza access methods)
  // send to upper layers
  indicate_to_router_(std::move(packet), ethernet_header);
  */
}

boost::optional<vanetza::EthernetHeader> CohdaLink::parse_ethernet_header(vanetza::CohesivePacket& packet) const {
  static const std::size_t min_length = sizeof(tMKxRxPacket) + access::G5LinkLayer::length_bytes + access::ieee802::dot11::fcs_length_bytes;
  if (packet.size(OsiLayer::Physical) < min_length) {
      return boost::none;
  }

  packet.set_boundary(OsiLayer::Physical, sizeof(tMKxRxPacket));
  auto phy = reinterpret_cast<const tMKxRxPacket*>(&*packet[OsiLayer::Physical].begin());
  if (phy->Hdr.Type != MKXIF_RXPACKET) {
      return boost::none;
  }

  if (phy->Hdr.Len != packet.size() || phy->RxPacketData.RxFrameLength != packet.size() - sizeof(tMKxRxPacket)) {
      return boost::none;
  }

  if (!phy->RxPacketData.FCSPass) {
      return boost::none;
  }

  packet.trim(OsiLayer::Link, packet.size() - access::ieee802::dot11::fcs_length_bytes);
  packet.set_boundary(OsiLayer::Link, access::G5LinkLayer::length_bytes);
  access::G5LinkLayer link_layer;
  deserialize_from_range(link_layer, packet[OsiLayer::Link]);
  if (!access::check_fixed_fields(link_layer)) {
      return boost::none;
  }

  EthernetHeader eth;
  eth.destination = link_layer.mac_header.destination;
  eth.source = link_layer.mac_header.source;
  eth.type = link_layer.llc_snap_header.protocol_id;
  return eth;
}

void CohdaLink::CohdaLink() {
  // configs
  int channel = 180;
  char* radio = "a"
  int channel_config = 0;
  int antenna = 3;
  char* mcs = "MK2MCS_R12QPSK";
  int power = 46;

  // initialize
  this.ublox = it2s_ublox_init(channel, radio, channel_config, antenna, mcs, power);

  // set rx callback
  it2s_ublox_rx_packet_callback_set(this.ublox, this.rx_callback);

  // rx loop
  pthread_create(&this.ublox.rx_thread, NULL, (void*) it2s_ublox_loop, this.ublox);
}

void CohdaLink::request(const vanetza::access::DataRequest& request, std::unique_ptr<vanetza::ChunkPacket> packet) {
  // DEBUG
  std::string* payload = transmission->mutable_payload();
  for (auto& layer : osi_layer_range<OsiLayer::Network, OsiLayer::Application>()){
    auto byte_view = create_byte_view(packet->layer(layer));
    payload->append(byte_view.begin(), byte_view.end());
  }
  printf("[ublox]-> transmitting packet | size:%dB data:%s", packet->size(), payload);

  // mac header
  /*access::G5LinkLayer link_layer;
  access::ieee802::dot11::QosDataHeader& mac_header = link_layer.mac_header;
  mac_header.destination = request.destination_addr;
  mac_header.source = request.source_addr;
  mac_header.qos_control.user_priority(request.access_category);

  // serialize
  ByteBuffer link_layer_buffer;
  serialize_into_buffer(link_layer, link_layer_buffer);

  uint32_t whole_packet_len = link_layer_buffer->size() +packet->size();
  uint8_t* whole_packet = malloc(whole_packet_len);

  memcpy(whole_packet, link_layer_buffer->byte_buffer(), link_layer_buffer->size());
  memcpy(whole_packet+link_layer_buffer->size(), packet->byte_buffer(), packet->size());

  // send to air
  it2s_ublox_tx_packet(this.ublox, &this.ublox->config, whole_packet, whole_packet_len);*/
  
}

void CohdaLink::indicate(IndicationCallback callback) {
  this.indicate_to_router_ = callback;
}
