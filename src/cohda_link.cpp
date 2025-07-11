#include "autoware_v2x/cohda_link.hpp"

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

extern "C" {
#include <pthread.h>
}

#include <cassert>
#include <iostream>
#include <memory>

using namespace vanetza;

boost::optional<EthernetHeader> CohdaLink::parse_ethernet_header(vanetza::CohesivePacket & packet) const {
  packet.set_boundary(OsiLayer::Physical, 0);
  if (packet.size(OsiLayer::Link) < EthernetHeader::length_bytes) {
    std::cerr << "Router dropped invalid packet (too short for Ethernet header)\n";
  } else {
    packet.set_boundary(OsiLayer::Link, EthernetHeader::length_bytes);
    auto link_range = packet[OsiLayer::Link];
    return decode_ethernet_header(link_range.begin(), link_range.end());
  }
  return boost::none;
}

void CohdaLink::rx_callback(it2s_ublox_t *ublox, void* user_data, uint8_t* buf, uint16_t packet_len){
  auto* self = static_cast<CohdaLink*>(ublox->userData);

  // debug 
  char *rx_hex = (char*)malloc(packet_len*2+2);
  char *buf_ptr = rx_hex;
  uint8_t *pkt_ptr = buf;
  for (int i = 0; i < packet_len; i++)
    buf_ptr += sprintf(buf_ptr, "%02x", pkt_ptr[i]);
  printf("[ublox]<- received packet |size:%dB data:%.*s", packet_len, packet_len*2, rx_hex);
  free(rx_hex);

  /* UNCOMMENT AFTER TESTING RX
  // parse into vanetza packet and send upwards
  ByteBuffer buffer(buf, buf + packet_len);
  CohesivePacket packet(std::move(buffer), OsiLayer::Physical);
  boost::optional<EthernetHeader> eth = self->parse_ethernet_header(packet);
  self->indicate_to_router_(std::move(packet), *eth);
  */
}

CohdaLink::CohdaLink() {
  // configs
  int channel = 180;
  char radio = 'a';
  int channel_config = 0;
  int antenna = 3;
  char* mcs = "MK2MCS_R12QPSK";
  int power = 46;

  // initialize
  this->ublox = it2s_ublox_init(channel, radio, channel_config, antenna, mcs, power);

  // set rx callback
  it2s_ublox_user_data_set(this->ublox, this);
  it2s_ublox_rx_packet_callback_set(this->ublox, CohdaLink::rx_callback);

  // rx loop
  pthread_create(&this->rx_thread, NULL, it2s_ublox_loop, this->ublox);
}

void CohdaLink::request(const vanetza::access::DataRequest& request, std::unique_ptr<vanetza::ChunkPacket> packet) {
  // mac header
  access::G5LinkLayer link_layer;
  access::ieee802::dot11::QosDataHeader& mac_header = link_layer.mac_header;
  mac_header.destination = request.destination_addr;
  mac_header.source = request.source_addr;
  mac_header.qos_control.user_priority(request.access_category);

  // append link layer
  ByteBuffer link_layer_buffer;
  serialize_into_buffer(link_layer, link_layer_buffer);
  packet->layer(OsiLayer::Link) = std::move(link_layer_buffer);

  // serialize
  vanetza::ByteBuffer full_buffer;
  for (auto& layer : osi_layer_range<OsiLayer::Link, OsiLayer::Application>()) {
    const auto& layer_data = packet->layer(layer);
    vanetza::ByteBuffer layer_buffer;
    layer_data.convert(layer_buffer);
    full_buffer.insert(full_buffer.end(), layer_buffer.begin(), layer_buffer.end());
  }
  uint8_t* whole_packet = full_buffer.data();
  uint32_t whole_packet_len = static_cast<uint32_t>(full_buffer.size());

  // send to air
  it2s_ublox_tx_packet(this->ublox, &this->ublox->config, whole_packet, whole_packet_len);

  // debug
  char *tx_hex = (char*) malloc(whole_packet_len * 2 + 2);
  char *buf_ptr = tx_hex;
  uint8_t *pkt_ptr = whole_packet;
  for (int i = 0; i < whole_packet_len; i++) {
    buf_ptr += sprintf(buf_ptr, "%02x", pkt_ptr[i]);
  }
  printf("[ublox]-> transmitting packet | size:%d data:%.*s\n", whole_packet_len, whole_packet_len*2, tx_hex);
  free(tx_hex);
}

void CohdaLink::indicate(IndicationCallback callback) {
  this->indicate_to_router_ = callback;
}
