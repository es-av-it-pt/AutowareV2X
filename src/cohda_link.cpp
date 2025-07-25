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

#include <linux/cohda/llc/llc-api.h>

#include <cassert>
#include <iostream>
#include <memory>

using namespace vanetza;

void CohdaLink::request(const vanetza::access::DataRequest& request, std::unique_ptr<vanetza::ChunkPacket> packet)
{
  access::G5LinkLayer link_layer;
  access::ieee802::dot11::QosDataHeader& mac_header = link_layer.mac_header;
  mac_header.destination = request.destination_addr;
  mac_header.source = request.source_addr;
  mac_header.qos_control.user_priority(request.access_category);

  ByteBuffer link_layer_buffer;
  serialize_into_buffer(link_layer, link_layer_buffer);
  assert(link_layer_buffer.size() == access::G5LinkLayer::length_bytes);
  packet->layer(OsiLayer::Link) = std::move(link_layer_buffer);

  const std::size_t payload_size = packet->size();
  const std::size_t total_size = sizeof(tMKxTxPacket) + payload_size;

  tMKxTxPacket phy = { 0 };
  phy.Hdr.Type = MKXIF_TXPACKET;
  phy.Hdr.Len = total_size;
  phy.TxPacketData.TxAntenna = MKX_ANT_DEFAULT;
  phy.TxPacketData.TxFrameLength = payload_size;
  auto phy_ptr = reinterpret_cast<const uint8_t*>(&phy);
  packet->layer(OsiLayer::Physical) = ByteBuffer { phy_ptr, phy_ptr + sizeof(tMKxTxPacket) };

  transmit(std::move(packet));
}

boost::optional<vanetza::EthernetHeader> CohdaLink::parse_ethernet_header(vanetza::CohesivePacket& packet) const
{
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
  // if (!access::check_fixed_fields(link_layer)) {
  //     return boost::none;
  // }

  EthernetHeader eth;
  eth.destination = link_layer.mac_header.destination;
  eth.source = link_layer.mac_header.source;
  eth.type = link_layer.llc_snap_header.protocol_id;
  return eth;
}
