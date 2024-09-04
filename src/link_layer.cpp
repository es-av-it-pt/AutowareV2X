#include "autoware_v2x/link_layer.hpp"
#include "autoware_v2x/cube_evk_link.hpp"
#include <boost/asio/generic/raw_protocol.hpp>
#include <vanetza/access/ethertype.hpp>
#include "autoware_v2x/raw_socket_link.hpp"

std::unique_ptr<LinkLayer> create_link_layer(
  boost::asio::io_service &io_service, const EthernetDevice &device, const std::string &name, const std::string &cube_ip)
{
  std::unique_ptr<LinkLayer> link_layer;

  if (name == "ethernet") {
    boost::asio::generic::raw_protocol raw_protocol(
      AF_PACKET, vanetza::access::ethertype::GeoNetworking.net());
    boost::asio::generic::raw_protocol::socket raw_socket(io_service, raw_protocol);
    raw_socket.bind(device.endpoint(AF_PACKET));
    link_layer.reset(new RawSocketLink{std::move(raw_socket)});
  } else if (name == "cube-evk") {
    link_layer.reset(new CubeEvkLink{ io_service, boost::asio::ip::address_v4::from_string(cube_ip) });
  }

  // Removed Cohda and UDP Support

  return link_layer;
}
