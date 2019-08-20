/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011-2018  Regents of the University of California.
 *
 * This file is part of ndnSIM. See AUTHORS for complete list of ndnSIM authors and
 * contributors.
 *
 * ndnSIM is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * ndnSIM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ndnSIM, e.g., in COPYING.md file.  If not, see <http://www.gnu.org/licenses/>.
 **/

#include "v2v-ndn-net-device-transport.hpp"

#include "../helper/ndn-stack-helper.hpp"
#include "ndn-block-header.hpp"
#include "../utils/ndn-ns3-packet-tag.hpp"

#include <ndn-cxx/encoding/block.hpp>
#include <ndn-cxx/interest.hpp>
#include <ndn-cxx/data.hpp>
#include <ndn-cxx/lp/packet.hpp>
#include <ndn-cxx/lp/location-header.hpp>
#include <ndn-cxx/lp/fields.hpp>

#include "ns3/queue.h"
#include "ns3/mobility-model.h"

NS_LOG_COMPONENT_DEFINE("ndn.V2VNetDeviceTransport");

namespace ns3 {
namespace ndn {

V2VNetDeviceTransport::V2VNetDeviceTransport(Ptr<Node> node,
                                       const Ptr<NetDevice>& netDevice,
                                       const std::string& localUri,
                                       const std::string& remoteUri,
                                       ::ndn::nfd::FaceScope scope,
                                       ::ndn::nfd::FacePersistency persistency,
                                       ::ndn::nfd::LinkType linkType)
  : m_netDevice(netDevice)
  , m_node(node)
{
  this->setLocalUri(FaceUri(localUri));
  this->setRemoteUri(FaceUri(remoteUri));
  this->setScope(scope);
  this->setPersistency(persistency);
  this->setLinkType(linkType);
  this->setMtu(m_netDevice->GetMtu()); // Use the MTU of the netDevice

  // Get send queue capacity for congestion marking
  PointerValue txQueueAttribute;
  if (m_netDevice->GetAttributeFailSafe("TxQueue", txQueueAttribute)) {
    Ptr<ns3::QueueBase> txQueue = txQueueAttribute.Get<ns3::QueueBase>();
    // must be put into bytes mode queue

    auto size = txQueue->GetMaxSize();
    if (size.GetUnit() == BYTES) {
      this->setSendQueueCapacity(size.GetValue());
    }
    else {
      // don't know the exact size in bytes, guessing based on "standard" packet size
      this->setSendQueueCapacity(size.GetValue() * 1500);
    }
  }

  NS_LOG_FUNCTION(this << "Creating an V2V transport instance for netDevice with URI"
                  << this->getLocalUri());

  NS_ASSERT_MSG(m_netDevice != 0, "NetDeviceFace needs to be assigned a valid NetDevice");

  m_node->RegisterProtocolHandler(MakeCallback(&V2VNetDeviceTransport::receiveFromNetDevice, this),
                                  L3Protocol::ETHERNET_FRAME_TYPE, m_netDevice,
                                  true /*promiscuous mode*/);
}

V2VNetDeviceTransport::~V2VNetDeviceTransport()
{
  NS_LOG_FUNCTION_NOARGS();
}

ssize_t
V2VNetDeviceTransport::getSendQueueLength()
{
  PointerValue txQueueAttribute;
  if (m_netDevice->GetAttributeFailSafe("TxQueue", txQueueAttribute)) {
    Ptr<ns3::QueueBase> txQueue = txQueueAttribute.Get<ns3::QueueBase>();
    return txQueue->GetNBytes();
  }
  else {
    return nfd::face::QUEUE_UNSUPPORTED;
  }
}

void
V2VNetDeviceTransport::doClose()
{
  NS_LOG_FUNCTION(this << "Closing transport for netDevice with URI"
                  << this->getLocalUri());

  // set the state of the transport to "CLOSED"
  this->setState(nfd::face::TransportState::CLOSED);
}

void
V2VNetDeviceTransport::doSend(Packet&& packet)
{
  // convert Transport::Packet to Lp::Packet
  ::ndn::lp::Packet lpPacket(packet.packet);
  
  // get packet type  
  ::ndn::Buffer::const_iterator fragBegin, fragEnd;
  std::tie(fragBegin, fragEnd) = lpPacket.get<::ndn::lp::FragmentField>();
  Block netPkt(&*fragBegin, std::distance(fragBegin, fragEnd));
  auto pktType = netPkt.type();
  
  // get current location
  Ptr<MobilityModel> mobility = m_node->GetObject<MobilityModel> ();
  if (mobility == 0) {
    NS_FATAL_ERROR("Mobility model has to be installed on the node");
    return;
  }
  Vector3D currentPosition = mobility->GetPosition();
  
  // Get location header that will be used to get DestLocation from Forwarding Strategy
  // lpPacket to Interest packet and get the header info
  //auto interestPacket = make_shared<::ndn::Interest>(packet.packet);
  //shared_ptr<::ndn::lp::LocationTag> locationtag = interestPacket.getTag<::ndn::lp::LocationTag>(); 
  
  // the lp packet contains header then save the dest location field
  ::ndn::Location destLocation(0.0, 0.0); 
  if (netPkt.type() == ::ndn::tlv::Interest) {
    if(lpPacket.has<lp::LocationField>()) {
      NS_LOG_FUNCTION("Interest Packet contains Location Header");
    
      ::ndn::lp::LocationHeader lh_tmp = lpPacket.get<lp::LocationField>(0);
      
      destLocation = lh_tmp.getDestLocation();// Get DestLocation which was tagged by ForwardingStrategy
      
      NS_LOG_FUNCTION(this << "DestLocation field: " << destLocation.getLongitude() << ", " << destLocation.getLatitude()); 
      
      // remove the old header and add the new header
      lpPacket.remove<::ndn::lp::LocationField>(0);
    }
  }
  else if (netPkt.type() == ::ndn::tlv::Data) {
    if(lpPacket.has<lp::LocationField>()) {
       NS_LOG_FUNCTION("Data Packet contains Location Header");
    
      ::ndn::lp::LocationHeader lh_tmp = lpPacket.get<lp::LocationField>(0);
      
      destLocation = lh_tmp.getDestLocation();// Get DestLocation which was tagged by ForwardingStrategy
      
      NS_LOG_FUNCTION(this << "DestLocation field: " << destLocation.getLongitude() << ", " << destLocation.getLatitude()); 
      
      // remove the old header and add the new header
      lpPacket.remove<::ndn::lp::LocationField>(0);
    }
    else {
      NS_LOG_FUNCTION("Data packet does not contain Location Header. Attaching MyLocation as DestLocation to the Data Packet.");

      destLocation.setLongitude(currentPosition.x);
      destLocation.setLatitude(currentPosition.y);
    }
  }
  else {
    NS_LOG_FUNCTION("Packet type neither Interest nor Data. Packet type: " << netPkt.type());
  }

  // Construct the Location fields
  ::ndn::lp::LocationHeader lh;
  // x = longitude, y = latitude
  ::ndn::Location prevLocation(currentPosition.x, currentPosition.y);
  ::ndn::Location myLocation(0.0, 0.0); // dummy value
   
  // Populate PrevLocation and DestLoation Field (MyLocationField will have dumy value)) of LocationHeader
  lh.setMyLocation(myLocation);
  lh.setPrevLocation(prevLocation);
  lh.setDestLocation(destLocation);

  // add new header
  lpPacket.add<::ndn::lp::LocationField>(lh);

  // convert back to Transport::Packet
  Transport::Packet transportPacket(lpPacket.wireEncode());

  // convert NFD packet to NS3 packet
  BlockHeader header(transportPacket);
  Ptr<ns3::Packet> ns3Packet = Create<ns3::Packet>();
  ns3Packet->AddHeader(header);

  
    
  if (netPkt.type() == ::ndn::tlv::Interest) {
    NS_LOG_FUNCTION(this << "Sending Interest packet from netDevice with URI: "
                  << this->getLocalUri());
  }
  else if (netPkt.type() == ::ndn::tlv::Data) {
    NS_LOG_FUNCTION(this << "Sending Data packet from netDevice with URI: "
                  << this->getLocalUri());
  }
  else if (netPkt.type() == ::ndn::lp::tlv::LpPacket) {
    NS_LOG_FUNCTION(this << "Sending LpPacket packet from netDevice with URI: "
                  << this->getLocalUri());
  }
  else {
    NS_LOG_FUNCTION(this << "Sending packet from netDevice with URI: "
                  << this->getLocalUri() << packet.packet.type());
  } 

  // send the NS3 packet
  m_netDevice->Send(ns3Packet, m_netDevice->GetBroadcast(),
                    L3Protocol::ETHERNET_FRAME_TYPE);
}

// callback
void
V2VNetDeviceTransport::receiveFromNetDevice(Ptr<NetDevice> device,
                                            Ptr<const ns3::Packet> p,
                                            uint16_t protocol,
                                            const Address& from, const Address& to,
                                            NetDevice::PacketType packetType)
{
  NS_LOG_FUNCTION(device << p << protocol << from << to << packetType);

  // Convert NS3 packet to transport packet
  Ptr<ns3::Packet> packet = p->Copy();
  BlockHeader header;
  packet->RemoveHeader(header);
  auto transPacket = Packet(std::move(header.getBlock()));
  
  // convert Transport::Packet to Lp::Packet + get and save PrevLocation, DestLocation field
  //     + remove LocationHeader
  ::ndn::lp::Packet lpPacket(transPacket.packet);
  
  ::ndn::Location prevLocation(0.0, 0.0);
  ::ndn::Location destLocation(0.0, 0.0);
  if(lpPacket.has<lp::LocationField>()) {
    NS_LOG_FUNCTION("Packet contains Location Header");
    ::ndn::lp::LocationHeader lh_tmp = lpPacket.get<lp::LocationField>(0); 
    prevLocation = lh_tmp.getPrevLocation();
    destLocation = lh_tmp.getDestLocation();
    //::ndn::Location destLocation(250.0, 0.0); // for testing puspose, replace it with prev line
    
    // Remove the previous lp field
    lpPacket.remove<lp::LocationField>(0);
  }
  
  // get current position
  Ptr<MobilityModel> mobility = m_node->GetObject<MobilityModel> ();
  if (mobility == 0) {
    NS_FATAL_ERROR("Mobility model has to be installed on the node");
    return;
  }

  Vector3D currentPosition = mobility->GetPosition();

  ::ndn::lp::LocationHeader lh;
  ::ndn::Location myLocation(currentPosition.x, currentPosition.y);
  
  // Populate MyLocation, PrevLocation Field, DestLocation field  of LocationHeader
  lh.setMyLocation(myLocation);
  lh.setPrevLocation(prevLocation);
  lh.setDestLocation(destLocation);
  
  // add the header
  lpPacket.add<::ndn::lp::LocationField>(lh);

  NS_LOG_FUNCTION(this << "MyLocation field: " <<  myLocation.getLongitude()  << ", " << myLocation.getLatitude());
  NS_LOG_FUNCTION(this << "PrevLocation field: " << prevLocation.getLongitude() << ", " << prevLocation.getLatitude()); 
  NS_LOG_FUNCTION(this << "DestLocation field: " << destLocation.getLongitude() << ", " << destLocation.getLatitude()); 

  // convert back to Transport::Packet
  Transport::Packet transportPacket(lpPacket.wireEncode());
  this->receive(std::move(transportPacket));
}

Ptr<NetDevice>
V2VNetDeviceTransport::GetNetDevice() const
{
  return m_netDevice;
}

} // namespace ndn
} // namespace ns3
