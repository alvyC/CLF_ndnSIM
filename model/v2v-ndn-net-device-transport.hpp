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

#ifndef V2V_NDN_NET_DEVICE_TRANSPORT_HPP
#define V2V_NDN_NET_DEVICE_TRANSPORT_HPP

#include <queue>

#include "ns3/ndnSIM/model/ndn-common.hpp"
#include "ns3/ndnSIM/NFD/daemon/face/transport.hpp"

#include "ns3/net-device.h"
#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/node.h"
#include "ns3/pointer.h"
#include "ns3/event-id.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/channel.h"

namespace ns3 {
namespace ndn {

/**
 * \ingroup ndn-face
 * \brief ndnSIM-specific transport
 */
class V2VNetDeviceTransport : public nfd::face::Transport
{
public:
  V2VNetDeviceTransport(Ptr<Node> node, const Ptr<NetDevice>& netDevice,
                        const std::string& localUri,
                        const std::string& remoteUri,
                        ::ndn::nfd::FaceScope scope = ::ndn::nfd::FACE_SCOPE_NON_LOCAL,
                        ::ndn::nfd::FacePersistency persistency = ::ndn::nfd::FACE_PERSISTENCY_PERSISTENT,
                        ::ndn::nfd::LinkType linkType = ::ndn::nfd::LINK_TYPE_AD_HOC);
                        //::ndn::nfd::LinkType linkType = ::ndn::nfd::LINK_TYPE_POINT_TO_POINT);

  ~V2VNetDeviceTransport();

  Ptr<NetDevice>
  GetNetDevice() const;

  virtual ssize_t
  getSendQueueLength() final;
  
  void
  sendBeacon();
  
  void
  sendNoOfNeighborToStrategy();
  
  void
  sendInterestFromQueue();
  
  void
  clearIsNeighbor();

  void
  deleteNeighbor(int nodeId);

private:
  virtual void
  doClose() override;

  virtual void
  doSend(Packet&& packet) override;

  void
  receiveFromNetDevice(Ptr<NetDevice> device,
                       Ptr<const ns3::Packet> p,
                       uint16_t protocol,
                       const Address& from, const Address& to,
                       NetDevice::PacketType packetType);

  Ptr<NetDevice> m_netDevice; ///< \brief Smart pointer to NetDevice
  Ptr<Node> m_node;

  bool isNeighbor;
  ns3::EventId clearIsNeighborEid;
  std::queue<Ptr<ns3::Packet>> interestQueue;

  std::map<int, ns3::EventId> listOfNeighbor;

  static const int NeighExpTimer;
  static const int BeaconTimer;
  static const std::string STRATEGY;
};

} // namespace ndn
} // namespace ns3

#endif // NDN_NULL_TRANSPORT_HPP