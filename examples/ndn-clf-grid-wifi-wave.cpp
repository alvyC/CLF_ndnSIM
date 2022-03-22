/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Copyright (c) 2011-2015  Regents of the University of California.
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

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"

#include "ns3/ndnSIM-module.h"

#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"

#include "ns3/ndnSIM/NFD/daemon/face/generic-link-service.hpp"
#include "model/v2v-ndn-net-device-transport.hpp"

using namespace std;
namespace ns3 {

NS_LOG_COMPONENT_DEFINE("ndn.WifiExample");

std::string
constructFaceUri(Ptr<NetDevice> netDevice)
{
  std::string uri = "netdev://";
  Address address = netDevice->GetAddress();
  if (Mac48Address::IsMatchingType(address)) {
    uri += "[" + boost::lexical_cast<std::string>(Mac48Address::ConvertFrom(address)) + "]";
  }

  return uri;
}

shared_ptr<ndn::Face>
V2VNetDeviceFaceCallback (Ptr<Node> node, Ptr<ndn::L3Protocol> ndn, 
			  Ptr<NetDevice> netDevice)
{
  NS_LOG_DEBUG("Creating V2V NetDevice Face on node " << node->GetId());

  // Create an ndnSIM-specific transport instance
  ::nfd::face::GenericLinkService::Options opts;
  opts.allowFragmentation = true;
  opts.allowReassembly = true;
  opts.allowCongestionMarking = true;

  auto linkService = make_unique<::nfd::face::GenericLinkService>(opts);

  auto v2vTransport = make_unique<ndn::V2VNetDeviceTransport>(node, netDevice,
                                                   constructFaceUri(netDevice),
                                                   "netdev://[ff:ff:ff:ff:ff:ff]");

  auto face = std::make_shared<ndn::Face>(std::move(linkService), std::move(v2vTransport));
  face->setMetric(1);

  ndn->addFace(face);
  NS_LOG_LOGIC("Node " << node->GetId() << ": added Face as face #"
                       << face->getLocalUri());
  
  return face;
}
//
// DISCLAIMER:  Note that this is an extremely simple example, containing just 2 wifi nodes
// communicating directly over AdHoc channel.
//

int
main(int argc, char* argv[])
{
  std::string phyMode ("OfdmRate6MbpsBW10MHz");
 
  // disable fragmentation
  /*
  Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue("2200"));
  Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue("2200"));
  Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode",
                     StringValue(phyMode));
  */
  
  CommandLine cmd;
  cmd.Parse(argc, argv);
  
  // the channel
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  //wifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel", "Exponent", DoubleValue (3.0)); // range = 120 
  double range = 100;
  wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel",
                                  "MaxRange", DoubleValue(range)); 
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  wifiPhy.SetChannel (channel);
 
  //wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
   
  // Setup WAVE PHY and MAC
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  
  // enable logging 
  //wifi80211p.EnableLogComponents (); 
  
  // Setup 802.11p stuff
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode", StringValue (phyMode),
                                      "ControlMode", StringValue (phyMode));

  //WifiMacHelper wifiMacHelper;
  //wifiMacHelper.SetType("ns3::AdhocWifiMac");
  
  MobilityHelper mobility;
  // transimission range is 46 
  //double distance = 250;
  //double distance = 30;
  double distance = 90;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (1.0),
                                 "MinY", DoubleValue (1.0),
                                 "DeltaX", DoubleValue (distance),
                                 "DeltaY", DoubleValue (distance),
                                 "GridWidth", UintegerValue (3),
                                 "LayoutType", StringValue ("RowFirst"));

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  int numOfNodes = 9;
  NodeContainer nodes;
  nodes.Create(numOfNodes);

  ////////////////
  // 1. Install Wifi
  //NetDeviceContainer wifiNetDevices = wifi.Install(wifiPhyHelper, wifiMacHelper, nodes);
  NetDeviceContainer wifiNetDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, nodes);
  
  // tracing
  //wifiPhy.EnablePcap ("ndn-clf-grid-80211p", wifiNetDevices);

  
  // 2. Install Mobility model
  mobility.Install(nodes);

  // 3. Install NDN stack
  NS_LOG_INFO("Installing NDN stack");
  ndn::StackHelper ndnHelper;
  // ndnHelper.AddNetDeviceFaceCreateCallback (WifiNetDevice::GetTypeId (), MakeCallback
  // (MyNetDeviceFaceCallback));
  ndnHelper.AddFaceCreateCallback (WifiNetDevice::GetTypeId (), MakeCallback(V2VNetDeviceFaceCallback));
  ndnHelper.SetOldContentStore("ns3::ndn::cs::Lru", "MaxSize", "1000");
  ndnHelper.SetDefaultRoutes(true);
  ndnHelper.Install(nodes);

  // Set BestRoute strategy
  ndn::StrategyChoiceHelper::Install(nodes, "/test", "/localhost/nfd/strategy/clf");
  //ndn::StrategyChoiceHelper::Install(nodes, "/", "/localhost/nfd/strategy/clf");
  //ndn::StrategyChoiceHelper::Install(nodes, "/", "/localhost/nfd/strategy/best-route");
  
  // 4. Set up applications
  NS_LOG_INFO("Installing Applications");

  //ndn::AppHelper consumerHelper("ns3::ndn::ConsumerCbr");
  ndn::AppHelper consumerHelper("ns3::ndn::ConsumerBatches");
  consumerHelper.SetPrefix("/test/prefix/a/b");
  //consumerHelper.SetAttribute("Frequency", DoubleValue(1.0));
  //consumerHelper.SetAttribute("MaxSeq", IntegerValue(5));
  //consumerHelper.SetAttribute("Batches", StringValue("1s 1 8s 1 14s 1 20s 1"));
  consumerHelper.SetAttribute("Batches", StringValue("2s 1 3s 1"));
  //consumerHelper.SetAttribute("Batches", StringValue("1s 1 2s 1 3s 1 4s 1 5s 1 6s 1 7s 1 8s 1 9s 1 10s 1"));
  consumerHelper.Install(nodes.Get(0));

  ndn::AppHelper producerHelper("ns3::ndn::ClfProducer");
  producerHelper.SetPrefix("/test/prefix");
  producerHelper.SetAttribute("PayloadSize", StringValue("1200"));
  producerHelper.Install(nodes.Get(numOfNodes - 1));
  /*for (int i = 0; i < 10; i++) {
    int nodeNo = rand() % numOfNodes;
    producerHelper.Install(nodes.Get(nodeNo));
  }*/

  ////////////////

  Simulator::Stop(Seconds(5.0));

  ndn::L3RateTracer::InstallAll("/vagrant/ndnSIM/16-static-grid-clf-rate-trace.txt", Seconds(5.0));
  L2RateTracer::InstallAll("/vagrant/ndnSIM/16-static-grid-clf-drop-trace.txt", Seconds(5.0));
  ndn::AppDelayTracer::InstallAll("/vagrant/ndnSIM/16-static-grid-clf-app-delays-trace.txt");
  
  Simulator::Run();
  Simulator::Destroy();

  return 0;
}

} // namespace ns3

int
main(int argc, char* argv[])
{
  return ns3::main(argc, argv);
}
