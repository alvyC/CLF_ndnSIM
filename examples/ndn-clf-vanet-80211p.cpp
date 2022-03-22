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

#include "ns3/netanim-module.h"

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

std::vector<int>
getNodesFromList(std::string listOfNodes) {
  std::stringstream ss(listOfNodes);
  
  std::vector<int> vectorOfNodes;
  while( ss.good() )
  {
    std::string substr;
    std::getline(ss, substr, ',' );
    vectorOfNodes.push_back(std::stoi(substr));
  }

  return vectorOfNodes;
}

void
printNodes(std::vector<int> vectorOfNodes) {
  for (auto i : vectorOfNodes) {
    std::cout << i << " ";
  }
  std::cout << std::endl;
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
  /*Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue("2200"));
  Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue("2200"));
  Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode",
                     StringValue("OfdmRate24Mbps"));*/

  std::string l3FileName;
  std::string appDelayFileName;
  std::string l2FileName;
  string traceFile;
  double range;
  int numOfNodes;
  //std::string listOfCons;
  //std::string listOfProds; 
  int consumerNode;
  int producerNode;

  CommandLine cmd;
  cmd.AddValue ("l3FileName", "L3 Rate trace file name", l3FileName);
  cmd.AddValue ("appDelayFileName", "App Delay file name", appDelayFileName);
  cmd.AddValue ("l2FileName", "L2 Rate file name", l2FileName);
  cmd.AddValue("traceFile", "Tracefile Name", traceFile);
  cmd.AddValue("range", "Tx Range", range);
  cmd.AddValue("numOfNodes", "No. of Nodes", numOfNodes);
  //cmd.AddValue("listOfCons", "List of Consumer Nodes (Comma Separated)", listOfCons);
  //cmd.AddValue("listOfProds", "List of Producer Nodes (Comman Separated)", listOfProds);
  cmd.AddValue("consumerNode", "Consumer node's id", consumerNode);
  cmd.AddValue("producerNode", "Producer node's id", producerNode);

  cmd.Parse(argc, argv);

  std::cout << "L3 Rate trace file name: " << l3FileName << std::endl;
  std::cout << "App Delay file name: " << l3FileName << std::endl;
  std::cout << "L2 rate file name: " << l2FileName << std::endl;
  std::cout << "Trace file name: " << traceFile  << std::endl;
  std::cout << "Range: " << range << std::endl;
  std::cout << "No. of Nodes: " << numOfNodes << std::endl;
  std::cout << "Consumer node's id: " << consumerNode << std::endl;
  std::cout << "Producer node's id: " << producerNode << std::endl;
  //std::cout << "List of Consumer Nodes (Comma Separated): " << listOfCons << std::endl;
  //std::cout << "List of Producer Nodes (Comman Separated): " << listOfProds << std::endl;

  //////////////////////
  //////////////////////
  //////////////////////
  
  YansWifiChannelHelper wifiChannel; // = YansWifiChannelHelper::Default ();
  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel",
                                  "MaxRange", DoubleValue(range));
    
  // YansWifiPhy wifiPhy = YansWifiPhy::Default();
  YansWifiPhyHelper wifiPhyHelper = YansWifiPhyHelper::Default();
  wifiPhyHelper.SetChannel(wifiChannel.Create());
  
  // Setup WAVE PHY and MAC
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  
  // Setup 802.11p stuff
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode", StringValue (phyMode),
                                      "ControlMode", StringValue (phyMode));

  WifiMacHelper wifiMacHelper;
  wifiMacHelper.SetType("ns3::AdhocWifiMac");
  //std::string traceFile="/vagrant/ndnSIM/ns-3/mhg_scenario_60_node_2_lane.ns_movements"; 
  Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);

  //int numOfNodes = 60;
  NodeContainer nodes;
  nodes.Create(numOfNodes);

  ns2.Install (); // configure movements for each node, while reading trace file
  
  //std::vector<int> vectorOfCons = getNodesFromList(listOfCons);
  //std::vector<int> vectorOfProds = getNodesFromList(listOfProds);

  ////////////////
  // 1. Install Wifi
  NetDeviceContainer wifiNetDevices = wifi80211p.Install(wifiPhyHelper, wifi80211pMac, nodes);

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
  ndn::StrategyChoiceHelper::Install(nodes, "/neighbor", "/localhost/nfd/strategy/clf");
  //ndn::StrategyChoiceHelper::Install(nodes, "/", "/localhost/nfd/strategy/clf");
  //ndn::StrategyChoiceHelper::Install(nodes, "/", "/localhost/nfd/strategy/best-route");
  
  // 4. Set up applications
  NS_LOG_INFO("Installing Applications");

  ndn::AppHelper consumerHelper("ns3::ndn::ConsumerCbr");
  //ndn::AppHelper consumerHelper("ns3::ndn::ConsumerBatches");
  consumerHelper.SetPrefix("/test/prefix/a/b");
  consumerHelper.SetAttribute("Frequency", DoubleValue(1.0));
  //consumerHelper.SetAttribute("Batches", StringValue("1s 1 8s 1 14s 1 20s 1"));
  //consumerHelper.SetAttribute("Batches", StringValue("5s 1 10s 1 15s 1 20s 1 25s 1"));
  consumerHelper.Install(nodes.Get(consumerNode));

  ndn::AppHelper producerHelper("ns3::ndn::ClfProducer");
  producerHelper.SetPrefix("/test/prefix");
  producerHelper.SetAttribute("PayloadSize", StringValue("1200"));
  producerHelper.Install(nodes.Get(producerNode));
 
  /*for (int i = 0; i < 4; i++) {
    int nodeNo = rand() % numOfNodes;
    ndn::AppHelper producerHelper("ns3::ndn::ClfProducer");
    producerHelper.SetPrefix("/test/prefix");
    producerHelper.SetAttribute("PayloadSize", StringValue("1200"));
    producerHelper.Install(nodes.Get(nodeNo));
  }*/
  ////////////////
  
  Simulator::Stop(Seconds(600.0));

  ndn::L3RateTracer::InstallAll(l3FileName , Seconds(599.0));
  L2RateTracer::InstallAll(l2FileName, Seconds(599.0));
  ndn::AppDelayTracer::InstallAll(appDelayFileName);
  
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
