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

#include "ndn-clf-producer.hpp"
#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"

#include "model/ndn-l3-protocol.hpp"
#include "helper/ndn-fib-helper.hpp"

#include "ndn-cxx/lp/tags.hpp"
#include "ndn-cxx/location.hpp"
#include "ndn-cxx/prefix-announcement.hpp"
#include "ndn-cxx/security/signing-helpers.hpp"
#include "ndn-cxx/security/v2/key-chain.hpp"

#include <memory>

#include "ns3/mobility-model.h"

NS_LOG_COMPONENT_DEFINE("ndn.ClfProducer");

namespace ns3 {
namespace ndn {

NS_OBJECT_ENSURE_REGISTERED(ClfProducer);

TypeId
ClfProducer::GetTypeId(void)
{
  static TypeId tid =
    TypeId("ns3::ndn::ClfProducer")
      .SetGroupName("Ndn")
      .SetParent<App>()
      .AddConstructor<ClfProducer>()
      .AddAttribute("Prefix", "Prefix, for which producer has the data", StringValue("/"),
                    MakeNameAccessor(&ClfProducer::m_prefix), MakeNameChecker())
      .AddAttribute(
         "Postfix",
         "Postfix that is added to the output data (e.g., for adding producer-uniqueness)",
         StringValue("/"), MakeNameAccessor(&ClfProducer::m_postfix), MakeNameChecker())
      .AddAttribute("PayloadSize", "Virtual payload size for Content packets", UintegerValue(1024),
                    MakeUintegerAccessor(&ClfProducer::m_virtualPayloadSize),
                    MakeUintegerChecker<uint32_t>())
      .AddAttribute("Freshness", "Freshness of data packets, if 0, then unlimited freshness",
                    TimeValue(Seconds(0)), MakeTimeAccessor(&ClfProducer::m_freshness),
                    MakeTimeChecker())
      .AddAttribute(
         "Signature",
         "Fake signature, 0 valid signature (default), other values application-specific",
         UintegerValue(0), MakeUintegerAccessor(&ClfProducer::m_signature),
         MakeUintegerChecker<uint32_t>())
      .AddAttribute("KeyLocator",
                    "Name to be used for key locator.  If root, then key locator is not used",
                    NameValue(), MakeNameAccessor(&ClfProducer::m_keyLocator), MakeNameChecker());
  return tid;
}

ClfProducer::ClfProducer()
{
  NS_LOG_FUNCTION_NOARGS();
}

// inherited from Application base class.
void
ClfProducer::StartApplication()
{
  NS_LOG_FUNCTION_NOARGS();
  App::StartApplication();

  FibHelper::AddRoute(GetNode(), m_prefix, m_face, 0);
}

void
ClfProducer::StopApplication()
{
  NS_LOG_FUNCTION_NOARGS();

  App::StopApplication();
}

void
ClfProducer::OnInterest(shared_ptr<const Interest> interest)
{
  App::OnInterest(interest); // tracing inside

  NS_LOG_FUNCTION(this << interest);

  if (!m_active)
    return;

  Name dataName(interest->getName());
  // dataName.append(m_postfix);
  // dataName.appendVersion();

  auto data = make_shared<Data>();
  data->setName(dataName);
  data->setFreshnessPeriod(::ndn::time::milliseconds(m_freshness.GetMilliSeconds()));

  data->setContent(make_shared< ::ndn::Buffer>(m_virtualPayloadSize));

  Signature signature;
  SignatureInfo signatureInfo(static_cast< ::ndn::tlv::SignatureTypeValue>(255));

  if (m_keyLocator.size() > 0) {
    signatureInfo.setKeyLocator(m_keyLocator);
  }

  signature.setInfo(signatureInfo);
  signature.setValue(::ndn::makeNonNegativeIntegerBlock(::ndn::tlv::SignatureValue, m_signature));

  data->setSignature(signature);

  // set current location in the data packet 
  Ptr<MobilityModel> mobility = GetNode()->GetObject<MobilityModel> ();
  Vector3D currentPosition = mobility->GetPosition();
  ::ndn::Location destLocation(currentPosition.x, currentPosition.y);  
  ::ndn::Location myLocation(0,0);
  ::ndn::Location prevLocation(0,0);
  lp::LocationHeader lh;
  lh.setMyLocation(myLocation);
  lh.setPrevLocation(prevLocation);
  lh.setDestLocation(destLocation);
  data->setTag(make_shared<lp::LocationTag>(lp::LocationHeader(lh)));
  // set prefix announcement
  ::ndn::PrefixAnnouncement pa;
  pa.setAnnouncedName("/test/prefix/a");
  //pa.setExpiration(1_h);
  //
  KeyChain keyChain("pib-memory", "tpm-memory");
  pa.toData(keyChain, ::ndn::signingWithSha256());
  lp::PrefixAnnouncementHeader pah(pa);
  data->setTag(make_shared<lp::PrefixAnnouncementTag>(lp::PrefixAnnouncementHeader(pah)));

  NS_LOG_INFO("node(" << GetNode()->GetId() << ") at (" << currentPosition.x << ", " << currentPosition.y  << ") corresponding with Data: " << data->getName());

  // to create real wire encoding
  data->wireEncode();

  m_transmittedDatas(data, this, m_face);
  m_appLink->onReceiveData(*data);
}

} // namespace ndn
} // namespace ns3
