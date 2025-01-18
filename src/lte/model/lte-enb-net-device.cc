/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010 TELEMATICS LAB, DEE - Politecnico di Bari
 * Copyright (c) 2024 Orange Innovation Poland
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Giuseppe Piro  <g.piro@poliba.it>
 * Author: Marco Miozzo <mmiozzo@cttc.es> : Update to FF API Architecture
 * Author: Nicola Baldo <nbaldo@cttc.es>  : Integrated with new RRC and MAC architecture
 * Author: Danilo Abrignani <danilo.abrignani@unibo.it> : Integrated with new architecture - GSoC 2015 - Carrier Aggregation
 *
 * Modified by: Michele Polese <michele.polese@gmail.com>
 *          Dual Connectivity functionalities
 * Modified by: Kamil Kociszewski <kamil.kociszewski@orange.com>
 *           Parallel reporting for E2
 */

#include <ns3/llc-snap-header.h>
#include <ns3/simulator.h>
#include <ns3/callback.h>
#include <ns3/node.h>
#include <ns3/packet.h>
#include <ns3/lte-net-device.h>
#include <ns3/packet-burst.h>
#include <ns3/uinteger.h>
#include <ns3/trace-source-accessor.h>
#include <ns3/pointer.h>
#include <ns3/enum.h>
#include <ns3/string.h>
#include <ns3/lte-amc.h>
#include <ns3/lte-enb-mac.h>
#include <ns3/lte-enb-net-device.h>
#include <ns3/lte-enb-rrc.h>
#include <ns3/lte-ue-net-device.h>
#include <ns3/lte-enb-phy.h>
#include <ns3/mc-enb-pdcp.h>
#include <ns3/ff-mac-scheduler.h>
#include <ns3/lte-handover-algorithm.h>
#include <ns3/lte-anr.h>
#include <ns3/lte-ffr-algorithm.h>
#include <ns3/ipv4-l3-protocol.h>
#include <ns3/ipv6-l3-protocol.h>
#include <ns3/abort.h>
#include <ns3/log.h>
#include <ns3/lte-enb-component-carrier-manager.h>
#include <ns3/object-map.h>
#include <ns3/object-factory.h>
#include <ns3/lte-radio-bearer-info.h>
#include <cstdint>
#include "encode_e2apv1.hpp"
#include <fstream>
#include <sstream>
#include <ns3/lte-indication-message-helper.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("LteEnbNetDevice");

NS_OBJECT_ENSURE_REGISTERED (LteEnbNetDevice);

/**
* KPM Subscription Request callback.
* This function is triggered whenever a RIC Subscription Request for 
* the KPM RAN Function is received.
*
* \param pdu request message
*/
void
LteEnbNetDevice::KpmSubscriptionCallback (E2AP_PDU_t *sub_req_pdu)
{
  NS_LOG_DEBUG ("\nReceived RIC Subscription Request, cellId = " << m_cellId << "\n");

  E2Termination::RicSubscriptionRequest_rval_s params =
      m_e2term->ProcessRicSubscriptionRequest (sub_req_pdu);
  NS_LOG_DEBUG ("requestorId " << +params.requestorId << ", instanceId " << +params.instanceId
                               << ", ranFuncionId " << +params.ranFuncionId << ", actionId "
                               << +params.actionId);

  if (!m_stopSendingMessages && !m_isReportingEnabled && !m_forceE2FileLogging)
    {
      BuildAndSendReportMessage (params);
      m_isReportingEnabled = true;
    }
}

void
LteEnbNetDevice::stopSendingAndCancelSchedule ()
{
  m_stopSendingMessages = true;
}

void
LteEnbNetDevice::ReadControlFile ()
{
  // open the control file and read handover commands
  if (m_controlFilename != "")
    {
      std::ifstream csv{};
      csv.open (m_controlFilename.c_str (), std::ifstream::in);
      if (!csv.is_open ())
        {
          NS_FATAL_ERROR ("Can't open file " << m_controlFilename.c_str ());
        }
      std::string line;

      if (m_controlFilename.find ("ts_actions_for_ns3.csv") != std::string::npos)
        {

          long long timestamp{};

          while (std::getline (csv, line))
            {
              if (line == "")
                {
                  // skip empty lines
                  continue;
                }
              NS_LOG_INFO ("Read handover command");
              std::stringstream lineStream (line);
              std::string cell;

              std::getline (lineStream, cell, ',');
              timestamp = std::stoll (cell);

              uint64_t imsi;
              std::getline (lineStream, cell, ',');
              imsi = std::stoi (cell);

              uint16_t targetCellId;
              std::getline (lineStream, cell, ',');
              // uncomment the next line if need to remove PLM ID, first 3 digits always 111
              // cell.erase(0, 3);
              targetCellId = std::stoi (cell);

              NS_LOG_INFO ("Handover command for timestamp " << timestamp << " imsi " << imsi
                                                             << " targetCellId " << targetCellId);

              m_rrc->TakeUeHoControl (imsi);
              Simulator::ScheduleWithContext (1, Seconds (0),
                                              &LteEnbRrc::PerformHandoverToTargetCell, m_rrc, imsi,
                                              targetCellId);
            }
        }
      else
        {
          NS_FATAL_ERROR (
              "Unknown use case not implemented yet with filename: " << m_controlFilename);
        }

      csv.close ();

      std::ofstream csvDelete{};
      csvDelete.open (m_controlFilename.c_str ());

      NS_LOG_INFO ("File flushed");
    }

  // TODO check if we need to run multiple times in a m_e2Periodicity time delta,
  // to catch commands that are late
  // We can run every ms, if the file is empty, do not do anything
  Simulator::Schedule (Seconds (0.001), &LteEnbNetDevice::ReadControlFile, this);
}

void
LteEnbNetDevice::ControlMessageReceivedCallback (E2AP_PDU_t *sub_req_pdu)
{
  NS_LOG_DEBUG (
      "\n\nLteEnbNetDevice::ControlMessageReceivedCallback: Received RIC Control Message");

  // Create RIC Control ACK
  Ptr<RicControlMessage> controlMessage = Create<RicControlMessage> (sub_req_pdu);
  NS_LOG_INFO ("After RicControlMessage::RicControlMessage constructor");
  NS_LOG_INFO ("Request type " << controlMessage->m_requestType);
  switch (controlMessage->m_requestType)
    {
      case RicControlMessage::ControlMessageRequestIdType::TS: {
        NS_LOG_INFO ("TS, do the handover");
        // do handover
        Ptr<OctetString> imsiString = Create<OctetString> (
            (void *) controlMessage->m_e2SmRcControlHeaderFormat1->ueID.choice.gNB_UEID,
            controlMessage->m_e2SmRcControlHeaderFormat1->ueID.present); //this line need to fix
        char *end;

        uint64_t imsi = std::strtoull (imsiString->DecodeContent ().c_str (), &end, 10);
        uint16_t targetCellId = std::stoi (controlMessage->GetSecondaryCellIdHO ());
        NS_LOG_INFO ("Imsi Decoded: " << imsi);
        NS_LOG_INFO ("Target Cell id " << targetCellId);
        m_rrc->TakeUeHoControl (imsi);
        if (!m_forceE2FileLogging)
          {
            Simulator::ScheduleWithContext (1, Seconds (0), &LteEnbRrc::PerformHandoverToTargetCell,
                                            m_rrc, imsi, targetCellId);
          }
        else
          {
            Simulator::Schedule (Seconds (0), &LteEnbRrc::PerformHandoverToTargetCell, m_rrc, imsi,
                                 targetCellId);
          }
        break;
      }
      case RicControlMessage::ControlMessageRequestIdType::QoS: {
        // use SetUeQoS()
        NS_FATAL_ERROR ("For QoS use file-based control.");
        break;
      }
      default: {
        NS_LOG_INFO ("Unrecognized id type of Ric Control Message");
        break;
      }
    }
}

TypeId
LteEnbNetDevice::GetTypeId (void)
{
  static TypeId tid =
      TypeId ("ns3::LteEnbNetDevice")
          .SetParent<LteNetDevice> ()
          .AddConstructor<LteEnbNetDevice> ()
          .AddAttribute ("LteEnbRrc", "The RRC associated to this EnbNetDevice", PointerValue (),
                         MakePointerAccessor (&LteEnbNetDevice::m_rrc),
                         MakePointerChecker<LteEnbRrc> ())
          .AddAttribute ("LteHandoverAlgorithm",
                         "The handover algorithm associated to this EnbNetDevice", PointerValue (),
                         MakePointerAccessor (&LteEnbNetDevice::m_handoverAlgorithm),
                         MakePointerChecker<LteHandoverAlgorithm> ())
          .AddAttribute (
              "LteAnr", "The automatic neighbour relation function associated to this EnbNetDevice",
              PointerValue (), MakePointerAccessor (&LteEnbNetDevice::m_anr),
              MakePointerChecker<LteAnr> ())
          .AddAttribute ("LteFfrAlgorithm", "The FFR algorithm associated to this EnbNetDevice",
                         PointerValue (), MakePointerAccessor (&LteEnbNetDevice::m_ffrAlgorithm),
                         MakePointerChecker<LteFfrAlgorithm> ())
          .AddAttribute ("LteEnbComponentCarrierManager", "The RRC associated to this EnbNetDevice",
                         PointerValue (),
                         MakePointerAccessor (&LteEnbNetDevice::m_componentCarrierManager),
                         MakePointerChecker<LteEnbComponentCarrierManager> ())
          .AddAttribute ("ComponentCarrierMap", "List of component carriers.", ObjectMapValue (),
                         MakeObjectMapAccessor (&LteEnbNetDevice::m_ccMap),
                         MakeObjectMapChecker<ComponentCarrierEnb> ())
          .AddAttribute ("UlBandwidth",
                         "Uplink Transmission Bandwidth Configuration in number of Resource Blocks",
                         UintegerValue (100),
                         MakeUintegerAccessor (&LteEnbNetDevice::SetUlBandwidth,
                                               &LteEnbNetDevice::GetUlBandwidth),
                         MakeUintegerChecker<uint8_t> ())
          .AddAttribute (
              "DlBandwidth",
              "Downlink Transmission Bandwidth Configuration in number of Resource Blocks",
              UintegerValue (100),
              MakeUintegerAccessor (&LteEnbNetDevice::SetDlBandwidth,
                                    &LteEnbNetDevice::GetDlBandwidth),
              MakeUintegerChecker<uint8_t> ())
          .AddAttribute ("CellId", "Cell Identifier", UintegerValue (0),
                         MakeUintegerAccessor (&LteEnbNetDevice::m_cellId),
                         MakeUintegerChecker<uint16_t> ())
          .AddAttribute ("DlEarfcn",
                         "Downlink E-UTRA Absolute Radio Frequency Channel Number (EARFCN) "
                         "as per 3GPP 36.101 Section 5.7.3. ",
                         UintegerValue (100), MakeUintegerAccessor (&LteEnbNetDevice::m_dlEarfcn),
                         MakeUintegerChecker<uint32_t> (0, 262143))
          .AddAttribute ("UlEarfcn",
                         "Uplink E-UTRA Absolute Radio Frequency Channel Number (EARFCN) "
                         "as per 3GPP 36.101 Section 5.7.3. ",
                         UintegerValue (18100), MakeUintegerAccessor (&LteEnbNetDevice::m_ulEarfcn),
                         MakeUintegerChecker<uint32_t> (0, 262143))
          .AddAttribute (
              "CsgId", "The Closed Subscriber Group (CSG) identity that this eNodeB belongs to",
              UintegerValue (0),
              MakeUintegerAccessor (&LteEnbNetDevice::SetCsgId, &LteEnbNetDevice::GetCsgId),
              MakeUintegerChecker<uint32_t> ())
          .AddAttribute (
              "CsgIndication",
              "If true, only UEs which are members of the CSG (i.e. same CSG ID) "
              "can gain access to the eNodeB, therefore enforcing closed access mode. "
              "Otherwise, the eNodeB operates as a non-CSG cell and implements open access mode.",
              BooleanValue (false),
              MakeBooleanAccessor (&LteEnbNetDevice::SetCsgIndication,
                                   &LteEnbNetDevice::GetCsgIndication),
              MakeBooleanChecker ())
          .AddAttribute ("E2Termination", "The E2 termination object associated to this node",
                         PointerValue (),
                         MakePointerAccessor (&LteEnbNetDevice::SetE2Termination,
                                              &LteEnbNetDevice::GetE2Termination),
                         MakePointerChecker<E2Termination> ())
          .AddAttribute ("E2PdcpCalculator", "The PDCP calculator object for E2 reporting",
                         PointerValue (),
                         MakePointerAccessor (&LteEnbNetDevice::m_e2PdcpStatsCalculator),
                         MakePointerChecker<mmwave::MmWaveBearerStatsCalculator> ())
          .AddAttribute ("E2RlcCalculator", "The RLC calculator object for E2 reporting",
                         PointerValue (),
                         MakePointerAccessor (&LteEnbNetDevice::m_e2RlcStatsCalculator),
                         MakePointerChecker<mmwave::MmWaveBearerStatsCalculator> ())
          .AddAttribute ("E2Periodicity", "Periodicity of E2 reporting (value in seconds)",
                         DoubleValue (0.1), MakeDoubleAccessor (&LteEnbNetDevice::m_e2Periodicity),
                         MakeDoubleChecker<double> ())
          .AddAttribute ("EnableCuUpReport", "If true, send CuUpReport", BooleanValue (true),
                         MakeBooleanAccessor (&LteEnbNetDevice::m_sendCuUp), MakeBooleanChecker ())
          .AddAttribute ("EnableCuCpReport", "If true, send CuCpReport", BooleanValue (true),
                         MakeBooleanAccessor (&LteEnbNetDevice::m_sendCuCp), MakeBooleanChecker ())
          .AddAttribute (
              "ReducedPmValues", "If true, send only a subset of pmValues", BooleanValue (false),
              MakeBooleanAccessor (&LteEnbNetDevice::m_reducedPmValues), MakeBooleanChecker ())
          .AddAttribute ("EnableE2FileLogging",
                         "If true, force E2 indication generation and write E2 fields in csv file",
                         BooleanValue (false),
                         MakeBooleanAccessor (&LteEnbNetDevice::m_forceE2FileLogging),
                         MakeBooleanChecker ())
          .AddAttribute ("e2andLogging",
                         "If true, sim will be send E2 through E2term and write to csv files",
                         BooleanValue (false), MakeBooleanAccessor (&LteEnbNetDevice::m_e2andlog),
                         MakeBooleanChecker ())
          .AddAttribute ("KPM_E2functionID", "Function ID to subscribe", DoubleValue (2),
                         MakeDoubleAccessor (&LteEnbNetDevice::e2_func_id),
                         MakeDoubleChecker<double> ())
          .AddAttribute ("RC_E2functionID", "Function ID to subscribe", DoubleValue (3),
                         MakeDoubleAccessor (&LteEnbNetDevice::rc_e2_func_id),
                         MakeDoubleChecker<double> ())
          .AddAttribute (
              "ControlFileName",
              "Filename for the stand alone control mode. The file is deleted after every read."
              "Format should correspond to the particular use case:\n"
              "TS: Contains multiple lines with ts, imsi, targetCellId\n",
              StringValue (""), MakeStringAccessor (&LteEnbNetDevice::m_controlFilename),
              MakeStringChecker ());
  return tid;
}

LteEnbNetDevice::LteEnbNetDevice ()
    : m_stopSendingMessages (false),
      m_isConstructed (false),
      m_isConfigured (false),
      m_anr (0),
      m_componentCarrierManager (0),
      m_isReportingEnabled (false),
      m_reducedPmValues (false),
      m_forceE2FileLogging (false),
      m_e2andlog (false),
      m_cuUpFileName (),
      m_cuCpFileName ()
{
  NS_LOG_FUNCTION (this);
}

LteEnbNetDevice::~LteEnbNetDevice (void)
{
  NS_LOG_FUNCTION (this);
}

void
LteEnbNetDevice::DoDispose ()
{
  NS_LOG_FUNCTION (this);

  m_rrc->Dispose ();
  m_rrc = 0;

  m_handoverAlgorithm->Dispose ();
  m_handoverAlgorithm = 0;

  if (m_anr != 0)
    {
      m_anr->Dispose ();
      m_anr = 0;
    }
  m_componentCarrierManager->Dispose ();
  m_componentCarrierManager = 0;
  // ComponentCarrierEnb::DoDispose() will call DoDispose
  // of its PHY, MAC, FFR and scheduler instance
  for (uint32_t i = 0; i < m_ccMap.size (); i++)
    {
      m_ccMap.at (i)->Dispose ();
      m_ccMap.at (i) = 0;
    }

  LteNetDevice::DoDispose ();
}

Ptr<LteEnbMac>
LteEnbNetDevice::GetMac () const
{
  return m_ccMap.at (0)->GetMac ();
}

Ptr<LteEnbPhy>
LteEnbNetDevice::GetPhy () const
{
  return m_ccMap.at (0)->GetPhy ();
}

Ptr<LteEnbMac>
LteEnbNetDevice::GetMac (uint8_t index)
{
  return m_ccMap.at (index)->GetMac ();
}

Ptr<LteEnbPhy>
LteEnbNetDevice::GetPhy (uint8_t index)
{
  return m_ccMap.at (index)->GetPhy ();
}

Ptr<LteEnbRrc>
LteEnbNetDevice::GetRrc () const
{
  return m_rrc;
}

Ptr<LteEnbComponentCarrierManager>
LteEnbNetDevice::GetComponentCarrierManager () const
{
  return m_componentCarrierManager;
}

uint16_t
LteEnbNetDevice::GetCellId () const
{
  return m_cellId;
}

bool
LteEnbNetDevice::HasCellId (uint16_t cellId) const
{
  for (auto &it : m_ccMap)
    {
      if (it.second->GetCellId () == cellId)
        {
          return true;
        }
    }
  return false;
}

uint8_t
LteEnbNetDevice::GetUlBandwidth () const
{
  return m_ulBandwidth;
}

void
LteEnbNetDevice::SetUlBandwidth (uint8_t bw)
{
  NS_LOG_FUNCTION (this << uint16_t (bw));
  switch (bw)
    {
    case 6:
    case 15:
    case 25:
    case 50:
    case 75:
    case 100:
      m_ulBandwidth = bw;
      break;

    default:
      NS_FATAL_ERROR ("invalid bandwidth value " << (uint16_t) bw);
      break;
    }
}

uint8_t
LteEnbNetDevice::GetDlBandwidth () const
{
  return m_dlBandwidth;
}

void
LteEnbNetDevice::SetDlBandwidth (uint8_t bw)
{
  NS_LOG_FUNCTION (this << uint16_t (bw));
  switch (bw)
    {
    case 6:
    case 15:
    case 25:
    case 50:
    case 75:
    case 100:
      m_dlBandwidth = bw;
      break;

    default:
      NS_FATAL_ERROR ("invalid bandwidth value " << (uint16_t) bw);
      break;
    }
}

uint32_t
LteEnbNetDevice::GetDlEarfcn () const
{
  return m_dlEarfcn;
}

void
LteEnbNetDevice::SetDlEarfcn (uint32_t earfcn)
{
  NS_LOG_FUNCTION (this << earfcn);
  m_dlEarfcn = earfcn;
}

uint32_t
LteEnbNetDevice::GetUlEarfcn () const
{
  return m_ulEarfcn;
}

void
LteEnbNetDevice::SetUlEarfcn (uint32_t earfcn)
{
  NS_LOG_FUNCTION (this << earfcn);
  m_ulEarfcn = earfcn;
}

uint32_t
LteEnbNetDevice::GetCsgId () const
{
  return m_csgId;
}

void
LteEnbNetDevice::SetCsgId (uint32_t csgId)
{
  NS_LOG_FUNCTION (this << csgId);
  m_csgId = csgId;
  UpdateConfig (); // propagate the change to RRC level
}

bool
LteEnbNetDevice::GetCsgIndication () const
{
  return m_csgIndication;
}

void
LteEnbNetDevice::SetCsgIndication (bool csgIndication)
{
  NS_LOG_FUNCTION (this << csgIndication);
  m_csgIndication = csgIndication;
  UpdateConfig (); // propagate the change to RRC level
}

std::map<uint8_t, Ptr<ComponentCarrierEnb>>
LteEnbNetDevice::GetCcMap ()
{
  return m_ccMap;
}

void
LteEnbNetDevice::SetCcMap (std::map<uint8_t, Ptr<ComponentCarrierEnb>> ccm)
{
  NS_ASSERT_MSG (!m_isConfigured, "attempt to set CC map after configuration");
  m_ccMap = ccm;
}

void
LteEnbNetDevice::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
  m_isConstructed = true;
  UpdateConfig ();
  std::map<uint8_t, Ptr<ComponentCarrierEnb>>::iterator it;
  for (it = m_ccMap.begin (); it != m_ccMap.end (); ++it)
    {
      it->second->Initialize ();
    }
  m_rrc->Initialize ();
  m_componentCarrierManager->Initialize ();
  m_handoverAlgorithm->Initialize ();

  if (m_anr != 0)
    {
      m_anr->Initialize ();
    }

  m_ffrAlgorithm->Initialize ();
}

bool
LteEnbNetDevice::Send (Ptr<Packet> packet, const Address &dest, uint16_t protocolNumber)
{
  NS_LOG_FUNCTION (this << packet << dest << protocolNumber);
  NS_ABORT_MSG_IF (protocolNumber != Ipv4L3Protocol::PROT_NUMBER &&
                       protocolNumber != Ipv6L3Protocol::PROT_NUMBER,
                   "unsupported protocol " << protocolNumber
                                           << ", only IPv4 and IPv6 are supported");
  return m_rrc->SendData (packet);
}

void
LteEnbNetDevice::UpdateConfig (void)
{
  NS_LOG_FUNCTION (this);

  if (m_isConstructed)
    {
      if (!m_isConfigured)
        {
          NS_LOG_LOGIC (this << " Configure cell " << m_cellId);
          // we have to make sure that this function is called only once
          NS_ASSERT (!m_ccMap.empty ());
          m_rrc->ConfigureCell (m_ccMap);
          m_isConfigured = true;
        }

      NS_LOG_LOGIC (this << " Updating SIB1 of cell " << m_cellId << " with CSG ID " << m_csgId
                         << " and CSG indication " << m_csgIndication);
      m_rrc->SetCsgId (m_csgId, m_csgIndication);

      if (m_e2term)
        {
          NS_LOG_DEBUG ("E2sim start in cell " << m_cellId << " force CSV logging "
                                               << m_forceE2FileLogging);

          //                if (!m_forceE2FileLogging) {
          //                    Simulator::Schedule(MicroSeconds(0), &E2Termination::Start, m_e2term);
          //                } else { // give some time for the simulation to start, TODO check value
          //                    m_cuUpFileName = "cu-up-cell-" + std::to_string(m_cellId) + ".txt";
          //                    std::ofstream csv{};
          //                    csv.open(m_cuUpFileName.c_str());
          //                    csv << "timestamp,ueImsiComplete,DRB.PdcpSduDelayDl (cellAverageLatency),"
          //                           "m_pDCPBytesUL (0),m_pDCPBytesDL (cellDlTxVolume),"
          //                           "DRB.PdcpSduVolumeDl_Filter.UEID (txBytes),"
          //                           "Tot.PdcpSduNbrDl.UEID (txDlPackets),DRB.PdcpSduBitRateDl.UEID (pdcpThroughput),"
          //                           "DRB.PdcpSduDelayDl.UEID (pdcpLatency),QosFlow.PdcpPduVolumeDL_Filter.UEID"
          //                           "(txPdcpPduBytesNrRlc),DRB.PdcpPduNbrDl.Qos.UEID (txPdcpPduNrRlc)\n";
          //                    csv.close();
          //
          //                    m_cuCpFileName = "cu-cp-cell-" + std::to_string(m_cellId) + ".txt";
          //                    csv.open(m_cuCpFileName.c_str());
          //                    csv << "timestamp,ueImsiComplete,numActiveUes,DRB.EstabSucc.5QI.UEID (numDrb),"
          //                           "DRB.RelActNbr.5QI.UEID (0),enbdev (m_cellId),UE (imsi),sameCellSinr,"
          //                           "sameCellSinr 3gpp encoded,L3 neigh Id (cellId),"
          //                           "sinr,3gpp encoded sinr (convertedSinr)\n";
          //                    csv.close();
          //                    Simulator::Schedule(MicroSeconds(500), &LteEnbNetDevice::BuildAndSendReportMessage, this,
          //                                        E2Termination::RicSubscriptionRequest_rval_s{});
          //
          //                    Simulator::Schedule(MicroSeconds(1000), &LteEnbNetDevice::ReadControlFile, this);
          //                }
          if (m_forceE2FileLogging || m_e2andlog)
            { // give some time for the simulation to start, TODO check value
              m_cuUpFileName = "cu-up-cell-" + std::to_string (m_cellId) + ".txt";
              std::ofstream csv{};
              csv.open (m_cuUpFileName.c_str ());
              csv << "timestamp,ueImsiComplete,DRB.PdcpSduDelayDl (cellAverageLatency),"
                     "m_pDCPBytesUL (0),m_pDCPBytesDL (cellDlTxVolume),"
                     "DRB.PdcpSduVolumeDl_Filter.UEID (txBytes),"
                     "Tot.PdcpSduNbrDl.UEID (txDlPackets),DRB.PdcpSduBitRateDl.UEID "
                     "(pdcpThroughput),"
                     "DRB.PdcpSduDelayDl.UEID (pdcpLatency),QosFlow.PdcpPduVolumeDL_Filter.UEID"
                     "(txPdcpPduBytesNrRlc),DRB.PdcpPduNbrDl.Qos.UEID (txPdcpPduNrRlc)\n";
              csv.close ();

              m_cuCpFileName = "cu-cp-cell-" + std::to_string (m_cellId) + ".txt";
              csv.open (m_cuCpFileName.c_str ());
              csv << "timestamp,ueImsiComplete,numActiveUes,DRB.EstabSucc.5QI.UEID (numDrb),"
                     "DRB.RelActNbr.5QI.UEID (0),enbdev (m_cellId),UE (imsi),sameCellSinr,"
                     "sameCellSinr 3gpp encoded,L3 neigh Id (cellId),"
                     "sinr,3gpp encoded sinr (convertedSinr)\n";
              csv.close ();

              //
              if (m_e2andlog)
                {
                  Simulator::Schedule (MicroSeconds (0), &E2Termination::Start, m_e2term);
                }
              else
                {
                  Simulator::Schedule (MicroSeconds (500),
                                       &LteEnbNetDevice::BuildAndSendReportMessage, this,
                                       E2Termination::RicSubscriptionRequest_rval_s{});

                  Simulator::Schedule (MicroSeconds (1000), &LteEnbNetDevice::ReadControlFile,
                                       this);
                }
            }
          else
            {
              Simulator::Schedule (MicroSeconds (0), &E2Termination::Start, m_e2term);
            }
        }
    }
  else
    {
      /*
             * Lower layers are not ready yet, so do nothing now and expect
             * ``DoInitialize`` to re-invoke this function.
             */
    }
}

Ptr<E2Termination>
LteEnbNetDevice::GetE2Termination () const
{
  return m_e2term;
}

void
LteEnbNetDevice::SetE2Termination (Ptr<E2Termination> e2term)
{
  m_e2term = e2term;

  NS_LOG_DEBUG ("Register E2SM LteEnbNetDevice");

  if (!m_forceE2FileLogging || m_e2andlog)
    {
      long m_e2_func_id = long (e2_func_id);
      long m_rc_e2_func_id = long (rc_e2_func_id);
      Ptr<KpmFunctionDescription> kpmFd = Create<KpmFunctionDescription> ();
      e2term->RegisterKpmCallbackToE2Sm (
          m_e2_func_id, kpmFd,
          std::bind (&LteEnbNetDevice::KpmSubscriptionCallback, this, std::placeholders::_1));

      Ptr<RicControlFunctionDescription> ricCtrlFd = Create<RicControlFunctionDescription> ();
      e2term->RegisterSmCallbackToE2Sm (m_rc_e2_func_id, ricCtrlFd,
                                        std::bind (&LteEnbNetDevice::ControlMessageReceivedCallback,
                                                   this, std::placeholders::_1));

      // Mostafa-FD-TODO
      // Ptr<RicDeletelFunctionDescription> ricDeletelFd = Create<RicDeletelFunctionDescription> ();
      // e2term->RegisterSmCallbackToE2Sm (4, static_cast<Ptr<FunctionDescription>>(ricDeletelFd),
      //                                   std::bind (&LteEnbNetDevice::stopSendingAndCancelSchedule,
      //                                              this, std::placeholders::_1));

      // Ptr<FunctionDescription> ricDeletelFd = Create<FunctionDescription> ();

      // e2term->RegisterSmCallbackToE2Sm (4, ricDeletelFd,
      //                                   std::bind (&LteEnbNetDevice::stopSendingAndCancelSchedule,
      //                                              this));
      e2term->RegisterCallbackFunctionToE2Sm (
          1, std::bind (&LteEnbNetDevice::stopSendingAndCancelSchedule, this));
    }
}

std::string
LteEnbNetDevice::GetImsiString (uint64_t imsi)
{
  std::string ueImsi = std::to_string (imsi);
  std::string ueImsiComplete{};
  if (ueImsi.length () == 1)
    {
      ueImsiComplete = "0000" + ueImsi;
    }
  else if (ueImsi.length () == 2)
    {
      ueImsiComplete = "000" + ueImsi;
    }
  else
    {
      ueImsiComplete = "00" + ueImsi;
    }
  return ueImsiComplete;
}

Ptr<KpmIndicationHeader>
LteEnbNetDevice::BuildRicIndicationHeader (std::string plmId, std::string gnbId, uint16_t nrCellId)
{
  if (!m_forceE2FileLogging)
    {
      KpmIndicationHeader::KpmRicIndicationHeaderValues headerValues;
      headerValues.m_plmId = plmId;
      headerValues.m_gnbId = gnbId;
      headerValues.m_nrCellId = nrCellId;
      auto time = Simulator::Now ();
      uint64_t timestamp = m_startTime + (uint64_t) time.GetMilliSeconds ();
      NS_LOG_DEBUG ("NR plmid " << plmId << " gnbId " << gnbId << " nrCellId " << nrCellId);
      NS_LOG_DEBUG ("Timestamp " << timestamp);
      headerValues.m_timestamp = timestamp;
      Ptr<KpmIndicationHeader> header =
          Create<KpmIndicationHeader> (KpmIndicationHeader::GlobalE2nodeType::eNB, headerValues);

      return header;
    }
  else
    {
      return nullptr;
    }
}

Ptr<KpmIndicationMessage>
LteEnbNetDevice::BuildRicIndicationMessageCuUp (std::string plmId)
{
  bool local_m_forceE2FileLogging;

  if (m_forceE2FileLogging)
    {
      local_m_forceE2FileLogging = true;
    }
  else
    {
      local_m_forceE2FileLogging = false;
    }
  if (m_e2andlog)
    {
      local_m_forceE2FileLogging = false;
    }
  Ptr<LteIndicationMessageHelper> indicationMessageHelper =
      Create<LteIndicationMessageHelper> (IndicationMessageHelper::IndicationMessageType::CuUp,
                                          local_m_forceE2FileLogging, m_reducedPmValues);

  // get <rnti, UeManager> map of connected UEs
  auto ueMap = m_rrc->GetUeMap ();
  // gNB-wide PDCP volume in downlink
  double cellDlTxVolume = 0;

  // sum of the per-user average latency
  double perUserAverageLatencySum = 0;

  std::unordered_map<uint64_t, std::string> uePmString{};

  for (auto ue : ueMap)
    {
      uint64_t imsi = ue.second->GetImsi ();
      std::string ueImsiComplete = GetImsiString (imsi);

      // TODO fix types
      long txDlPackets =
          m_e2PdcpStatsCalculator->GetDlTxPackets (imsi, 3); // LCID 3 is used for data
      double txBytes =
          m_e2PdcpStatsCalculator->GetDlTxData (imsi, 3) * 8 / 1e3; // in kbit, not byte
      cellDlTxVolume += txBytes;

      long txPdcpPduLteRlc = 0;
      double txPdcpPduBytesLteRlc = 0;
      auto drbMap = ue.second->GetDrbMap ();
      for (auto drb : drbMap)
        {
          txPdcpPduLteRlc += drb.second->m_rlc->GetTxPacketsInReportingPeriod ();
          txPdcpPduBytesLteRlc += drb.second->m_rlc->GetTxBytesInReportingPeriod ();
          drb.second->m_rlc->ResetRlcCounters ();
        }
      auto rlcMap = ue.second->GetRlcMap (); // secondary-connected RLCs
      for (auto drb : rlcMap)
        {
          txPdcpPduLteRlc += drb.second->m_rlc->GetTxPacketsInReportingPeriod ();
          txPdcpPduBytesLteRlc += drb.second->m_rlc->GetTxBytesInReportingPeriod ();
          drb.second->m_rlc->ResetRlcCounters ();
        }
      txPdcpPduBytesLteRlc *= 8 / 1e3;

      long txPdcpPduNrRlc = std::max (long (0), txDlPackets - txPdcpPduLteRlc);
      double txPdcpPduBytesNrRlc = std::max (0.0, txBytes - txPdcpPduBytesLteRlc);

      double pdcpLatency = m_e2PdcpStatsCalculator->GetDlDelay (imsi, 3) / 1e5; // unit: x 0.1 ms
      perUserAverageLatencySum += pdcpLatency;

      double pdcpThroughput = txBytes / m_e2Periodicity; // unit kbps

      NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                    << " " << std::to_string (m_cellId) << " cell, connected UE with IMSI " << imsi
                    << " ueImsiString " << ueImsiComplete << " txDlPackets " << txDlPackets
                    << " txDlPacketsNr " << txPdcpPduNrRlc << " txBytes " << txBytes
                    << " txDlBytesNr " << txPdcpPduBytesNrRlc << " pdcpLatency " << pdcpLatency
                    << " pdcpThroughput " << pdcpThroughput);

      m_e2PdcpStatsCalculator->ResetResultsForImsiLcid (imsi, 3);

      if (!indicationMessageHelper->IsOffline ())
        {
          indicationMessageHelper->AddCuUpUePmItem (ueImsiComplete, txBytes, txDlPackets,
                                                    pdcpThroughput, pdcpLatency);
        }

      uePmString.insert (std::make_pair (
          imsi, std::to_string (txBytes) + "," + std::to_string (txDlPackets) + "," +
                    std::to_string (pdcpThroughput) + "," + std::to_string (pdcpLatency)));
    }

  // get average cell latency
  double cellAverageLatency = 0;
  if (!ueMap.empty ())
    {
      cellAverageLatency = perUserAverageLatencySum / ueMap.size ();
    }

  NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                << " " << std::to_string (m_cellId) << " cell, connected UEs number "
                << ueMap.size () << " cellAverageLatency " << cellAverageLatency);

  if (!indicationMessageHelper->IsOffline ())
    {
      indicationMessageHelper->AddCuUpCellPmItem (cellAverageLatency);
    }

  // PDCP volume for the whole cell
  if (!indicationMessageHelper->IsOffline ())
    {
      // pDCPBytesUL = 0 since it is not supported from the simulator
      indicationMessageHelper->FillCuUpValues (plmId, 0, cellDlTxVolume);
    }

  NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                << " " << std::to_string (m_cellId) << " cell volume " << cellDlTxVolume);

  if (m_forceE2FileLogging)
    {

      std::ofstream csv{};
      csv.open (m_cuUpFileName.c_str (), std::ios_base::app);
      if (!csv.is_open ())
        {
          NS_FATAL_ERROR ("Can't open file " << m_cuUpFileName.c_str ());
        }

      uint64_t timestamp = m_startTime + (uint64_t) Simulator::Now ().GetMilliSeconds ();

      // the string is timestamp,ueImsiComplete,DRB.PdcpSduDelayDl (cellAverageLatency),
      // m_pDCPBytesUL (0),m_pDCPBytesDL (cellDlTxVolume),DRB.PdcpSduVolumeDl_Filter.UEID (txBytes),
      // Tot.PdcpSduNbrDl.UEID (txDlPackets),DRB.PdcpSduBitRateDl.UEID (pdcpThroughput),
      // DRB.PdcpSduDelayDl.UEID (pdcpLatency),QosFlow.PdcpPduVolumeDL_Filter.UEID (txPdcpPduBytesNrRlc),
      // DRB.PdcpPduNbrDl.Qos.UEID (txPdcpPduNrRlc)

      // the last two are not available on LTE
      for (auto ue : ueMap)
        {
          uint64_t imsi = ue.second->GetImsi ();
          std::string ueImsiComplete = GetImsiString (imsi);

          auto uePms = uePmString.find (imsi)->second;

          std::string to_print = std::to_string (timestamp) + "," + ueImsiComplete + "," +
                                 std::to_string (cellAverageLatency) + "," + std::to_string (0) +
                                 "," + std::to_string (cellDlTxVolume) + "," + uePms + ",,\n";

          csv << to_print;
        }
      csv.close ();
      return nullptr;
    }
  else
    {
      //
      if (m_e2andlog == 1)
        {
          std::ofstream csv{};
          csv.open (m_cuUpFileName.c_str (), std::ios_base::app);
          if (!csv.is_open ())
            {
              NS_FATAL_ERROR ("Can't open file " << m_cuUpFileName.c_str ());
            }

          uint64_t timestamp = m_startTime + (uint64_t) Simulator::Now ().GetMilliSeconds ();

          // the string is timestamp,ueImsiComplete,DRB.PdcpSduDelayDl (cellAverageLatency),
          // m_pDCPBytesUL (0),m_pDCPBytesDL (cellDlTxVolume),DRB.PdcpSduVolumeDl_Filter.UEID (txBytes),
          // Tot.PdcpSduNbrDl.UEID (txDlPackets),DRB.PdcpSduBitRateDl.UEID (pdcpThroughput),
          // DRB.PdcpSduDelayDl.UEID (pdcpLatency),QosFlow.PdcpPduVolumeDL_Filter.UEID (txPdcpPduBytesNrRlc),
          // DRB.PdcpPduNbrDl.Qos.UEID (txPdcpPduNrRlc)

          // the last two are not available on LTE
          for (auto ue : ueMap)
            {
              uint64_t imsi = ue.second->GetImsi ();
              std::string ueImsiComplete = GetImsiString (imsi);

              auto uePms = uePmString.find (imsi)->second;

              std::string to_print = std::to_string (timestamp) + "," + ueImsiComplete + "," +
                                     std::to_string (cellAverageLatency) + "," +
                                     std::to_string (0) + "," + std::to_string (cellDlTxVolume) +
                                     "," + uePms + ",,\n";

              csv << to_print;
            }
          csv.close ();
        }
      //
      return indicationMessageHelper->CreateIndicationMessage ();
    }
}

Ptr<KpmIndicationMessage>
LteEnbNetDevice::BuildRicIndicationMessageCuCp (std::string plmId)
{
  bool local_m_forceE2FileLogging;

  if (m_forceE2FileLogging)
    {
      local_m_forceE2FileLogging = true;
    }
  else
    {
      local_m_forceE2FileLogging = false;
    }
  if (m_e2andlog)
    {
      local_m_forceE2FileLogging = false;
    }

  Ptr<LteIndicationMessageHelper> indicationMessageHelper =
      Create<LteIndicationMessageHelper> (IndicationMessageHelper::IndicationMessageType::CuCp,
                                          local_m_forceE2FileLogging, m_reducedPmValues);

  auto ueMap = m_rrc->GetUeMap ();
  auto ueMapSize = ueMap.size ();

  std::unordered_map<uint64_t, std::string> uePmString{};

  for (auto ue : ueMap)
    {
      uint64_t imsi = ue.second->GetImsi ();
      std::string ueImsiComplete = GetImsiString (imsi);
      long numDrb = ue.second->GetDrbMap ().size ();

      if (!indicationMessageHelper->IsOffline ())
        {
          // DRB.RelActNbr.5QI.UEID not modeled in the simulator
          indicationMessageHelper->AddCuCpUePmItem (ueImsiComplete, numDrb, 0);
        }

      uePmString.insert (std::make_pair (imsi, std::to_string (numDrb) + "," + std::to_string (0)));
    }

  if (!indicationMessageHelper->IsOffline ())
    {
      indicationMessageHelper->FillCuCpValues (ueMapSize);
    }

  if (m_forceE2FileLogging)
    {
      std::ofstream csv{};
      csv.open (m_cuCpFileName.c_str (), std::ios_base::app);
      if (!csv.is_open ())
        {
          NS_FATAL_ERROR ("Can't open file " << m_cuCpFileName.c_str ());
        }

      NS_LOG_DEBUG ("m_cuCpFileName open " << m_cuCpFileName);

      // the string is timestamp, ueImsiComplete, numActiveUes, DRB.EstabSucc.5QI.UEID (numDrb), DRB.RelActNbr.5QI.UEID (0)

      uint64_t timestamp = m_startTime + (uint64_t) Simulator::Now ().GetMilliSeconds ();

      for (auto ue : ueMap)
        {
          uint64_t imsi = ue.second->GetImsi ();
          std::string ueImsiComplete = GetImsiString (imsi);

          auto uePms = uePmString.find (imsi)->second;

          std::string to_print = std::to_string (timestamp) + "," + ueImsiComplete + "," +
                                 std::to_string (ueMapSize) + "," + uePms + ",,,,,,," + "\n";

          NS_LOG_DEBUG (to_print);

          csv << to_print;
        }

      csv.close ();

      return nullptr;
    }
  else
    {
      if (m_e2andlog == 1)
        {
          std::ofstream csv{};
          csv.open (m_cuCpFileName.c_str (), std::ios_base::app);
          if (!csv.is_open ())
            {
              NS_FATAL_ERROR ("Can't open file " << m_cuCpFileName.c_str ());
            }

          NS_LOG_DEBUG ("m_cuCpFileName open " << m_cuCpFileName);

          // the string is timestamp, ueImsiComplete, numActiveUes, DRB.EstabSucc.5QI.UEID (numDrb), DRB.RelActNbr.5QI.UEID (0)

          uint64_t timestamp = m_startTime + (uint64_t) Simulator::Now ().GetMilliSeconds ();

          for (auto ue : ueMap)
            {
              uint64_t imsi = ue.second->GetImsi ();
              std::string ueImsiComplete = GetImsiString (imsi);

              auto uePms = uePmString.find (imsi)->second;

              std::string to_print = std::to_string (timestamp) + "," + ueImsiComplete + "," +
                                     std::to_string (ueMapSize) + "," + uePms + ",,,,,,," + "\n";

              NS_LOG_DEBUG (to_print);

              csv << to_print;
            }

          csv.close ();
        }
      return indicationMessageHelper->CreateIndicationMessage ();
    }
}

void
LteEnbNetDevice::BuildAndSendReportMessage (E2Termination::RicSubscriptionRequest_rval_s params)
{
  std::string plmId = "111";
  std::string gnbId = std::to_string (m_cellId);

  // TODO here we can get something from RRC and onward
  NS_LOG_DEBUG ("LteEnbNetDevice " << std::to_string (m_cellId) << " BuildAndSendMessage at time "
                                   << Simulator::Now ().GetSeconds ());

  if (m_sendCuUp)
    {
      // Create CU-UP
      Ptr<KpmIndicationHeader> header = BuildRicIndicationHeader (plmId, gnbId, m_cellId);
      Ptr<KpmIndicationMessage> cuUpMsg = BuildRicIndicationMessageCuUp (plmId);

      // Send CU-UP only if offline logging is disabled
      if (!m_forceE2FileLogging && header != nullptr && cuUpMsg != nullptr)
        {
          NS_LOG_DEBUG ("Send LTE CU-UP");
          E2AP_PDU *pdu_cuup_ue = new E2AP_PDU;
          encoding::generate_e2apv1_indication_request_parameterized (
              pdu_cuup_ue, params.requestorId, params.instanceId, params.ranFuncionId,
              params.actionId,
              1, // TODO sequence number
              (uint8_t *) header->m_buffer, // buffer containing the encoded header
              header->m_size, // size of the encoded header
              (uint8_t *) cuUpMsg->m_buffer, // buffer containing the encoded message
              cuUpMsg->m_size); // size of the encoded message

          m_e2term->SendE2Message (pdu_cuup_ue);
          delete pdu_cuup_ue;
        }
    }

  if (m_sendCuCp)
    {
      // Create CU-CP
      Ptr<KpmIndicationHeader> header = BuildRicIndicationHeader (plmId, gnbId, m_cellId);
      Ptr<KpmIndicationMessage> cuCpMsg = BuildRicIndicationMessageCuCp (plmId);

      // Send CU-CP only if offline logging is disabled
      if (!m_forceE2FileLogging && header != nullptr && cuCpMsg != nullptr)
        {

          NS_LOG_DEBUG ("Send LTE CU-CP");
          E2AP_PDU *pdu_cucp_ue = new E2AP_PDU;
          encoding::generate_e2apv1_indication_request_parameterized (
              pdu_cucp_ue, params.requestorId, params.instanceId, params.ranFuncionId,
              params.actionId,
              1, // TODO sequence number
              (uint8_t *) header->m_buffer, // buffer containing the encoded header
              header->m_size, // size of the encoded header
              (uint8_t *) cuCpMsg->m_buffer, // buffer containing the encoded message
              cuCpMsg->m_size); // size of the encoded message
          m_e2term->SendE2Message (pdu_cucp_ue);
          delete pdu_cucp_ue;
        }
    }

  if (m_stopSendingMessages)
    {
      return;
    }

  if (!m_stopSendingMessages)
    {
      if (!m_forceE2FileLogging)
        Simulator::ScheduleWithContext (1, Seconds (m_e2Periodicity),
                                        &LteEnbNetDevice::BuildAndSendReportMessage, this, params);
      else
        Simulator::Schedule (Seconds (m_e2Periodicity), &LteEnbNetDevice::BuildAndSendReportMessage,
                             this, params);
    }
}

void
LteEnbNetDevice::SetStartTime (uint64_t st)
{
  m_startTime = st;
}

} // namespace ns3
