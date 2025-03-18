/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/* *
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
 * Authors: Andrea Lacava <thecave003@gmail.com>
 *          Michele Polese <michele.polese@gmail.com>
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include <ns3/lte-ue-net-device.h>
#include "ns3/mmwave-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"
#include "ns3/lte-helper.h"

using namespace ns3;
using namespace mmwave;

/**
 * Scenario Two
 * 
 */

NS_LOG_COMPONENT_DEFINE ("ScenarioTwo");

void
PrintGnuplottableUeListToFile (std::string filename)
{
  std::ofstream outFile;
  outFile.open (filename.c_str (), std::ios_base::out | std::ios_base::trunc);
  if (!outFile.is_open ())
    {
      NS_LOG_ERROR ("Can't open file " << filename);
      return;
    }
  for (NodeList::Iterator it = NodeList::Begin (); it != NodeList::End (); ++it)
    {
      Ptr<Node> node = *it;
      int nDevs = node->GetNDevices ();
      for (int j = 0; j < nDevs; j++)
        {
          Ptr<LteUeNetDevice> uedev = node->GetDevice (j)->GetObject<LteUeNetDevice> ();
          Ptr<MmWaveUeNetDevice> mmuedev = node->GetDevice (j)->GetObject<MmWaveUeNetDevice> ();
          Ptr<McUeNetDevice> mcuedev = node->GetDevice (j)->GetObject<McUeNetDevice> ();
          if (uedev)
            {
              Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
              outFile << "set label \"" << uedev->GetImsi () << "\" at " << pos.x << "," << pos.y
                      << " left font \"sans,8\" textcolor rgb \"black\" front point pt 1 ps "
                         "0.3 lc rgb \"black\" offset 0,0"
                      << std::endl;
            }
          else if (mmuedev)
            {
              Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
              outFile << "set label \"" << mmuedev->GetImsi () << "\" at " << pos.x << "," << pos.y
                      << " left font \"sans,8\" textcolor rgb \"black\" front point pt 1 ps "
                         "0.3 lc rgb \"black\" offset 0,0"
                      << std::endl;
            }
          else if (mcuedev)
            {
              Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
              outFile << "set label \"" << mcuedev->GetImsi () << "\" at " << pos.x << "," << pos.y
                      << " left font \"sans,8\" textcolor rgb \"black\" front point pt 1 ps "
                         "0.3 lc rgb \"black\" offset 0,0"
                      << std::endl;
            }
        }
    }
}

void
PrintGnuplottableEnbListToFile (std::string filename)
{
  std::ofstream outFile;
  outFile.open (filename.c_str (), std::ios_base::out | std::ios_base::trunc);
  if (!outFile.is_open ())
    {
      NS_LOG_ERROR ("Can't open file " << filename);
      return;
    }
  for (NodeList::Iterator it = NodeList::Begin (); it != NodeList::End (); ++it)
    {
      Ptr<Node> node = *it;
      int nDevs = node->GetNDevices ();
      for (int j = 0; j < nDevs; j++)
        {
          Ptr<LteEnbNetDevice> enbdev = node->GetDevice (j)->GetObject<LteEnbNetDevice> ();
          Ptr<MmWaveEnbNetDevice> mmdev = node->GetDevice (j)->GetObject<MmWaveEnbNetDevice> ();
          if (enbdev)
            {
              Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
              outFile << "set label \"" << enbdev->GetCellId () << "\" at " << pos.x << "," << pos.y
                      << " left font \"sans,8\" textcolor rgb \"blue\" front  point pt 4 ps "
                         "0.3 lc rgb \"blue\" offset 0,0"
                      << std::endl;
            }
          else if (mmdev)
            {
              Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
              outFile << "set label \"" << mmdev->GetCellId () << "\" at " << pos.x << "," << pos.y
                      << " left font \"sans,8\" textcolor rgb \"red\" front  point pt 4 ps "
                         "0.3 lc rgb \"red\" offset 0,0"
                      << std::endl;
            }
        }
    }
}

void
SaveLoadBalanceQoSValueToFile (std::string filename,  Ptr<LteEnbNetDevice> lteEnb, std::vector<Ptr<MmWaveEnbNetDevice>> mmWaveGnbs)
{
  std::ofstream outFile;

  NS_LOG_LOGIC ("Trigger Save Load balancing at " << Simulator::Now ().GetMilliSeconds ());

  outFile.open (filename.c_str (), std::ios_base::app);
  if (!outFile.is_open ())
    {
      NS_LOG_ERROR ("Can't open file " << filename);
      return;
    }

  std::map<uint16_t, double>
      loadBalancingMap; // key is the mmWave cell Id, value is the QoS Load balancing for that cell

  // We calculate first the Load balancing of the LTE Cell
  double lteLoadBalance = 0.0;
  double ueQos;
  for (auto ue : lteEnb->GetRrc ()->GetUeMap ())
    {
      ueQos = lteEnb->GetUeQoS (ue.first); // Here we use the RNTI since all the UEs are connected to the LTE eNB
      lteLoadBalance += ueQos;
      NS_LOG_LOGIC ("LTE Load Balance for " << ue.second->GetImsi () << " is " << ueQos
                                           << ", complex (cell " << lteEnb->GetCellId ()
                                           << ") : " << lteLoadBalance);
    }

  // We know that the QoS of each UE on the mmWave cell is 1 - LTE QoS and we exploit it for calculate the QoS for each cell
  double mmwaLoadBalance;
  for (auto mmWaveGnb : mmWaveGnbs) // Total cost of these nested loops is the number of UEs (same of the LTE loop)
    {
      mmwaLoadBalance = 0.0;
      for (auto ue : mmWaveGnb->GetRrc ()->GetUeMap ())
        {
          ueQos = lteEnb->GetUeQoS (ue.second->GetImsi ());
          mmwaLoadBalance += (1 - ueQos);
          NS_LOG_LOGIC ("MmWave Load Balance for " << ue.second->GetImsi () << " is " << 1 - ueQos
                                                  << ", complex: " << mmwaLoadBalance);
        }
      loadBalancingMap[mmWaveGnb->GetCellId ()] = mmwaLoadBalance;
    }

  // Log everything to file
  // Format is timestamp,ueImsiComplete,QoS.CellLoadBalance.Lte,NrCellId,QoS.CellLoadBalance.Nr
  uint64_t timestamp = lteEnb->GetStartTime () + (uint64_t) Simulator::Now ().GetMilliSeconds ();
  std::string ueImsiString;
  for (auto mmWaveGnb : mmWaveGnbs)
    {
      for (auto ue : mmWaveGnb->GetRrc ()->GetUeMap ())
        {
          ueImsiString = mmWaveGnb->GetImsiString (ue.second->GetImsi());
          outFile << std::to_string (timestamp) << "," << ueImsiString << ","
                  << std::to_string (lteLoadBalance) << ","
                  << std::to_string (mmWaveGnb->GetCellId ()) << ","
                  << std::to_string (loadBalancingMap[mmWaveGnb->GetCellId ()]) << std::endl;
        }
    }

  outFile.close ();
}

static ns3::GlobalValue g_bufferSize ("bufferSize", "RLC tx buffer size (MB)",
                                      ns3::UintegerValue (10),
                                      ns3::MakeUintegerChecker<uint32_t> ());

static ns3::GlobalValue g_rlcAmEnabled ("rlcAmEnabled", "If true, use RLC AM, else use RLC UM",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_PercUEeMBB ("PercUEeMBB",
                                        "Percentage of UEs to deploy for eMBB traffic model",
                                        ns3::DoubleValue (0.3),
                                        ns3::MakeDoubleChecker<double> ());

static ns3::GlobalValue g_PercUEURLLC ("PercUEURLLC",
                                        "Percentage of UEs to deploy for URLLC traffic model",
                                        ns3::DoubleValue (0.3),
                                        ns3::MakeDoubleChecker<double> ());

static ns3::GlobalValue g_qoSeMBB ("qoSeMBB",
                                        "Percentage of UEs to deploy for eMBB traffic model",
                                        ns3::DoubleValue (-1),
                                        ns3::MakeDoubleChecker<double> (-1, 1.0));

static ns3::GlobalValue g_qoSURLLC ("qoSURLLC",
                                        "Percentage of UEs to deploy for URLLC traffic model",
                                        ns3::DoubleValue (-1),
                                        ns3::MakeDoubleChecker<double> (-1, 1.0));

static ns3::GlobalValue g_qoSmIoT ("qoSmIoT",
                                        "Percentage of UEs to deploy for URLLC traffic model",
                                        ns3::DoubleValue (-1),
                                        ns3::MakeDoubleChecker<double> (-1, 1.0));

static ns3::GlobalValue q_useSemaphores ("useSemaphores", "If true, enables the use of semaphores for external environment control",
                                        ns3::BooleanValue (false), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_configuration ("configuration", "Set the RF configuration [0,2]",
                                         ns3::UintegerValue (1),
                                         ns3::MakeUintegerChecker<uint8_t> ());

static ns3::GlobalValue g_ues ("ues", "Number of UEs for each mmWave gNB.", ns3::UintegerValue (7),
                               ns3::MakeUintegerChecker<uint8_t> ());

static ns3::GlobalValue g_simTime ("simTime", "Simulation time in seconds", ns3::DoubleValue (1.9),
                                   ns3::MakeDoubleChecker<double> (0.1, 1000.0));

static ns3::GlobalValue g_policy ("policy", "Placeholder to let the controller understand which policy to implement.\n"
                                          "0 is Single Agent xApp Policy, "
                                          "1 is Random Policy, "
                                          "2 is Throughput based Policy, "
                                          "3 is Sinr based Policy, and 4 is a MultiAgent xApp Policy\n"
                                          "This value is not used in the script, but in the SemCallback", 
                                          ns3::UintegerValue (0),
                                          ns3::MakeUintegerChecker<uint8_t> ());

static ns3::GlobalValue g_controlFileName ("controlFileName", "The path to the control file (can be absolute)",
                                     ns3::StringValue ("qos_actions.csv"), ns3::MakeStringChecker ());

static ns3::GlobalValue g_indicationPeriodicity ("indicationPeriodicity", "E2 Indication Periodicity reports (value in seconds)", ns3::DoubleValue (0.1),
                                   ns3::MakeDoubleChecker<double> (0.01, 2.0));

int
main (int argc, char *argv[])
{

  // std::freopen("stdout.txt", "a", stdout);
  // std::freopen("stderr.txt", "a", stderr);

  LogComponentEnableAll (LOG_PREFIX_ALL);
  LogComponentEnable ("ScenarioTwo", LOG_LEVEL_INFO);
  LogComponentEnable ("LteEnbNetDevice", LOG_LEVEL_INFO);
  LogComponentEnable ("MmWaveEnbNetDevice", LOG_LEVEL_INFO);
  // LogComponentEnable ("OnOffApplication", LOG_LEVEL_INFO);
  LogComponentEnable ("MmWaveBearerStatsCalculator", LOG_LEVEL_FUNCTION);
  LogComponentEnable ("LteStatsCalculator", LOG_LEVEL_FUNCTION);
  LogComponentEnable ("RadioBearerStatsCalculator", LOG_LEVEL_FUNCTION);
  LogComponentEnable ("LteRlcAm", LOG_LEVEL_FUNCTION);
  LogComponentEnable ("MmWaveBearerStatsConnector", LOG_LEVEL_FUNCTION);
  LogComponentEnable ("RadioBearerStatsConnector", LOG_LEVEL_FUNCTION);
  // LogComponentEnable ("MmWaveUeMac", LOG_LEVEL_ALL);
  // LogComponentEnable ("MmWaveEnbMac", LOG_LEVEL_ALL);
  // LogComponentEnable ("LteUeMac", LOG_LEVEL_ALL);
  // LogComponentEnable ("LteEnbMac", LOG_LEVEL_ALL);
  // LogComponentEnable ("MmWaveFlexTtiMacScheduler", LOG_LEVEL_ALL);
  LogComponentEnable ("LteEnbRrc", LOG_LEVEL_INFO);
  // LogComponentEnable ("LteUeRrc", LOG_LEVEL_ALL);
  LogComponentEnable ("McEnbPdcp", LOG_LEVEL_FUNCTION);
  LogComponentEnable ("McUePdcp", LOG_LEVEL_INFO);
  // LogComponentEnable ("RicControlMessage", LOG_LEVEL_ALL);
  // LogComponentEnable ("Asn1Types", LOG_LEVEL_LOGIC);
  // LogComponentEnable ("E2Termination", LOG_LEVEL_LOGIC);
  // LogComponentEnable ("MmWaveSpectrumPhy", LOG_LEVEL_ALL);

  // The maximum X coordinate of the scenario
  double maxXAxis = 4000;
  // The maximum Y coordinate of the scenario
  double maxYAxis = 4000;

  // Command line arguments
  CommandLine cmd;
  cmd.Parse (argc, argv);

  bool harqEnabled = true;

  UintegerValue uintegerValue;
  BooleanValue booleanValue;
  StringValue stringValue;
  DoubleValue doubleValue;

  GlobalValue::GetValueByName ("rlcAmEnabled", booleanValue);
  bool rlcAmEnabled = booleanValue.Get ();
  GlobalValue::GetValueByName ("useSemaphores", booleanValue);
  bool useSemaphores = booleanValue.Get ();
  GlobalValue::GetValueByName ("bufferSize", uintegerValue);
  uint32_t bufferSize = uintegerValue.Get ();
  GlobalValue::GetValueByName ("PercUEeMBB", doubleValue);
  double PercUEeMBB = doubleValue.Get ();
  GlobalValue::GetValueByName ("PercUEURLLC", doubleValue);
  double PercUEURLLC = doubleValue.Get ();
  GlobalValue::GetValueByName ("qoSeMBB", doubleValue);
  double qoSeMBB = doubleValue.Get ();
  GlobalValue::GetValueByName ("qoSURLLC", doubleValue);
  double qoSURLLC = doubleValue.Get ();
  GlobalValue::GetValueByName ("qoSmIoT", doubleValue);
  double qoSmIoT = doubleValue.Get ();
  GlobalValue::GetValueByName ("controlFileName", stringValue);
  std::string controlFilename = stringValue.Get ();
  GlobalValue::GetValueByName ("indicationPeriodicity", doubleValue);
  double indicationPeriodicity = doubleValue.Get ();

  if (PercUEeMBB + PercUEURLLC > 1)
    {
      NS_FATAL_ERROR ("The total percentage of UEs for each traffic model is higher than 1: "
                      << PercUEeMBB + PercUEURLLC);
    }

  NS_LOG_INFO ("rlcAmEnabled " << rlcAmEnabled << " bufferSize " << bufferSize
                                 << " percentage UEs eMBB " << PercUEeMBB
                                 << " percentage UEs URLLC " << PercUEURLLC
                                 << " QoS percentage eMBB " << qoSeMBB
                                 << " QoS percentage URLLC " << qoSURLLC 
                                 << " QoS percentage mIoT " << qoSmIoT
                                 << " controlFilename " << controlFilename
                                 << " useSemaphores " << useSemaphores
                                 << " indicationPeriodicity " << indicationPeriodicity);

  Config::SetDefault ("ns3::MmWaveHelper::E2ModeLte", BooleanValue(true));
  Config::SetDefault ("ns3::MmWaveHelper::E2ModeNr", BooleanValue(true));
  Config::SetDefault ("ns3::MmWaveHelper::E2Periodicity", DoubleValue (indicationPeriodicity));
  
  // The DU PM reports should come from both NR gNB as well as LTE eNB, 
  // since in the RLC/MAC/PHY entities are present in BOTH NR gNB as well as LTE eNB.
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableDuReport", BooleanValue(true));

  Config::SetDefault ("ns3::LteEnbNetDevice::ControlFileName", StringValue (controlFilename));

  Config::SetDefault ("ns3::LteEnbNetDevice::UseSemaphores", BooleanValue (useSemaphores));

  // The CU-UP PM reports should only come from LTE eNB, since in the NS3 “EN-DC 
  // simulation (Option 3A)”, the PDCP is only in the LTE eNB and NOT in the NR gNB
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableCuUpReport", BooleanValue(true));
  Config::SetDefault ("ns3::LteEnbNetDevice::EnableCuUpReport", BooleanValue(true));

  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableCuCpReport", BooleanValue(true));
  Config::SetDefault ("ns3::LteEnbNetDevice::EnableCuCpReport", BooleanValue(true));

  Config::SetDefault ("ns3::LteEnbNetDevice::EnableE2FileLogging", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableE2FileLogging", BooleanValue (true));

  Config::SetDefault ("ns3::MmWaveEnbMac::NumberOfRaPreambles", UintegerValue (40));
  Config::SetDefault ("ns3::LteEnbMac::NumberOfRaPreambles", UintegerValue (40));

  Config::SetDefault ("ns3::MmWaveHelper::RlcAmEnabled", BooleanValue (rlcAmEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::UseIdealRrc", BooleanValue (true));

  Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::NumHarqProcess", UintegerValue (100));

  Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (100.0)));
  Config::SetDefault ("ns3::ThreeGppChannelConditionModel::UpdatePeriod", TimeValue (MilliSeconds (100)));

  Config::SetDefault ("ns3::LteRlcAm::ReportBufferStatusTimer", TimeValue (MilliSeconds (10.0)));
  Config::SetDefault ("ns3::LteRlcUmLowLat::ReportBufferStatusTimer",
                      TimeValue (MilliSeconds (10.0)));
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (bufferSize * 1024 * 1024));
  Config::SetDefault ("ns3::LteRlcUmLowLat::MaxTxBufferSize",
                      UintegerValue (bufferSize * 1024 * 1024));
  Config::SetDefault ("ns3::LteRlcAm::MaxTxBufferSize", UintegerValue (bufferSize * 1024 * 1024));

  // Carrier bandwidth in Hz
  double bandwidth;
  // Center frequency in Hz
  double centerFrequency;
  // Distance between the mmWave BSs and the two co-located LTE and mmWave BSs in meters
  double isd; // (interside distance)
  // Number of antennas in each UE
  int numAntennasMcUe;
  // Number of antennas in each mmWave BS
  int numAntennasMmWave;

  GlobalValue::GetValueByName ("configuration", uintegerValue);
  uint8_t configuration = uintegerValue.Get ();
  switch (configuration)
    {
    case 0:
      centerFrequency = 850e6;
      bandwidth = 20e6;
      isd = 1000;
      numAntennasMcUe = 1;
      numAntennasMmWave = 1;
      break;

    case 1:
      centerFrequency = 3.5e9;
      bandwidth = 20e6;
      isd = 1000;
      numAntennasMcUe = 1;
      numAntennasMmWave = 1;
      break;

    case 2:
      centerFrequency = 28e9;
      bandwidth = 100e6;
      isd = 200;
      numAntennasMcUe = 16;
      numAntennasMmWave = 64;
      break;

    default:
      NS_FATAL_ERROR ("Configuration not recognized" << configuration);
      break;
    }

  Config::SetDefault ("ns3::MmWavePhyMacCommon::Bandwidth", DoubleValue (bandwidth));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue (centerFrequency));

  Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper> ();
  mmwaveHelper->SetPathlossModelType ("ns3::ThreeGppUmiStreetCanyonPropagationLossModel");
  mmwaveHelper->SetChannelConditionModelType ("ns3::ThreeGppUmiStreetCanyonChannelConditionModel");

  // Set the number of antennas in the devices
  mmwaveHelper->SetUePhasedArrayModelAttribute("NumColumns", UintegerValue(std::sqrt(numAntennasMcUe)));
  mmwaveHelper->SetUePhasedArrayModelAttribute("NumRows", UintegerValue(std::sqrt(numAntennasMcUe)));
  mmwaveHelper->SetEnbPhasedArrayModelAttribute("NumColumns",UintegerValue(std::sqrt(numAntennasMmWave)));
  mmwaveHelper->SetEnbPhasedArrayModelAttribute("NumRows", UintegerValue(std::sqrt(numAntennasMmWave)));

  Ptr<MmWavePointToPointEpcHelper> epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
  mmwaveHelper->SetEpcHelper (epcHelper);

  uint8_t nMmWaveEnbNodes = 7;
  uint8_t nLteEnbNodes = 1;
  GlobalValue::GetValueByName ("ues", uintegerValue);
  uint32_t ues = uintegerValue.Get ();
  uint8_t nUeNodes = ues * nMmWaveEnbNodes;

  NS_LOG_INFO (" Bandwidth " << bandwidth << " centerFrequency " << centerFrequency << " isd "
                             << isd << " numAntennasMcUe " << numAntennasMcUe
                             << " numAntennasMmWave " << numAntennasMmWave << " nMmWaveEnbNodes "
                             << unsigned (nMmWaveEnbNodes) << " nUeNodes " << unsigned (nUeNodes));

  // Get SGW/PGW and create a single RemoteHost
  Ptr<Node> pgw = epcHelper->GetPgwNode ();
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  // Create the Internet by connecting remoteHost to pgw. Setup routing too
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (2500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  // Ipv4InterfaceContainer internetIpIfaces = 
  ipv4h.Assign (internetDevices);
  // interface 0 is localhost, 1 is the p2p device
  // Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  // create LTE, mmWave eNB nodes and UE node
  NodeContainer ueNodes;
  NodeContainer mmWaveEnbNodes;
  NodeContainer lteEnbNodes;
  NodeContainer allEnbNodes;
  mmWaveEnbNodes.Create (nMmWaveEnbNodes);
  lteEnbNodes.Create (nLteEnbNodes);
  ueNodes.Create (nUeNodes);
  allEnbNodes.Add (lteEnbNodes);
  allEnbNodes.Add (mmWaveEnbNodes);

  // Position
  Vector centerPosition = Vector (maxXAxis / 2, maxYAxis / 2, 3);

  // Install Mobility Model
  Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();

  // We want a center with one LTE enb and one mmWave co-located in the same place
  enbPositionAlloc->Add (centerPosition);
  enbPositionAlloc->Add (centerPosition);

  double x, y;
  double nConstellation = nMmWaveEnbNodes - 1;

  // This guarantee that each of the rest BSs is placed at the same distance from the two co-located in the center
  for (int8_t i = 0; i < nConstellation; ++i)
    {
      x = isd * cos ((2 * M_PI * i) / (nConstellation));
      y = isd * sin ((2 * M_PI * i) / (nConstellation));
      enbPositionAlloc->Add (Vector (centerPosition.x + x, centerPosition.y + y, 3));
    }

  MobilityHelper enbmobility;
  enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  enbmobility.SetPositionAllocator (enbPositionAlloc);
  enbmobility.Install (allEnbNodes);

  Ptr<UniformDiscPositionAllocator> uePositionAlloc = CreateObject<UniformDiscPositionAllocator> ();
  uePositionAlloc->SetX (centerPosition.x);
  uePositionAlloc->SetY (centerPosition.y);
  uePositionAlloc->SetRho (0.7 * isd);

  uint32_t countUe;
  // eMBB
  for (countUe = 0; countUe < ueNodes.GetN () * PercUEeMBB; countUe++)
    {
      MobilityHelper uemobilityeMBB;

      // low mobility m/s
      Ptr<UniformRandomVariable> speed = CreateObject<UniformRandomVariable> ();
      speed->SetAttribute ("Min", DoubleValue (0.3)); // 1 km/h
      speed->SetAttribute ("Max", DoubleValue (3)); // 10 km/h

      uemobilityeMBB.SetMobilityModel ("ns3::RandomWalk2dOutdoorMobilityModel", "Speed",
                                       PointerValue (speed), "Bounds",
                                       RectangleValue (Rectangle (0, maxXAxis, 0, maxYAxis)));

      uemobilityeMBB.SetPositionAllocator (uePositionAlloc);
      uemobilityeMBB.Install (ueNodes.Get (countUe));
      NS_LOG_INFO ("Node " << countUe + 1 << " is eMBB");
    }

  // URLLC
  for (; countUe < ueNodes.GetN () * PercUEeMBB + ueNodes.GetN () * PercUEURLLC; countUe++)
    { 
      MobilityHelper uemobilityURLLC;
      // high mobility m/s
      Ptr<UniformRandomVariable> speed = CreateObject<UniformRandomVariable> ();
      speed->SetAttribute ("Min", DoubleValue (3)); // 10 km/h
      speed->SetAttribute ("Max", DoubleValue (13.8)); // 50 km/h

      uemobilityURLLC.SetMobilityModel ("ns3::RandomWalk2dOutdoorMobilityModel", "Speed",
                                  PointerValue (speed), "Bounds",
                                  RectangleValue (Rectangle (0, maxXAxis, 0, maxYAxis)));
      uemobilityURLLC.SetPositionAllocator (uePositionAlloc);
      uemobilityURLLC.Install (ueNodes.Get (countUe));
      NS_LOG_INFO ("Node " << countUe + 1 << " is uRLLC");
    }

  // mIoT 
  for (; countUe < ueNodes.GetN (); countUe++)
    {
      MobilityHelper uemobilitymIoT;
      // static mobility m/s
      uemobilitymIoT.SetPositionAllocator (uePositionAlloc);
      uemobilitymIoT.Install (ueNodes.Get (countUe));
      NS_LOG_INFO ("Node " << countUe + 1 << " is mIoT");
    }

  // Install mmWave, lte, mc Devices to the nodes
  NetDeviceContainer lteEnbDevs = mmwaveHelper->InstallLteEnbDevice (lteEnbNodes);
  NetDeviceContainer mmWaveEnbDevs = mmwaveHelper->InstallEnbDevice (mmWaveEnbNodes);
  NetDeviceContainer mcUeDevs = mmwaveHelper->InstallMcUeDevice (ueNodes);

  // Install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (mcUeDevs));
  // Assign IP address to UEs, and install applications
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting =
          ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

  // Add X2 interfaces
  mmwaveHelper->AddX2Interface (lteEnbNodes, mmWaveEnbNodes);

  // Manual attachment
  mmwaveHelper->AttachToClosestEnb (mcUeDevs, mmWaveEnbDevs, lteEnbDevs);

  // Install and start applications
  ApplicationContainer sinkApp;
  
  // In the traffic models loop we also set the QoS parameter
  Ptr<LteEnbNetDevice> eNBDevice = DynamicCast<LteEnbNetDevice> (lteEnbDevs.Get (0));

  ApplicationContainer clientApp;
  PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory",
                                       InetSocketAddress (Ipv4Address::GetAny (), 1234));

  // eMBB
  for (countUe = 0; countUe < ueNodes.GetN () * PercUEeMBB; countUe++)
    {
      sinkApp.Add (dlPacketSinkHelper.Install (ueNodes.Get (countUe)));
      UdpClientHelper embbClient (ueIpIface.GetAddress (countUe), 1234);
      embbClient.SetAttribute ("Interval", TimeValue (MicroSeconds (2560))); // 4 Mbit/s
      embbClient.SetAttribute ("MaxPackets", UintegerValue (0)); // Zero means infinite
      embbClient.SetAttribute ("PacketSize", UintegerValue (1280));
      clientApp.Add (embbClient.Install (remoteHost));

      // QoS traffic split setup
      if (qoSeMBB != -1)
        {
          Ptr<McUeNetDevice> ueDevice = DynamicCast<McUeNetDevice> (ueNodes.Get (countUe));
          uint64_t ueIdRnti = eNBDevice->GetRrc ()->GetRntiFromImsi (ueDevice->GetImsi ());
          NS_LOG_INFO ("[eMBB] Setting UE with IMSI " << ueDevice->GetImsi () << " PDCP split to "
                                                        << qoSeMBB);
          Simulator::Schedule (MilliSeconds (100), &LteEnbNetDevice::SetUeQoS, eNBDevice, ueIdRnti,
                               qoSeMBB);
        }
    }

  // URLLC
  for (countUe++; countUe < ueNodes.GetN () * PercUEeMBB + ueNodes.GetN () * PercUEURLLC; countUe++)
    {
      sinkApp.Add (dlPacketSinkHelper.Install (ueNodes.Get (countUe)));
  
      OnOffHelper onOffAppUrllc ("ns3::UdpSocketFactory",
                                 InetSocketAddress (ueIpIface.GetAddress (countUe), 1234));
      onOffAppUrllc.SetAttribute ("PacketSize", UintegerValue (128));
      onOffAppUrllc.SetAttribute ("OnTime", StringValue ("ns3::ExponentialRandomVariable[Mean=0.01]"));
      onOffAppUrllc.SetAttribute ("OffTime",  StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
      onOffAppUrllc.SetAttribute ("DataRate", StringValue ("89.3kbps"));
      clientApp.Add (onOffAppUrllc.Install (remoteHost));

      // QoS traffic split setup
      if (qoSURLLC != -1)
        {
          Ptr<McUeNetDevice> ueDevice = DynamicCast<McUeNetDevice> (ueNodes.Get (countUe));
          uint64_t ueIdRnti = eNBDevice->GetRrc ()->GetRntiFromImsi (ueDevice->GetImsi ());
          NS_LOG_INFO ("[URLLC] Setting UE with IMSI " << ueDevice->GetImsi ()
                                                         << " PDCP split to " << qoSURLLC);
          Simulator::Schedule (MilliSeconds (100), &LteEnbNetDevice::SetUeQoS, eNBDevice, ueIdRnti,
                               qoSURLLC);
        }
    }

  // mIoT
  for (countUe++; countUe < ueNodes.GetN (); countUe++)
    {
      sinkApp.Add (dlPacketSinkHelper.Install (ueNodes.Get (countUe)));
      OnOffHelper onOffAppMIoT ("ns3::UdpSocketFactory",
                                InetSocketAddress (ueIpIface.GetAddress (countUe), 1234));
      onOffAppMIoT.SetAttribute ("PacketSize", UintegerValue (128));
      onOffAppMIoT.SetAttribute ("OnTime", StringValue ("ns3::ExponentialRandomVariable[Mean=0.01]"));
      onOffAppMIoT.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
      onOffAppMIoT.SetAttribute ("DataRate", StringValue ("44.6kbps"));
      clientApp.Add (onOffAppMIoT.Install (remoteHost));

      // QoS traffic split setup
      if (qoSmIoT != -1)
        {
          Ptr<McUeNetDevice> ueDevice = DynamicCast<McUeNetDevice> (ueNodes.Get (countUe));
          uint64_t ueIdRnti = eNBDevice->GetRrc ()->GetRntiFromImsi (ueDevice->GetImsi ());
          NS_LOG_INFO ("[mIot] Setting UE with IMSI " << ueDevice->GetImsi () << " PDCP split to "
                                                        << qoSmIoT);
          Simulator::Schedule (MilliSeconds (100), &LteEnbNetDevice::SetUeQoS, eNBDevice, ueIdRnti,
                               qoSmIoT);
        }
    }

  // Start applications
  GlobalValue::GetValueByName ("simTime", doubleValue);
  double simTime = doubleValue.Get ();
  sinkApp.Start (Seconds (0));
  
  clientApp.Start (MilliSeconds (50));
  clientApp.Stop (Seconds (simTime));

  mmwaveHelper->EnableTraces ();

  // trick to enable PHY traces for the LTE stack
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  lteHelper->Initialize ();
  lteHelper->EnablePhyTraces ();
  lteHelper->EnableMacTraces ();
  // Since nodes are randomly allocated during each run we always need to print their positions
  PrintGnuplottableUeListToFile ("ues.txt");
  PrintGnuplottableEnbListToFile ("enbs.txt");

  // Create file to save load balancing values
  std::string loadBalanceFilename = "QoSLoadBalancing.txt";

  std::ofstream header_file (loadBalanceFilename.c_str ());
  header_file << "timestamp,ueImsiComplete,QoS.CellLoadBalance.Lte,NrCellId,QoS.CellLoadBalance.Nr"
              << std::endl;
  header_file.close ();

  std::vector<Ptr<MmWaveEnbNetDevice>> mmWaveGnbs;
  Ptr<MmWaveEnbNetDevice> tempMmWaveGnb;
  for (uint32_t i = 0; i < mmWaveEnbDevs.GetN (); ++i)
    {
      tempMmWaveGnb = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (i));
      if (tempMmWaveGnb)
        {
          mmWaveGnbs.push_back (tempMmWaveGnb);
        }
      else
        {
          NS_FATAL_ERROR ("DynamicCast of MmWaveEnbNetDevice failed miserably for " << unsigned (i));
        }
    }

  // Schedule the tracing of the load balancing values with the same indication periodicity to match the timestamps 
  double nReports = simTime / indicationPeriodicity; // Number of reports
  double scheduleTime;
  for (int i = 1; i <= nReports; ++i)
    {
      scheduleTime = (i * indicationPeriodicity);
      NS_LOG_INFO ("Schedule LoadBalancing report at time " << scheduleTime << " s");
      Simulator::Schedule (Seconds (scheduleTime), &SaveLoadBalanceQoSValueToFile,
                           loadBalanceFilename, eNBDevice, mmWaveGnbs);
    }


  double offset = 0.005;
  NS_LOG_INFO ("Simulation time is " << simTime + offset << " seconds ");
  Simulator::Stop (Seconds (simTime + offset));
  Simulator::Run ();
  NS_LOG_INFO (lteHelper);

  Simulator::Destroy ();
  NS_LOG_INFO ("Done.");
  return 0;
}
