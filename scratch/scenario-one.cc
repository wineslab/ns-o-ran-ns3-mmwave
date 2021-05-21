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
 * Author: Michele Polese <michele.polese@gmail.com>
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

using namespace ns3;
using namespace mmwave;

/**
 * Scenario One
 * 
 */

NS_LOG_COMPONENT_DEFINE ("ScenarioOne");

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
                      << " left font \"Helvetica,8\" textcolor rgb \"black\" front point pt 1 ps "
                         "0.3 lc rgb \"black\" offset 0,0"
                      << std::endl;
            }
          else if (mmuedev)
            {
              Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
              outFile << "set label \"" << mmuedev->GetImsi () << "\" at " << pos.x << "," << pos.y
                      << " left font \"Helvetica,8\" textcolor rgb \"black\" front point pt 1 ps "
                         "0.3 lc rgb \"black\" offset 0,0"
                      << std::endl;
            }
          else if (mcuedev)
            {
              Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
              outFile << "set label \"" << mcuedev->GetImsi () << "\" at " << pos.x << "," << pos.y
                      << " left font \"Helvetica,8\" textcolor rgb \"black\" front point pt 1 ps "
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
                      << " left font \"Helvetica,8\" textcolor rgb \"blue\" front  point pt 4 ps "
                         "0.3 lc rgb \"blue\" offset 0,0"
                      << std::endl;
            }
          else if (mmdev)
            {
              Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
              outFile << "set label \"" << mmdev->GetCellId () << "\" at " << pos.x << "," << pos.y
                      << " left font \"Helvetica,8\" textcolor rgb \"red\" front  point pt 4 ps "
                         "0.3 lc rgb \"red\" offset 0,0"
                      << std::endl;
            }
        }
    }
}

void
PrintPosition (Ptr<Node> node)
{
  Ptr<MobilityModel> model = node->GetObject<MobilityModel> ();
  NS_LOG_UNCOND ("Position +****************************** " << model->GetPosition () << " at time "
                                                             << Simulator::Now ().GetSeconds ());
}

static ns3::GlobalValue g_bufferSize ("bufferSize", "RLC tx buffer size (MB)",
                                      ns3::UintegerValue (10),
                                      ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_rlcAmEnabled ("rlcAmEnabled", "If true, use RLC AM, else use RLC UM",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_configuration ("configuration",
                                         "Set the wanted configuration to emulate [0,2]",
                                         ns3::UintegerValue (1),
                                         ns3::MakeUintegerChecker<uint8_t> ());

static ns3::GlobalValue g_perPckToLTE ("perPckToLTE",
                                       "Percentage of packets to be directed to LTE.",
                                       ns3::DoubleValue (0.5),
                                       ns3::MakeDoubleChecker<double> (-1, 1.0));

static ns3::GlobalValue g_ues ("ues", "Number of UEs for each mmWave ENB.", ns3::UintegerValue (7),
                               ns3::MakeUintegerChecker<uint8_t> ());

static ns3::GlobalValue g_simTime ("simTime", "Simulation time in seconds", ns3::DoubleValue (6.0),
                                   ns3::MakeDoubleChecker<double> (0.1, 1000.0));

int
main (int argc, char *argv[])
{
  LogComponentEnableAll (LOG_PREFIX_ALL);
  //LogComponentEnable ("PacketSink", LOG_LEVEL_ALL);
  //LogComponentEnable ("OnOffApplication", LOG_LEVEL_ALL);
  LogComponentEnable ("LtePdcp", LOG_LEVEL_ALL);
  LogComponentEnable ("LteRlcAm", LOG_LEVEL_ALL);
  LogComponentEnable ("MmWaveUeMac", LOG_LEVEL_ALL);
  LogComponentEnable ("MmWaveEnbMac", LOG_LEVEL_ALL);
  LogComponentEnable ("LteUeMac", LOG_LEVEL_ALL);
  LogComponentEnable ("LteEnbMac", LOG_LEVEL_ALL);
  LogComponentEnable ("MmWaveFlexTtiMacScheduler", LOG_LEVEL_ALL);
  LogComponentEnable ("LteEnbRrc", LOG_LEVEL_ALL);
  LogComponentEnable ("LteUeRrc", LOG_LEVEL_ALL);
  // LogComponentEnable ("McEnbPdcp", LOG_LEVEL_ALL);
  // LogComponentEnable ("McUePdcp", LOG_LEVEL_ALL);
  LogComponentEnable ("MmWaveSpectrumPhy", LOG_LEVEL_ALL);
  LogComponentEnable ("ScenarioOne", LOG_LEVEL_ALL);

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

  GlobalValue::GetValueByName ("perPckToLTE", doubleValue);
  double perPckToLTE = doubleValue.Get ();
  GlobalValue::GetValueByName ("rlcAmEnabled", booleanValue);
  bool rlcAmEnabled = booleanValue.Get ();
  GlobalValue::GetValueByName ("bufferSize", uintegerValue);
  uint32_t bufferSize = uintegerValue.Get ();

  NS_LOG_UNCOND ("rlcAmEnabled " << rlcAmEnabled << " bufferSize " << bufferSize);

  //get current time
  time_t rawtime;
  struct tm *timeinfo;
  char buffer[80];
  time (&rawtime);
  timeinfo = localtime (&rawtime);
  strftime (buffer, 80, "%d_%m_%Y_%I_%M_%S", timeinfo);
  std::string time_str (buffer);

  Config::SetDefault ("ns3::MmWaveHelper::RlcAmEnabled", BooleanValue (rlcAmEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::UseIdealRrc", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::NumHarqProcess", UintegerValue (100));
  Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (100.0)));
  Config::SetDefault ("ns3::ThreeGppChannelConditionModel::UpdatePeriod",
                      TimeValue (MilliSeconds (100)));
  Config::SetDefault ("ns3::LteRlcAm::ReportBufferStatusTimer", TimeValue (MilliSeconds (10.0)));
  Config::SetDefault ("ns3::LteRlcUmLowLat::ReportBufferStatusTimer",
                      TimeValue (MilliSeconds (10.0)));
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (bufferSize * 1024 * 1024));
  Config::SetDefault ("ns3::LteRlcUmLowLat::MaxTxBufferSize",
                      UintegerValue (bufferSize * 1024 * 1024));
  Config::SetDefault ("ns3::LteRlcAm::MaxTxBufferSize", UintegerValue (bufferSize * 1024 * 1024));
  Config::SetDefault ("ns3::McEnbPdcp::perPckToLTE", DoubleValue (perPckToLTE));

  // set to false to use the 3GPP radiation pattern (proper configuration of the bearing and downtilt angles is needed)
  Config::SetDefault ("ns3::ThreeGppAntennaArrayModel::IsotropicElements", BooleanValue (true));

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
  // Data rate of transport layer
  std::string dataRate;

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
      dataRate = "3Mbps";
      break;

    case 1:
      centerFrequency = 3.5e9;
      bandwidth = 20e6;
      isd = 1000;
      numAntennasMcUe = 1;
      numAntennasMmWave = 1;
      dataRate = "3Mbps";
      break;

    case 2:
      centerFrequency = 28e9;
      bandwidth = 100e6;
      isd = 200;
      numAntennasMcUe = 16;
      numAntennasMmWave = 64;
      dataRate = "30Mbps";
      break;

    default:
      NS_FATAL_ERROR ("Configuration not recognized" << configuration);
      break;
    }

  NS_LOG_INFO ("Bandwidth " << bandwidth << " centerFrequency " << double (centerFrequency)
                            << " isd " << isd << " numAntennasMcUe " << numAntennasMcUe
                            << " numAntennasMmWave " << numAntennasMmWave << " dataRate "
                            << dataRate);

  // set the number of antennas in the devices
  Config::SetDefault ("ns3::McUeNetDevice::AntennaNum", UintegerValue (numAntennasMcUe));
  Config::SetDefault ("ns3::MmWaveNetDevice::AntennaNum", UintegerValue (numAntennasMmWave));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::Bandwidth", DoubleValue (bandwidth));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue (centerFrequency));

  Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper> ();
  mmwaveHelper->SetPathlossModelType ("ns3::ThreeGppUmiStreetCanyonPropagationLossModel");
  mmwaveHelper->SetChannelConditionModelType ("ns3::ThreeGppUmiStreetCanyonChannelConditionModel");

  Ptr<MmWavePointToPointEpcHelper> epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
  mmwaveHelper->SetEpcHelper (epcHelper);

  uint8_t nMmWaveEnbNodes = 7;
  uint8_t nLteEnbNodes = 1;
  GlobalValue::GetValueByName ("ues", uintegerValue);
  uint32_t ues = uintegerValue.Get ();
  uint8_t nUeNodes = ues * nMmWaveEnbNodes;

  NS_LOG_INFO (" Bandwidth " << bandwidth << " centerFrequency " << double (centerFrequency)
                             << " isd " << isd << " numAntennasMcUe " << numAntennasMcUe
                             << " numAntennasMmWave " << numAntennasMmWave << " dataRate "
                             << dataRate << " nMmWaveEnbNodes " << unsigned (nMmWaveEnbNodes));

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
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  // interface 0 is localhost, 1 is the p2p device
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);
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

  double x;
  double y;
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

  MobilityHelper uemobility;

  Ptr<UniformDiscPositionAllocator> uePositionAlloc = CreateObject<UniformDiscPositionAllocator> ();

  uePositionAlloc->SetX (centerPosition.x);
  uePositionAlloc->SetY (centerPosition.y);
  uePositionAlloc->SetRho (isd);

  uemobility.SetMobilityModel ("ns3::RandomWalk2dOutdoorMobilityModel", "Bounds",
                               RectangleValue (Rectangle (0, maxXAxis, 0, maxYAxis)));
  uemobility.SetPositionAllocator (uePositionAlloc);
  uemobility.Install (ueNodes);

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

  // Install and start applications on UEs

  // On the remoteHost is placed a TCP server
  uint16_t portTcp = 50000;
  Address serverLocalAddressTcp (InetSocketAddress (Ipv4Address::GetAny (), portTcp));
  OnOffHelper serverHelperTcp ("ns3::TcpSocketFactory", serverLocalAddressTcp);
  AddressValue serverAddressTcp (InetSocketAddress (remoteHostAddr, portTcp));
  serverHelperTcp.SetAttribute ("OnTime", StringValue ("ns3::ExponentialRandomVariable"));
  serverHelperTcp.SetAttribute ("OffTime", StringValue ("ns3::ExponentialRandomVariable"));
  serverHelperTcp.SetAttribute ("DataRate", StringValue (dataRate));
  serverHelperTcp.SetAttribute ("PacketSize", UintegerValue (1280));

  // On the remoteHost is placed a UDP server
  uint16_t portUdp = 60000;
  Address serverLocalAddressUdp (InetSocketAddress (Ipv4Address::GetAny (), portUdp));
  OnOffHelper serverHelperUdp ("ns3::UdpSocketFactory", serverLocalAddressUdp);
  AddressValue serverAddressUdp (InetSocketAddress (remoteHostAddr, portUdp));
  serverHelperUdp.SetAttribute ("OnTime", StringValue ("ns3::ExponentialRandomVariable"));
  serverHelperUdp.SetAttribute ("OffTime", StringValue ("ns3::ExponentialRandomVariable"));
  serverHelperUdp.SetAttribute ("DataRate", StringValue (dataRate));
  serverHelperUdp.SetAttribute ("PacketSize", UintegerValue (1280));

  ApplicationContainer serverApp;
  serverApp.Add (serverHelperTcp.Install (remoteHost));
  serverApp.Add (serverHelperUdp.Install (remoteHost));

  // On the UEs there are TCP and UDP clients
  // If needed [Mean=1,Bound=0]
  PacketSinkHelper clientHelperTcp ("ns3::TcpSocketFactory", Address ());
  clientHelperTcp.SetAttribute ("Remote", serverAddressTcp);

  PacketSinkHelper clientHelperUdp ("ns3::UdpSocketFactory", Address ());
  clientHelperUdp.SetAttribute ("Remote", serverAddressUdp);

  // Half of the nodes uses an UDP client and the other half a TCP client
  ApplicationContainer clientApp;
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      if (u % 2 == 0)
        {
          clientApp.Add (clientHelperTcp.Install (ueNodes.Get (u)));
        }
      else
        {
          clientApp.Add (clientHelperUdp.Install (ueNodes.Get (u)));
        }
    }

  // Start applications
  GlobalValue::GetValueByName ("simTime", doubleValue);
  double simTime = doubleValue.Get ();

  serverApp.Start (Seconds (0));
  serverApp.Stop (Seconds (simTime - 1));

  clientApp.Start (MilliSeconds (100));
  clientApp.Stop (Seconds (simTime - 1));

  // int numPrints = 5;
  // for (int i = 0; i < numPrints; i++)
  //   {
  //     for (uint32_t j = 0; j < ueNodes.GetN (); j++)
  //       {
  //         Simulator::Schedule (Seconds (i * simTime / numPrints), &PrintPosition, ueNodes.Get (j));
  //       }
  //   }

  mmwaveHelper->EnableTraces ();

  // Since nodes are randomly allocated during each run we always need to print their positions
  PrintGnuplottableUeListToFile ("ues.txt");
  PrintGnuplottableEnbListToFile ("enbs.txt");

  bool run = true;
  if (run)
    {
      NS_LOG_UNCOND ("Simulation time is " << simTime << " seconds ");
      Simulator::Stop (Seconds (simTime));
      NS_LOG_INFO ("Run Simulation.");
      Simulator::Run ();
    }

  Simulator::Destroy ();
  NS_LOG_INFO ("Done.");
  return 0;
}
