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
 *          Matteo Bordin <matbord97@gmail.com>
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
#include "ns3/energy-heuristic.h"

using namespace ns3;
using namespace mmwave;

/**
 * Scenario Five
 * 
 */

NS_LOG_COMPONENT_DEFINE ("ScenarioFive");

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
                      << " left font \"Helvetica,15\" textcolor rgb \"black\" front point pt 4 ps "
                         "0.3 lc rgb \"black\" offset 0,0"
                      << std::endl;
            }
          else if (mmuedev)
            {
              Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
              outFile << "set label \"" << mmuedev->GetImsi () << "\" at " << pos.x << "," << pos.y
                      << " left font \"Helvetica,15\" textcolor rgb \"black\" front point pt 4 ps "
                         "0.3 lc rgb \"black\" offset 0,0"
                      << std::endl;
            }
          else if (mcuedev)
            {
              Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
              outFile << "set label \"" << mcuedev->GetImsi () << "\" at " << pos.x << "," << pos.y
                      << " left font \"Helvetica,15\" textcolor rgb \"black\" front point pt 4 ps "
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
                      << " left font \"Helvetica,24\" textcolor rgb \"blue\" front  point pt 8 ps "
                         "0.3 lc rgb \"blue\" offset 0,0"
                      << std::endl;
            }
          else if (mmdev)
            {
              Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
              outFile << "set label \"" << mmdev->GetCellId () << "\" at " << pos.x << "," << pos.y
                      << " left font \"Helvetica,24\" textcolor rgb \"red\" front  point pt 8 ps "
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
  NS_LOG_UNCOND ("Position ****************************** " << model->GetPosition () << " at time "
                                                             << Simulator::Now ().GetSeconds ());
}

static ns3::GlobalValue g_bufferSize ("bufferSize", "RLC tx buffer size (MB)",
                                      ns3::UintegerValue (10),
                                      ns3::MakeUintegerChecker<uint32_t> ());

static ns3::GlobalValue g_rlcAmEnabled ("rlcAmEnabled", "If true, use RLC AM, else use RLC UM",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_enableTraces ("enableTraces", "If true, generate ns-3 traces",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());
                                                                                
static ns3::GlobalValue g_e2lteEnabled ("e2lteEnabled", "If true, send LTE E2 reports",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_e2nrEnabled ("e2nrEnabled", "If false, send NR E2 reports",
                                        ns3::BooleanValue (false), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_e2du ("e2du", "If true, send DU reports",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_e2cuUp ("e2cuUp", "If true, send CU-UP reports",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_e2cuCp ("e2cuCp", "If true, send CU-CP reports",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_configuration ("configuration",
                                         "Set the wanted configuration to emulate [0,2]",
                                         ns3::UintegerValue (1),
                                         ns3::MakeUintegerChecker<uint8_t> ());

static ns3::GlobalValue
    g_dataRate ("dataRate", "Set the data rate to be used [only \"0\"(low),\"1\"(high) admitted]",
                ns3::DoubleValue (0), ns3::MakeDoubleChecker<double> (0, 1));

static ns3::GlobalValue g_ues ("ues", "Number of UEs for each mmWave ENB.", ns3::UintegerValue (7),
                               ns3::MakeUintegerChecker<uint8_t> ());

static ns3::GlobalValue g_indicationPeriodicity ("indicationPeriodicity", "E2 Indication Periodicity reports (value in seconds)", ns3::DoubleValue (0.1),
                                   ns3::MakeDoubleChecker<double> (0.01, 2.0));

static ns3::GlobalValue g_simTime ("simTime", "Simulation time in seconds", ns3::DoubleValue (1.9),
                                   ns3::MakeDoubleChecker<double> (0.1, 1000.0));


static ns3::GlobalValue g_reducedPmValues ("reducedPmValues", "If true, use a subset of the the pm containers",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_outageThreshold ("outageThreshold",
                                           "SNR threshold for outage events [dB]",
                                           ns3::DoubleValue (-1000.0),
                                           ns3::MakeDoubleChecker<double> ());

static ns3::GlobalValue g_basicCellId ("basicCellId", "The next value will be the first cellId",
                                       ns3::UintegerValue (1),
                                       ns3::MakeUintegerChecker<uint8_t> ());

static ns3::GlobalValue g_numberOfRaPreambles ("numberOfRaPreambles", "how many random access preambles are available for the contention based RACH process",
                                       ns3::UintegerValue (30), // TS use case should be 40, 52 is default, ES should be 30
                                       ns3::MakeUintegerChecker<uint8_t> ());

static ns3::GlobalValue
    g_handoverMode ("handoverMode",
                    "HO euristic to be used, can be only \"NoAuto\", \"FixedTtt\", \"DynamicTtt\",   \"Threshold\"",
                    ns3::StringValue ("NoAuto"), ns3::MakeStringChecker ());

static ns3::GlobalValue g_e2TermIp ("e2TermIp", "The IP address of the RIC E2 termination",
                                    ns3::StringValue ("10.244.0.240"), ns3::MakeStringChecker ());

static ns3::GlobalValue
    g_enableE2FileLogging ("enableE2FileLogging",
              "If true, generate offline file logging instead of connecting to RIC",
              ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_controlFileName ("controlFileName", "The path to the control file (can be absolute)",
                                     ns3::StringValue ("es_actions_for_ns3.csv"), ns3::MakeStringChecker ());

static ns3::GlobalValue q_useSemaphores ("useSemaphores", "If true, enables the use of semaphores for external environment control",
                                        ns3::BooleanValue (false), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_digestControlMessages ("scheduleControlMessages",
                                                 "If true, read the whole control file at the beginning "
                                                 "of the simulation and schedules all the control actions events in advance",
                                                 ns3::BooleanValue (false),
                                                 ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_minSpeed ("minSpeed",
                                           "minimum UE speed in m/s",
                                           ns3::DoubleValue (2.0),
                                           ns3::MakeDoubleChecker<double> ());

static ns3::GlobalValue g_maxSpeed ("maxSpeed",
                                           "maximum UE speed in m/s",
                                           ns3::DoubleValue (4.0),
                                           ns3::MakeDoubleChecker<double> ());

static ns3::GlobalValue g_heuristic (
    "heuristicType",
    "Type of heuristic for managing BS status,"
    " No heuristic (-1),"
    " Random sleeping (0),"
    " Static sleeping (1),"
    " Dynamic sleeping (2)",
    ns3::IntegerValue (-1), ns3::MakeIntegerChecker<int8_t> ());
static ns3::GlobalValue g_probOn (
    "probOn",
    "Probability to turn BS ON for the random sleeping heuristic"
    "the value is proposed on the paper 'Small Cell Base Station Sleep"
    "Strategies for Energy Efficiency' in order to obtain an overall "
    "small average cell wake up time"
    "https://ieeexplore.ieee.org/abstract/document/7060678",
    ns3::DoubleValue (0.6038), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_probIdle (
    "probIdle",
    "Probability to turn BS Idle for the random sleeping heuristic"
    "the value is proposed on the paper 'Small Cell Base Station Sleep"
    "Strategies for 'Energy Efficiency' in order to obtain an overall" 
    "small average cell wake up time"
    "https://ieeexplore.ieee.org/abstract/document/7060678",
    ns3::DoubleValue (0.3854), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_probSleep (
    "probSleep",
    "Probability to turn BS Sleep for the random sleeping heuristic"
    "the value is proposed on the paper 'Small Cell Base Station Sleep"
    "Strategies for Energy Efficiency' in order to obtain an overall" 
    "small average cell wake up time"
    "https://ieeexplore.ieee.org/abstract/document/7060678",
    ns3::DoubleValue (0.0107), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_probOff (
    "probOff",
    "Probability to turn BS Off for the random sleeping heuristic"
    "the value is proposed on the paper 'Small Cell Base Station Sleep"
    "Strategies for Energy Efficiency' in order to obtain an overall" 
    "small average cell wake up time"
    "https://ieeexplore.ieee.org/abstract/document/7060678",
    ns3::DoubleValue (0.0), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_sinrTh (
    "sinrTh",
    "SINR threshold for static and dynamic sleeping heuristic",
    ns3::DoubleValue (73.0), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_bsOn (
    "bsOn",
    "number of BS to turn ON for static and dynamic sleeping heuristic",
    ns3::UintegerValue (2), ns3::MakeUintegerChecker<uint8_t> ());
static ns3::GlobalValue g_bsIdle (
    "bsIdle",
    "number of BS to turn IDLE for static and dynamic sleeping heuristic",
    ns3::UintegerValue (2), ns3::MakeUintegerChecker<uint8_t> ());
static ns3::GlobalValue g_bsSleep (
    "bsSleep",
    "number of BS to turn Sleep for static and dynamic sleeping heuristic",
    ns3::UintegerValue (2), ns3::MakeUintegerChecker<uint8_t> ());
static ns3::GlobalValue g_bsOff (
    "bsOff",
    "number of BS to turn Off for static and dynamic sleeping heuristic",
    ns3::UintegerValue (1), ns3::MakeUintegerChecker<uint8_t> ());

int
main (int argc, char *argv[])
{
  LogComponentEnableAll (LOG_PREFIX_ALL);
  LogComponentEnable ("ScenarioFive", LOG_LEVEL_DEBUG);
  // LogComponentEnable ("EnergyHeuristic", LOG_LEVEL_DEBUG);
  // LogComponentEnable ("PacketSink", LOG_LEVEL_ALL);
  // LogComponentEnable ("OnOffApplication", LOG_LEVEL_ALL);
  // LogComponentEnable ("LtePdcp", LOG_LEVEL_ALL);
  // LogComponentEnable ("LteRlcAm", LOG_LEVEL_ALL);
  // LogComponentEnable ("MmWaveUeMac", LOG_LEVEL_ALL);
  // LogComponentEnable ("MmWaveEnbMac", LOG_LEVEL_ALL);
  // LogComponentEnable ("LteUeMac", LOG_LEVEL_ALL);
  // LogComponentEnable ("LteEnbMac", LOG_LEVEL_ALL);
  // LogComponentEnable ("MmWaveFlexTtiMacScheduler", LOG_LEVEL_ALL);
  // LogComponentEnable ("LteEnbRrc", LOG_LEVEL_ALL);
  // LogComponentEnable ("LteUeRrc", LOG_LEVEL_ALL);
  // LogComponentEnable ("McEnbPdcp", LOG_LEVEL_ALL);
  // LogComponentEnable ("McUePdcp", LOG_LEVEL_ALL);
  // LogComponentEnable ("ScenarioOne", LOG_LEVEL_ALL);
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
  IntegerValue integerValue;
  BooleanValue booleanValue;
  StringValue stringValue;
  DoubleValue doubleValue;

  GlobalValue::GetValueByName ("dataRate", doubleValue);
  double dataRateFromConf = doubleValue.Get ();
  GlobalValue::GetValueByName ("rlcAmEnabled", booleanValue);
  bool rlcAmEnabled = booleanValue.Get ();
  GlobalValue::GetValueByName ("bufferSize", uintegerValue);
  uint32_t bufferSize = uintegerValue.Get ();
  GlobalValue::GetValueByName ("basicCellId", uintegerValue);
  uint16_t basicCellId = uintegerValue.Get ();
  GlobalValue::GetValueByName ("enableTraces", booleanValue);
  bool enableTraces = booleanValue.Get ();
  GlobalValue::GetValueByName ("outageThreshold",doubleValue);
  double outageThreshold = doubleValue.Get ();
  GlobalValue::GetValueByName ("handoverMode", stringValue);
  std::string handoverMode = stringValue.Get ();
  GlobalValue::GetValueByName ("e2TermIp", stringValue);
  std::string e2TermIp = stringValue.Get ();
  GlobalValue::GetValueByName ("enableE2FileLogging", booleanValue);
  bool enableE2FileLogging = booleanValue.Get ();
  GlobalValue::GetValueByName ("minSpeed", doubleValue);
  double minSpeed = doubleValue.Get ();
  GlobalValue::GetValueByName ("maxSpeed", doubleValue);
  double maxSpeed = doubleValue.Get ();
  GlobalValue::GetValueByName ("numberOfRaPreambles", uintegerValue);
  uint8_t numberOfRaPreambles = uintegerValue.Get ();
  // Heuristic parameters
  GlobalValue::GetValueByName ("heuristicType", integerValue);
  int8_t heuristicType = integerValue.Get ();
  GlobalValue::GetValueByName ("probOn", doubleValue);
  double probOn = doubleValue.Get ();
  GlobalValue::GetValueByName ("probIdle", doubleValue);
  double probIdle = doubleValue.Get ();
  GlobalValue::GetValueByName ("probSleep", doubleValue);
  double probSleep = doubleValue.Get ();
  GlobalValue::GetValueByName ("probOff", doubleValue);
  double probOff = doubleValue.Get ();
  GlobalValue::GetValueByName ("sinrTh", doubleValue);
  double sinrTh = doubleValue.Get ();
  GlobalValue::GetValueByName ("bsOn", uintegerValue);
  int bsOn = uintegerValue.Get ();
  GlobalValue::GetValueByName ("bsIdle", uintegerValue);
  int bsIdle = uintegerValue.Get ();
  GlobalValue::GetValueByName ("bsSleep", uintegerValue);
  int bsSleep = uintegerValue.Get ();
  GlobalValue::GetValueByName ("bsOff", uintegerValue);
  int bsOff = uintegerValue.Get ();

  NS_LOG_UNCOND ("rlcAmEnabled " << rlcAmEnabled << " bufferSize " << unsigned(bufferSize)
                                 << " OutageThreshold " << outageThreshold << " HandoverMode " << handoverMode
                                 << " BasicCellId " << unsigned(basicCellId) << " e2TermIp " << e2TermIp
                                 << " enableE2FileLogging " << enableE2FileLogging << " minSpeed "
                                 << minSpeed << " maxSpeed " << maxSpeed << " numberofRaPreambles " << unsigned(numberOfRaPreambles));

  GlobalValue::GetValueByName ("e2lteEnabled", booleanValue);
  bool e2lteEnabled = booleanValue.Get ();
  GlobalValue::GetValueByName ("e2nrEnabled", booleanValue);
  bool e2nrEnabled = booleanValue.Get ();
  GlobalValue::GetValueByName ("e2du", booleanValue);
  bool e2du = booleanValue.Get ();
  GlobalValue::GetValueByName ("e2cuUp", booleanValue);
  bool e2cuUp = booleanValue.Get ();
  GlobalValue::GetValueByName ("e2cuCp", booleanValue);
  bool e2cuCp = booleanValue.Get ();

  GlobalValue::GetValueByName ("reducedPmValues", booleanValue);
  bool reducedPmValues = booleanValue.Get ();
  GlobalValue::GetValueByName ("indicationPeriodicity", doubleValue);
  double indicationPeriodicity = doubleValue.Get ();

  GlobalValue::GetValueByName ("controlFileName", stringValue);
  std::string controlFilename = stringValue.Get ();

  GlobalValue::GetValueByName ("useSemaphores", booleanValue);
  bool useSemaphores = booleanValue.Get ();

  GlobalValue::GetValueByName ("scheduleControlMessages", booleanValue);
  bool scheduleControlMessages = booleanValue.Get ();

  NS_LOG_UNCOND("e2lteEnabled " << e2lteEnabled 
    << " e2nrEnabled " << e2nrEnabled
    << " e2du " << e2du
    << " e2cuCp " << e2cuCp
    << " e2cuUp " << e2cuUp
    << " reducedPmValues " << reducedPmValues
    << " controlFilename " << controlFilename
    << " useSemaphores " << useSemaphores
    << " indicationPeriodicity " << indicationPeriodicity
    << " ScheduleControlMessages " << scheduleControlMessages
    << " heuristicType " << int(heuristicType)
  );

  Config::SetDefault ("ns3::LteEnbNetDevice::UseSemaphores", BooleanValue (useSemaphores));
  Config::SetDefault ("ns3::LteEnbNetDevice::ControlFileName", StringValue(controlFilename));
  Config::SetDefault ("ns3::LteEnbNetDevice::E2Periodicity", DoubleValue (indicationPeriodicity));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::E2Periodicity", DoubleValue (indicationPeriodicity));

  Config::SetDefault ("ns3::MmWaveHelper::E2ModeLte", BooleanValue(e2lteEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::E2ModeNr", BooleanValue(e2nrEnabled));  
  Config::SetDefault ("ns3::MmWaveHelper::E2Periodicity", DoubleValue (indicationPeriodicity));
  
  // The DU PM reports should come from both NR gNB as well as LTE eNB, 
  // since in the RLC/MAC/PHY entities are present in BOTH NR gNB as well as LTE eNB.
  // TODO DU reports from LTE eNB are not implemented yet
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableDuReport", BooleanValue(e2du));

  // Config::SetDefault ("ns3::LteEnbNetDevice::ControlFileName", StringValue (controlFileName));

  // The CU-UP PM reports should only come from LTE eNB, since in the NS3 “EN-DC 
  // simulation (Option 3A)”, the PDCP is only in the LTE eNB and NOT in the NR gNB
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableCuUpReport", BooleanValue(e2cuUp));
  Config::SetDefault ("ns3::LteEnbNetDevice::EnableCuUpReport", BooleanValue(e2cuUp));

  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableCuCpReport", BooleanValue(e2cuCp));
  Config::SetDefault ("ns3::LteEnbNetDevice::EnableCuCpReport", BooleanValue(e2cuCp));
  
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::ReducedPmValues", BooleanValue (reducedPmValues));
  Config::SetDefault ("ns3::LteEnbNetDevice::ReducedPmValues", BooleanValue (reducedPmValues));

  Config::SetDefault ("ns3::LteEnbNetDevice::EnableE2FileLogging", BooleanValue (enableE2FileLogging));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableE2FileLogging", BooleanValue (enableE2FileLogging));

  Config::SetDefault ("ns3::LteEnbNetDevice::ScheduleControlMessages", BooleanValue (scheduleControlMessages));

  Config::SetDefault ("ns3::MmWaveEnbMac::NumberOfRaPreambles", UintegerValue (numberOfRaPreambles));
  Config::SetDefault ("ns3::LteEnbMac::NumberOfRaPreambles", UintegerValue (numberOfRaPreambles));

  Config::SetDefault ("ns3::MmWaveHelper::RlcAmEnabled", BooleanValue (rlcAmEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::UseIdealRrc", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveHelper::BasicCellId", UintegerValue (basicCellId));
  Config::SetDefault ("ns3::MmWaveHelper::BasicImsi", UintegerValue ((basicCellId-1)));
  Config::SetDefault ("ns3::MmWaveHelper::E2TermIp", StringValue (e2TermIp));

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

  Config::SetDefault ("ns3::LteEnbRrc::OutageThreshold", DoubleValue (outageThreshold));
  Config::SetDefault ("ns3::LteEnbRrc::SecondaryCellHandoverMode", StringValue (handoverMode));

  // Carrier bandwidth in Hz
  double bandwidth;
  // Center frequency in Hz
  double centerFrequency;
  // Distance between the mmWave BSs and the two co-located LTE and mmWave BSs in meters
  double isd = 850; // (interside distance)
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

    // 
    case 0:
      centerFrequency = 850e6;
      bandwidth = 20e6;
      numAntennasMcUe = 1;
      numAntennasMmWave = 1;
      dataRate = (dataRateFromConf == 0 ? "1.5Mbps" : "4.5Mbps");
      break;

    // FR-1 
    case 1:
      centerFrequency = 3.5e9;
      bandwidth = 20e6;
      numAntennasMcUe = 1;
      numAntennasMmWave = 1;
      dataRate = (dataRateFromConf == 0 ? "1.5Mbps" : "4.5Mbps");
      break;

    // FR-2
    case 2:
      centerFrequency = 28e9;
      bandwidth = 100e6;
      numAntennasMcUe = 16;
      numAntennasMmWave = 64;
      break;

    default:
      NS_FATAL_ERROR ("Configuration not recognized" << configuration);
      break;
    }

  NS_LOG_INFO ("Bandwidth " << bandwidth << " centerFrequency " << double (centerFrequency)
                            << " isd " << isd << " numAntennasMcUe " << numAntennasMcUe
                            << " numAntennasMmWave " << numAntennasMmWave << " dataRate "
                            << dataRate);

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

  uint8_t nMmWaveEnbNodes = 13;
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

  // This guarantees that each of the rest BSs is placed at the same distance from the two co-located in the center
  // 12 base stations where the closest are std = 850 
  
  //Smaller square
  for (int8_t i = 0; i < 4; ++i)
    {
      float x= pow(-1,floor(i/2))*isd/sqrt(2);
      float y= pow(-1,i)*isd/sqrt(2);
      enbPositionAlloc->Add (Vector (centerPosition.x + x, centerPosition.y + y, 3));
    }

  // Square rotated by 90 degrees
  for (int8_t i = 0; i < 4; ++i)
    {
      x = isd/sqrt(2)*2 * cos ((2 * M_PI * i) / (4));
      y = isd/sqrt(2)*2 * sin ((2 * M_PI * i) / (4));
      enbPositionAlloc->Add (Vector (centerPosition.x + x, centerPosition.y + y, 3));
    }

  // Bigger square
  for (int8_t i = 0; i < 4; ++i)
    {
      float x= pow(-1,floor(i/2))*isd*2/sqrt(2);
      float y= pow(-1,i)*isd*2/sqrt(2);
      enbPositionAlloc->Add (Vector (centerPosition.x + x, centerPosition.y + y, 3));
    }
    
  MobilityHelper enbmobility;
  enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  enbmobility.SetPositionAllocator (enbPositionAlloc);
  enbmobility.Install (allEnbNodes);
  // UE position allocator
  MobilityHelper uemobility;

 // Rectangle allocator with a uniform random variable
  Ptr<RandomRectanglePositionAllocator> uePositionAlloc = CreateObject<RandomRectanglePositionAllocator> ();
  Ptr<UniformRandomVariable> uePosAllX = CreateObject<UniformRandomVariable> ();
  uePosAllX->SetAttribute ("Min", DoubleValue (0));
  uePosAllX->SetAttribute ("Max", DoubleValue (maxXAxis));

  Ptr<UniformRandomVariable> uePosAllY = CreateObject<UniformRandomVariable> ();
  uePosAllY->SetAttribute ("Min", DoubleValue (0));
  uePosAllY->SetAttribute ("Max", DoubleValue (maxYAxis));

  uePositionAlloc->SetX (uePosAllX);
  uePositionAlloc->SetY (uePosAllY);
  uePositionAlloc->SetZ (centerPosition.z);

  Ptr<UniformRandomVariable> speed = CreateObject<UniformRandomVariable> ();
  speed->SetAttribute ("Min", DoubleValue (minSpeed));
  speed->SetAttribute ("Max", DoubleValue (maxSpeed));

  uemobility.SetMobilityModel ("ns3::RandomWalk2dOutdoorMobilityModel", "Speed",
                               PointerValue (speed), "Bounds",
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

  // Install and start applications
  // On the remoteHost there are TCP and UDP OnOff Applications
  uint16_t portTcp = 50000;
  Address sinkLocalAddressTcp (InetSocketAddress (Ipv4Address::GetAny (), portTcp));
  PacketSinkHelper sinkHelperTcp ("ns3::TcpSocketFactory", sinkLocalAddressTcp);
  AddressValue serverAddressTcp (InetSocketAddress (remoteHostAddr, portTcp));

  uint16_t portUdp = 60000;
  Address sinkLocalAddressUdp (InetSocketAddress (Ipv4Address::GetAny (), portUdp));
  PacketSinkHelper sinkHelperUdp ("ns3::UdpSocketFactory", sinkLocalAddressUdp);
  AddressValue serverAddressUdp (InetSocketAddress (remoteHostAddr, portUdp));

  ApplicationContainer sinkApp;
  sinkApp.Add (sinkHelperTcp.Install (remoteHost));
  sinkApp.Add (sinkHelperUdp.Install (remoteHost));

  // On the UEs there are TCP and UDP clients
  // If needed [Mean=1,Bound=0]
  OnOffHelper clientHelperTcp ("ns3::TcpSocketFactory", Address ());
  clientHelperTcp.SetAttribute ("Remote", serverAddressTcp);
  clientHelperTcp.SetAttribute ("OnTime", StringValue ("ns3::ExponentialRandomVariable"));
  clientHelperTcp.SetAttribute ("OffTime", StringValue ("ns3::ExponentialRandomVariable"));
  // Datarate defined in the switch
  clientHelperTcp.SetAttribute ("PacketSize", UintegerValue (1280));

  OnOffHelper clientHelperUdp ("ns3::UdpSocketFactory", Address ());
  clientHelperUdp.SetAttribute ("Remote", serverAddressUdp);
  clientHelperUdp.SetAttribute ("OnTime", StringValue ("ns3::ExponentialRandomVariable"));
  clientHelperUdp.SetAttribute ("OffTime", StringValue ("ns3::ExponentialRandomVariable"));
  // Datarate defined in the switch
  clientHelperUdp.SetAttribute ("PacketSize", UintegerValue (1280));

  switch (configuration)
    {
    // 
    case 0:
      clientHelperTcp.SetAttribute ("DataRate", StringValue (dataRate));
      clientHelperUdp.SetAttribute ("DataRate", StringValue (dataRate));
      break;

    // FR-1
    case 1:
      clientHelperTcp.SetAttribute ("DataRate", StringValue (dataRate));
      clientHelperUdp.SetAttribute ("DataRate", StringValue (dataRate));
      break;

    // FR-2
    case 2:
      clientHelperTcp.SetAttribute ("DataRate", StringValue ("45Mbps"));
      clientHelperUdp.SetAttribute ("DataRate", StringValue ("15Mbps"));
      break;

    default:
      NS_FATAL_ERROR ("Configuration not recognized" << configuration);
      break;
    }

  OnOffHelper clientHelperTcp150 ("ns3::TcpSocketFactory", Address ());
  clientHelperTcp150.SetAttribute ("Remote", serverAddressTcp);
  clientHelperTcp150.SetAttribute ("OnTime", StringValue ("ns3::ExponentialRandomVariable"));
  clientHelperTcp150.SetAttribute ("OffTime", StringValue ("ns3::ExponentialRandomVariable"));
  clientHelperTcp150.SetAttribute ("DataRate", StringValue ("150kbps"));
  clientHelperTcp150.SetAttribute ("PacketSize", UintegerValue (1280));

  OnOffHelper clientHelperTcp750 ("ns3::TcpSocketFactory", Address ());
  clientHelperTcp750.SetAttribute ("Remote", serverAddressTcp);
  clientHelperTcp750.SetAttribute ("OnTime", StringValue ("ns3::ExponentialRandomVariable"));
  clientHelperTcp750.SetAttribute ("OffTime", StringValue ("ns3::ExponentialRandomVariable"));
  clientHelperTcp750.SetAttribute ("DataRate", StringValue ("750kbps"));
  clientHelperTcp750.SetAttribute ("PacketSize", UintegerValue (1280));



  ApplicationContainer clientApp;
  // 25% Full-buffer traffic
  // 25% Bursty traffic with higher application bit-rate averaging around 3 Mbps
  // 25% Bursty traffic with higher application bit-rate averaging around 750 Kbps
  // 25% Bursty traffic with lower application bit-rate averaging around 150 Kbps.
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {

      if (u % 4 == 0)
        {
          // Full buffer traffic
          PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory",
                                               InetSocketAddress (Ipv4Address::GetAny (), 1234));
          sinkApp.Add (dlPacketSinkHelper.Install (ueNodes.Get (u)));
          UdpClientHelper dlClient (ueIpIface.GetAddress (u), 1234);
          dlClient.SetAttribute ("MaxPackets", UintegerValue (UINT32_MAX));
          dlClient.SetAttribute ("PacketSize", UintegerValue (1280));
          if (configuration == 2)
            {
              // Data rate 40 Mbps
              dlClient.SetAttribute ("Interval", TimeValue (MicroSeconds (250)));
            }
          else
            {
              // Data rate 20 Mbps
              dlClient.SetAttribute ("Interval", TimeValue (MicroSeconds (500)));
            }

          clientApp.Add (dlClient.Install (remoteHost));
        }
      else if (u % 4 == 1)
        {
          if (configuration == 2)
            clientHelperTcp.SetAttribute ("DataRate", StringValue ("20Mbps"));
          clientApp.Add (clientHelperTcp.Install (ueNodes.Get (u)));
        }
      else if (u % 4 == 2)
        {
          clientApp.Add (clientHelperTcp750.Install (ueNodes.Get (u)));
        }
      else if (u % 4 == 3)
        {
          clientApp.Add (clientHelperTcp150.Install (ueNodes.Get (u)));
        }
    }

  // Start applications
  GlobalValue::GetValueByName ("simTime", doubleValue);
  double simTime = doubleValue.Get ();
  sinkApp.Start (Seconds (0));

  clientApp.Start (MilliSeconds (100));
  clientApp.Stop (Seconds (simTime - 0.1));

  int BsStatus[4] = {bsOn, bsIdle, bsSleep, bsOff};
  // bsIdle turn ON the BS like would do bsOn
  // If bsIdle is equal to zero, treat bsOn as bsIdle and put bsOn=0, in this way we are skipping
  // the first part of heuristic 1 and 2 regarding SINR calculus and comparison: through this
  // changing we will give the possibility to OFF cells to turn ON
  if (bsIdle == 0)
  {
        BsStatus[1] = bsOn;
        BsStatus[0] = 0;
  }

  Ptr<EnergyHeuristic> energyHeur=CreateObject<EnergyHeuristic>();

  switch (heuristicType)
    {
      // No heuristc
      case -1: {
        NS_LOG_UNCOND ("Running the scenario with no Energy Heuristic");
      }
      break;

      // Random sleeping
      case 0: {
        for (double i = 0.0; i < simTime; i = i + indicationPeriodicity)
          {
            for (int j = 0; j < nMmWaveEnbNodes; j++)
              {
                Ptr<MmWaveEnbNetDevice> mmdev =
                    DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
                Ptr<LteEnbNetDevice> ltedev = DynamicCast<LteEnbNetDevice> (lteEnbDevs.Get (0));
                Simulator::Schedule (Seconds (i), &EnergyHeuristic::ProbabilityState, energyHeur,
                                     probOn, probIdle, probSleep, probOff, mmdev, ltedev);
              }
          }
      }
      break;

      // Static sleeping
      case 1: {
        for (double i = 0.0; i < simTime; i = i + indicationPeriodicity)
          {
            for (int j = 0; j < nMmWaveEnbNodes && bsOn!=0; j++)
              {
                Ptr<MmWaveEnbNetDevice> mmdev =
                    DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
                Simulator::Schedule (Seconds (i), &EnergyHeuristic::CountBestUesSinr, energyHeur,
                                     sinrTh, mmdev);
              }
            Ptr<LteEnbNetDevice> ltedev = DynamicCast<LteEnbNetDevice> (lteEnbDevs.Get (0));
            Simulator::Schedule (Seconds (i), &EnergyHeuristic::TurnOnBsSinrPos, energyHeur,
                                 nMmWaveEnbNodes, mmWaveEnbDevs, "static", BsStatus, ltedev);
          }
      }
      break;

      // Dynamic sleeping
      case 2: {
        for (double i = 0.0; i < simTime; i = i + indicationPeriodicity)
          {
            for (int j = 0; j < nMmWaveEnbNodes && bsOn!=0; j++)
              {
                Ptr<MmWaveEnbNetDevice> mmdev =
                    DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
                Simulator::Schedule (Seconds (i), &EnergyHeuristic::CountBestUesSinr, energyHeur,
                                     sinrTh, mmdev);
              }
            Ptr<LteEnbNetDevice> ltedev = DynamicCast<LteEnbNetDevice> (lteEnbDevs.Get (0));
            Simulator::Schedule (Seconds (i), &EnergyHeuristic::TurnOnBsSinrPos, energyHeur,
                                 nMmWaveEnbNodes, mmWaveEnbDevs, "dynamic", BsStatus, ltedev);
          }
      }
      break;

      default: {
        NS_FATAL_ERROR (
            "Heuristic type not recognized, the only possible values are [-1,0,1,2,3]. Value passed: "
            << heuristicType);
      }
      break;
    }

  if (enableTraces)
  {
    mmwaveHelper->EnableTraces ();
  }  

  // trick to enable PHY traces for the LTE stack
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  lteHelper->Initialize ();
  lteHelper->EnablePhyTraces ();
  lteHelper->EnableMacTraces ();

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

  NS_LOG_INFO (lteHelper);

  Simulator::Destroy ();
  NS_LOG_INFO ("Done.");
  return 0;
}