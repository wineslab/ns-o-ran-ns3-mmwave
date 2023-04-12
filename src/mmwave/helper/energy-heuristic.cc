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
#include "ns3/applications-module.h"
#include "ns3/lte-ue-net-device.h"
#include "ns3/mmwave-helper.h"
#include "energy-heuristic.h"
#include <vector>
#include <iostream>
#include <sstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("EnergyHeuristic");

namespace mmwave {

//NS_OBJECT_ENSURE_REGISTERED (EnergyHeuristic);

EnergyHeuristic::EnergyHeuristic ()
{
  NS_LOG_FUNCTION (this);
}


EnergyHeuristic::~EnergyHeuristic ()
{
  NS_LOG_FUNCTION (this);
  if (m_energyHeuristicFile.is_open ())
    m_energyHeuristicFile.close();
}

void
EnergyHeuristic::EnergyHeuristicTrace (Ptr<MmWaveEnbNetDevice> mmDev)
{
  NS_LOG_LOGIC ("EnergyHeuristicSizeTrace " << Simulator::Now ().GetSeconds () << " "
                                            << mmDev->GetCellId () << " " << mmDev->GetBsState ());
  // write to file
  if (!m_energyHeuristicFile.is_open ())
    {
      NS_LOG_DEBUG (GetEnergyHeuristicFilename ().c_str ());
      m_energyHeuristicFile.open (GetEnergyHeuristicFilename ().c_str (),
                                  std::ofstream::out | std::ofstream::trunc);
      NS_LOG_LOGIC ("File opened");
      m_energyHeuristicFile << "Timestamp"
                            << " "
                            << "UNIX"
                            << " "
                            << "Id"
                            << " "
                            << "State" << std::endl;
    }
  uint64_t timestamp = mmDev->GetStartTime () + Simulator::Now ().GetMilliSeconds ();
  m_energyHeuristicFile << Simulator::Now ().GetSeconds () << " " << timestamp << " "
                        << mmDev->GetCellId () << " " << mmDev->GetBsState () << std::endl;
}

std::string EnergyHeuristic::GetEnergyHeuristicFilename()
{
  return m_energyHeuristicFilename;
}

void EnergyHeuristic::SetEnergyHeuristicFilename(std::string filename)
{
  m_energyHeuristicFilename = filename;
}

TypeId EnergyHeuristic::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::EnergyHeuristic")
    .SetParent<Object>()
    .AddConstructor<EnergyHeuristic>()
    .AddAttribute ("EnergyHeuristicFilename",
                "Name of the file where the energy heuristic information will be periodically written.",
                StringValue ("EnergyHeuristic.txt"),
                MakeStringAccessor (&EnergyHeuristic::SetEnergyHeuristicFilename),
                MakeStringChecker ())
    ;
  return tid;
}

void EnergyHeuristic::CountBestUesSinr(double sinrTh, Ptr<MmWaveEnbNetDevice> mmDev){
  Ptr<LteEnbRrc> lte_rrc= mmDev->GetRrc();

  // Get connected UEs from BS
  std::map<uint16_t, Ptr<UeManager>> ue_attached = lte_rrc->GetUeMap (); // list of attached UEs
  NS_LOG_DEBUG ("N ues attached: " << ue_attached.size ());
  // yes, UEs are attached during simulation
  // std::map<uint64_t, std::map<uint16_t, long double>> -- <imsi, <cellid, sinr>
  std::map<uint64_t, std::map<uint16_t, long double>> m_l3sinrMap = mmDev->Getl3sinrMap ();
  uint16_t goodUesCount = 0;

  for (auto it = ue_attached.cbegin (); it != ue_attached.cend (); ++it)
    {
      double sinrThisCell =
          10 * std::log10 (m_l3sinrMap[it->second->GetImsi ()][mmDev->GetCellId ()]);
      double convertedSinr = L3RrcMeasurements::ThreeGppMapSinr (sinrThisCell);
      NS_LOG_DEBUG ("sinrThisCell: " << convertedSinr);
      if (convertedSinr > sinrTh) // over 13 is a good SINR range = over 73 convertedSinr
        {
          goodUesCount++;
        }
    }
   mmDev->SetNUeGoodSinr (goodUesCount); // Number of UEs in the cell with a good SINR value

   NS_LOG_DEBUG ("NUeGoodSinr for BS "
                 << mmDev->GetCellId ()
                 << " is: " << mmDev->GetNUeGoodSinr ());
}

void
EnergyHeuristic::ProbabilityState (double p1, double p2, double p3, double p4,
                                   Ptr<MmWaveEnbNetDevice> mmDev, Ptr<LteEnbNetDevice> ltedev)
{
  uint16_t nodeId = mmDev->GetCellId ();
  Ptr<LteEnbRrc> lte_rrc = ltedev->GetRrc ();
  Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
  double r = x->GetValue();

  NS_LOG_DEBUG ("Sim time " << Simulator::Now ().GetSeconds () << " BS Id " << nodeId << 
                " Prob " << r);

  MmWaveEnbNetDevice::enumModeEnergyBs state;

  if (r <= p1)
    {
      state = mmDev->enumModeEnergyBs::ON;
    }
  else if (r > p1 && r <= (p1 + p2))
    {
      state = mmDev->enumModeEnergyBs::Idle;
    }
  else if (r > (p1 + p2) && r <= (p1 + p2 + p3))
    {
      state = mmDev->enumModeEnergyBs::Sleep;
    }
  else // case (r > (p1 + p2 + p3) && r <= 1)
    {
      state = mmDev->enumModeEnergyBs::OFF;
    }

  NS_LOG_DEBUG ("BS state " << state);
  lte_rrc->SetSecondaryCellHandoverAllowedStatus (nodeId, state);
  mmDev->SetCellState (state);
  EnergyHeuristicTrace (mmDev);
}

std::vector<std::pair<Ptr<MmWaveEnbNetDevice>, double>>
EnergyHeuristic::HeuristicDynamic (int bsToTurnOn[], int bsOn, int bsIdle, int bsSleep, int bsOff,
                                   int nMmWaveEnbNodes, NetDeviceContainer mmWaveEnbDevs)
{
  // For others BS get position of closest UE
  for (int j = 0; j < nMmWaveEnbNodes; j++)
    {
      // If BS is already turned ON (from previous piece of code) skip this BS
      bool found = false;
      for (int i = 0; i < bsOn; i++)
        {
          if (bsToTurnOn[i] == j)
            {
              found = true;
              break;
            }
        }

      if (found)
        {
          continue;
        }
      //otherwise
      Ptr<MmWaveEnbNetDevice> mmDev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
      Ptr<LteEnbRrc> lte_rrc = mmDev->GetRrc ();
      NS_LOG_DEBUG ("UEs time distances from cell " << lte_rrc->GetCellId ());

      // Get an iterator on the list of all the nodes deployed in the scenario
      for (NodeList::Iterator it = NodeList::Begin (); it != NodeList::End (); ++it) 
        {
          Ptr<Node> node = *it;
          int nDevs = node->GetNDevices ();
          for (int jDevs = 0; jDevs < nDevs; jDevs++)
            {
              Ptr<McUeNetDevice> mcuedev =
                  node->GetDevice (jDevs)->GetObject<McUeNetDevice> (); //get UE device
              if (mcuedev)
                {
                  uint64_t nodeIMSI = mcuedev->GetImsi ();
                  // Save the UE closest position of this BS (index j)
                  double relativeSpeed =
                      node->GetObject<MobilityModel> ()->GetRelativeSpeed (
                          mmDev->GetNode ()->GetObject<MobilityModel> ());
                  // Get scenario ue pos
                  Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
                  // Get the position of the GnB
                  Vector posGnB =
                      mmDev->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();
                  // GetDistance between BS and UE
                  double ueDist =
                      sqrt (pow (pos.x - posGnB.x, 2) +
                            pow (pos.y - posGnB.y,
                                  2)); // distance between UE (nodeIMSI) and BS
                  double time = ueDist / relativeSpeed;
                  double ClosestUeTime = mmDev->GetClosestUeTime (); // get the attribute where is saved the closest attached UE position
                  // If the ue in the scenario (nodeIMSI) is closer compared to the one already saved, save it
                  if (time < ClosestUeTime)
                    {
                      mmDev->SetClosestUeTime (time);
                    }
                  NS_LOG_DEBUG ("BS: " << lte_rrc->GetCellId () << " UE:" << nodeIMSI
                                        << " distance:" << ueDist << " pos: " << pos << " relativeSpeed: "
                                        << relativeSpeed << " time: " << time);
                }
            }
        }
      NS_LOG_DEBUG ("The closest for BS: " << lte_rrc->GetCellId ()
                                    << " is time: " << mmDev->GetClosestUeTime());
    }

  // Save all closest UEs of all the BS in the scenario into a Map
  std::vector<std::pair<Ptr<MmWaveEnbNetDevice>, double>> UeTime; //cellID, UEdistance
  for (int j = 0; j < nMmWaveEnbNodes; j++)
    {
      //if BS is already turned ON skip this BS
      bool found = false;
      for (int i = 0; i < bsOn; i++)
        {
          if (j == bsToTurnOn[i])
            {
              found = true;
              break;
            }
        }
      if (found)
        {
          continue;
        }
      Ptr<MmWaveEnbNetDevice> mmDev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
      UeTime.push_back (
          std::pair<Ptr<MmWaveEnbNetDevice>, double> (mmDev, mmDev->GetClosestUeTime ()));
      // Reset attribute position of closest UE
      mmDev->SetClosestUeTime (10000.0);
    }

  // Order the vector or pairs
  std::sort (UeTime.begin (), UeTime.end (),
             [] (const std::pair<Ptr<MmWaveEnbNetDevice>, double> &left,
                 const std::pair<Ptr<MmWaveEnbNetDevice>, double> &right) {
               return left.second < right.second;
             });

  NS_LOG_DEBUG ("Print the map of BS-time ordered");
  for (std::pair<Ptr<MmWaveEnbNetDevice>, double> i : UeTime)
    {
      NS_LOG_DEBUG ("BS " << i.first->GetCellId () << " with time " << i.second);
    }

  return UeTime;
}

std::vector<std::pair<Ptr<MmWaveEnbNetDevice>, double>>
EnergyHeuristic::HeuristicStatic (int bsToTurnOn[], int bsOn, int bsIdle, int bsSleep, int bsOff,
                                  int nMmWaveEnbNodes, NetDeviceContainer mmWaveEnbDevs)
{
  //for others BS get position of closest UE
  for (int j = 0; j < nMmWaveEnbNodes; j++)
    {
      //if BS is already turned ON (from previous piece of code) skip this BS
      bool found = false;
      for (int i = 0; i < bsOn; i++)
        {
          if (bsToTurnOn[i] == j)
            {
              found = true;
              break;
            }
        }
      if (found)
        {
          continue;
        }
      //otherwise
      Ptr<MmWaveEnbNetDevice> mmDev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
      Ptr<LteEnbRrc> lte_rrc = mmDev->GetRrc ();
      NS_LOG_DEBUG ("UEs distances from cell " << lte_rrc->GetCellId ());
      for (NodeList::Iterator it = NodeList::Begin (); it != NodeList::End ();
           ++it) //get an iterator on the list of all the nodes deployed in the scenario
        {
          Ptr<Node> node = *it;
          int nDevs = node->GetNDevices ();
          for (int jDevs = 0; jDevs < nDevs; jDevs++)
            {
              Ptr<McUeNetDevice> mcuedev =
                  node->GetDevice (jDevs)->GetObject<McUeNetDevice> (); //get UE device
              if (mcuedev)
                {
                  uint64_t nodeIMSI = mcuedev->GetImsi ();
                  //get scenario ue pos
                  Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
                  //get the position of the GnB
                  Vector posGnB =
                      mmDev->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();
                  //getDistance between BS and UE
                  double ueDist =
                      sqrt (pow (pos.x - posGnB.x, 2) +
                            pow (pos.y - posGnB.y,
                                  2)); //distance between UE (nodeIMSI) and BS
                  std::pair<double, double> ClosestUepos = mmDev->GetClosestUePos (); // get the attribute where is saved the closest attached UE position
                  double ClosestUeposDist =
                      sqrt (pow (ClosestUepos.first - posGnB.x, 2) +
                            pow (ClosestUepos.second - posGnB.y,
                                  2)); // actual distance (BS-UE) of the closest saved UE
                  //if the ue in the scenario (nodeIMSI) is closer compared to the one already saved, save it
                  if (ueDist < ClosestUeposDist)
                    {
                      std::pair<double, double> uePos = {pos.x, pos.y};
                      mmDev->SetClosestUePos (uePos);
                    }
                  NS_LOG_DEBUG ("BS: " << lte_rrc->GetCellId () << " UE:" << nodeIMSI
                                        << " distance:" << ueDist << " pos: " << pos);
                }
            }
        }
      NS_LOG_DEBUG ("The closest for BS: " << lte_rrc->GetCellId ()
                                           << " is pos: " << mmDev->GetClosestUePos ());
    }
  //save all closest UEs of all the BS in the scenario into a Map
  std::vector<std::pair<Ptr<MmWaveEnbNetDevice>, double>> UeDistances; //cellID, UEdistance
  for (int j = 0; j < nMmWaveEnbNodes; j++)
    {
      //if BS is already turned ON skip this BS
      bool found = false;
      for (int i = 0; i < bsOn; i++)
        {
          if (j == bsToTurnOn[i])
            {
              found = true;
              break;
            }
        }
      if (found)
        {
          continue;
        }
        
      Ptr<MmWaveEnbNetDevice> mmDev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
      std::pair<double, double> ClosestUepos = mmDev->GetClosestUePos (); // get the saved UE pos
      Vector posGnB = mmDev->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();
      double ClosestUeposDist =
          sqrt (pow (ClosestUepos.first - posGnB.x, 2) + pow (ClosestUepos.second - posGnB.y, 2));
      UeDistances.push_back (std::pair<Ptr<MmWaveEnbNetDevice>, double> (mmDev, ClosestUeposDist));
      //reset attribute position of closest UE
      std::pair<double, double> uePos = {10000.0, 10000.0};
      mmDev->SetClosestUePos (uePos);
    }

  //order the vector or pairs
  std::sort (UeDistances.begin (), UeDistances.end (),
             [] (const std::pair<Ptr<MmWaveEnbNetDevice>, double> &left,
                 const std::pair<Ptr<MmWaveEnbNetDevice>, double> &right) {
               return left.second < right.second;
             });

  NS_LOG_DEBUG ("Print the map of BS-distances ordered");
  for (std::pair<Ptr<MmWaveEnbNetDevice>, double> i : UeDistances)
    NS_LOG_DEBUG ("BS " << i.first->GetCellId () << " with distance " << i.second);

  return UeDistances;
}

void
EnergyHeuristic::TurnOnBsSinrPos (uint8_t nMmWaveEnbNodes, NetDeviceContainer mmWaveEnbDevs,
                                  std::string heuristic, int BsStatus[],
                                  Ptr<LteEnbNetDevice> ltedev)
{
  int bsOn = BsStatus[0];
  int bsIdle = BsStatus[1];
  int bsSleep = BsStatus[2];
  int bsOff = BsStatus[3];
  Ptr<LteEnbRrc> lte_rrc = ltedev->GetRrc ();
  if (bsOn + bsIdle + bsSleep + bsOff != nMmWaveEnbNodes)
    {
      NS_FATAL_ERROR ("ERROR: number of BS we interact with( "
                      << bsOn + bsIdle + bsSleep + bsOff
                      << ") is different to the total number of BS in the scenario ("
                      << nMmWaveEnbNodes << ")");
    }
  std::vector<int> BShighestValues (bsOn, -1);
  int bsToTurnOn[bsOn];

  for (int j = 0; j < nMmWaveEnbNodes; j++)
    {
      // Get the mmwave BS
      Ptr<MmWaveEnbNetDevice> mmDev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));

      for (int i = 0; i < bsOn; i++)
        {
          // Save the bests (number equal to bsOn) BSs with highest NUeGoodSINR
          if (BShighestValues[i] < mmDev->GetNUeGoodSinr ())
            {
              BShighestValues[i] = mmDev->GetNUeGoodSinr (); // Save N UEs value
              bsToTurnOn[i] = j; // Save BS index
              break; // If this value is the highest exit the loop and check next BS
            }
        }
    }

  // Turn on first N BSs
  for (int j = 0; j < bsOn; j++)
    {
      Ptr<MmWaveEnbNetDevice> mmDev =
          DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (bsToTurnOn[j]));
      NS_LOG_DEBUG ("BS to turn on ID: " << mmDev->GetCellId ());
      mmDev->TurnOn (mmDev->GetCellId (), lte_rrc);
      EnergyHeuristicTrace(mmDev);
    }

  std::vector<std::pair<Ptr<MmWaveEnbNetDevice>, double>> UEVectorPairs;
  if (heuristic == "static")
    {
      UEVectorPairs = HeuristicStatic (bsToTurnOn, bsOn, bsIdle, bsSleep, bsOff, nMmWaveEnbNodes,
                                       mmWaveEnbDevs);
    }
  else if (heuristic == "dynamic")
    {
      UEVectorPairs = HeuristicDynamic (bsToTurnOn, bsOn, bsIdle, bsSleep, bsOff, nMmWaveEnbNodes,
                                        mmWaveEnbDevs);
    }

  // Turn idle first N BSs
  for (int j = 0; j < bsIdle; j++)
    {
      Ptr<MmWaveEnbNetDevice> mmDev = UEVectorPairs[j].first;
      NS_LOG_DEBUG ("BS to turn Idle ID: " << UEVectorPairs[j].first->GetCellId ());
      mmDev->TurnIdle (mmDev->GetCellId (), lte_rrc);
      EnergyHeuristicTrace(mmDev);
    }

  // Turn sleep first N BSs
  for (int j = bsIdle; j < bsIdle + bsSleep; j++)
    {
      Ptr<MmWaveEnbNetDevice> mmDev = UEVectorPairs[j].first;
      NS_LOG_DEBUG ("BS to turn sleep ID: " << UEVectorPairs[j].first->GetCellId ());
      mmDev->TurnSleep (mmDev->GetCellId (), lte_rrc);
      EnergyHeuristicTrace(mmDev);
    }

  // Turn off first N BSs
  for (int j = bsIdle + bsSleep; j < bsIdle + bsSleep + bsOff; j++)
    {
      Ptr<MmWaveEnbNetDevice> mmDev = UEVectorPairs[j].first;
      NS_LOG_DEBUG ("BS to turn off ID: " << UEVectorPairs[j].first->GetCellId ());
      mmDev->TurnOff (mmDev->GetCellId (), lte_rrc);
      EnergyHeuristicTrace(mmDev);
    }
}

}

} // namespace ns3