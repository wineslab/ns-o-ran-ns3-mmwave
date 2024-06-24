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
#include <bitset>

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
}

TypeId EnergyHeuristic::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::EnergyHeuristic")
    .SetParent<Object>()
    .AddConstructor<EnergyHeuristic>();
  return tid;
}

void EnergyHeuristic::CountBestUesSinr(double sinrTh, Ptr<MmWaveEnbNetDevice> mmDev){
  Ptr<LteEnbRrc> lte_rrc= mmDev->GetRrc();

  // Get connected UEs from BS
  std::map<uint16_t, Ptr<UeManager>> ue_attached = lte_rrc->GetUeMap (); // List of attached UEs
  NS_LOG_DEBUG ("N ues attached: " << ue_attached.size ());
  // UEs are attached when simulation starts
  // std::map<uint64_t, std::map<uint16_t, long double>> -- <imsi, <cellid, sinr>
  std::map<uint64_t, std::map<uint16_t, long double>> m_l3sinrMap = mmDev->Getl3sinrMap ();
  uint16_t goodUesCount = 0;

  for (auto it = ue_attached.cbegin (); it != ue_attached.cend (); ++it)
    {
      double sinrThisCell =
          10 * std::log10 (m_l3sinrMap[it->second->GetImsi ()][mmDev->GetCellId ()]);
      double convertedSinr = L3RrcMeasurements::ThreeGppMapSinr (sinrThisCell);
      NS_LOG_DEBUG ("sinrThisCell: " << convertedSinr);
      if (convertedSinr > sinrTh) // Over 13 is a good SINR range = over 73 convertedSinr
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
EnergyHeuristic::RandomAction(std::vector<Ptr<MmWaveEnbNetDevice>> mmdevArray, Ptr<LteEnbNetDevice> ltedev)
{
    const int filter_list[] = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13,  14,  16,
                               17, 18, 19, 20, 21, 22, 24, 25, 26, 28, 32, 33, 34, 35,  36,  37,
                               38, 40, 41, 42, 44, 48, 49, 50, 52, 56, 64, 65, 66, 67,  68,  69,
                               70, 72, 73, 74, 76, 80, 81, 82, 84, 88, 96, 97, 98, 100, 104, 112};
    // Get the random value
    Ptr<UniformRandomVariable>
        x = CreateObject<UniformRandomVariable>();
    int min = 0;
    int max = sizeof(filter_list) / sizeof(filter_list[0]);
    x->SetAttribute("Min", DoubleValue(min));
    x->SetAttribute("Max", DoubleValue(max - 1));
    int r = x->GetInteger();
    NS_LOG_DEBUG("Random number: " << r );
    // Identify at which action is it referring to
    int action = filter_list[r];
    // Convert the integer to a 7-bit binary representation
    std::string actionBinary = std::bitset<7>(action).to_string(); // to binary
    Ptr<LteEnbRrc> lte_rrc = ltedev->GetRrc();
    // Iterate over the string
    // Using a range-based for loop
    int iterator = 0;
    NS_LOG_DEBUG("Action: " << action << " Action binary: " << actionBinary);
    for (char currentChar : actionBinary)
    {
        Ptr<MmWaveEnbNetDevice> mmDev = mmdevArray[iterator];
        uint16_t nodeId = mmDev->GetCellId();
        NS_LOG_DEBUG("Sim time " << Simulator::Now().GetSeconds() << " BS Id " << nodeId << " Action "
                          << action << " Binary " << actionBinary);
        // If it is a 1 turn oFF (1 = energy saving state)
        if (currentChar == '1')
        {
            mmDev->TurnOff (nodeId, lte_rrc);
        }
        // Else turn ON
        else
        {
            mmDev->TurnOn (nodeId, lte_rrc);
        }
        NS_LOG_DEBUG("BS state " << mmDev->GetBsState());
        iterator = iterator + 1;
    }
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
      // Otherwise
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
                  node->GetDevice (jDevs)->GetObject<McUeNetDevice> (); //Get UE device
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
                                  2)); // Distance between UE (nodeIMSI) and BS
                  double time = ueDist / relativeSpeed;
                  double ClosestUeTime = mmDev->GetClosestUeTime (); // Get the attribute where is saved the closest attached UE position
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
      // If BS is already turned ON skip this BS
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
      // Otherwise
      Ptr<MmWaveEnbNetDevice> mmDev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
      Ptr<LteEnbRrc> lte_rrc = mmDev->GetRrc ();
      NS_LOG_DEBUG ("UEs distances from cell " << lte_rrc->GetCellId ());
      for (NodeList::Iterator it = NodeList::Begin (); it != NodeList::End ();
           ++it) // Get an iterator on the list of all the nodes deployed in the scenario
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
                  // Get scenario UE position
                  Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
                  // Get the position of the GnB
                  Vector posGnB =
                      mmDev->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();
                  // Get distance between BS and UE
                  double ueDist =
                      sqrt (pow (pos.x - posGnB.x, 2) +
                            pow (pos.y - posGnB.y,
                                  2)); // Distance between UE (nodeIMSI) and BS
                  std::pair<double, double> ClosestUepos = mmDev->GetClosestUePos (); // Get the attribute where is saved the closest attached UE position
                  double ClosestUeposDist =
                      sqrt (pow (ClosestUepos.first - posGnB.x, 2) +
                            pow (ClosestUepos.second - posGnB.y,
                                  2)); // Actual distance (BS-UE) of the closest saved UE
                  // If the ue in the scenario (nodeIMSI) is closer compared to the one already saved, save it
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
  // Save all closest UEs of all the BS in the scenario into a Map
  std::vector<std::pair<Ptr<MmWaveEnbNetDevice>, double>> UeDistances; // cellID, UEdistance
  for (int j = 0; j < nMmWaveEnbNodes; j++)
    {
      // If BS is already turned ON skip this BS
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
      std::pair<double, double> ClosestUepos = mmDev->GetClosestUePos (); // Get the saved UE pos
      Vector posGnB = mmDev->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();
      double ClosestUeposDist =
          sqrt (pow (ClosestUepos.first - posGnB.x, 2) + pow (ClosestUepos.second - posGnB.y, 2));
      UeDistances.push_back (std::pair<Ptr<MmWaveEnbNetDevice>, double> (mmDev, ClosestUeposDist));
      // Reset attribute position of closest UE
      std::pair<double, double> uePos = {10000.0, 10000.0};
      mmDev->SetClosestUePos (uePos);
    }

  // Order the vector of pairs from the smallest to the biggest
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
  int bsToTurnOn[bsOn];
  // If bsOn=0 just skip the part used to decide which cells to turn ON
  if (bsOn!=0){

    // Find the lowest value in bsHighestValues and save the index of the BS with the related highest value
    std::vector<int> bsHighestValues (bsOn, -1);
    for (int j = 0; j < nMmWaveEnbNodes; j++)
      {
        // Get the mmwave BS
        Ptr<MmWaveEnbNetDevice> mmDev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));

        // Find the lowest value in the vector bsHighestValues and maybe substitute
        int lowest = bsHighestValues[0];
        int lowestIndex = 0;
        for (int i = 1; i < bsOn; i++) {
          if (bsHighestValues[i] < lowest) {
            lowest = bsHighestValues[i];
            lowestIndex = i;
          }
        }
        // Save the actual NUeGoodSINR in the vector if it is better than the lowest available
        if (lowest < mmDev->GetNUeGoodSinr ())
          {
            bsHighestValues[lowestIndex] = mmDev->GetNUeGoodSinr (); // Save N UEs value
            bsToTurnOn[lowestIndex] = j; // Save BS index
          }
      }

    // Turn on first N BSs
    for (int j = 0; j < bsOn; j++)
      {
        Ptr<MmWaveEnbNetDevice> mmDev =
            DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (bsToTurnOn[j]));
        NS_LOG_DEBUG ("BS to turn on ID: " << mmDev->GetCellId ());
        mmDev->TurnOn (mmDev->GetCellId (), lte_rrc);
      }
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
    }

  // Turn sleep first N BSs
  for (int j = bsIdle; j < bsIdle + bsSleep; j++)
    {
      Ptr<MmWaveEnbNetDevice> mmDev = UEVectorPairs[j].first;
      NS_LOG_DEBUG ("BS to turn sleep ID: " << UEVectorPairs[j].first->GetCellId ());
      mmDev->TurnSleep (mmDev->GetCellId (), lte_rrc);
    }

  // Turn off first N BSs
  for (int j = bsIdle + bsSleep; j < bsIdle + bsSleep + bsOff; j++)
    {
      Ptr<MmWaveEnbNetDevice> mmDev = UEVectorPairs[j].first;
      NS_LOG_DEBUG ("BS to turn off ID: " << UEVectorPairs[j].first->GetCellId ());
      mmDev->TurnOff (mmDev->GetCellId (), lte_rrc);
    }
}

} // namespace mmwave

} // namespace ns3