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
  m_enHeuristicFile.close();
}

TypeId EnergyHeuristic::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::EnergyHeuristic")
    .SetParent<Object>()
    .AddConstructor<EnergyHeuristic>();
  return tid;
}

void EnergyHeuristic::CountBestUesSinr(double sinrTh, Ptr<MmWaveEnbNetDevice> mmDev){
  //reset parameter
  mmDev->SetNUeGoodSinr(0);
  Ptr<LteEnbRrc> m_rrc= mmDev->GetRrc();
  //get connected UEs from BS
  std::map<uint16_t, Ptr<UeManager>>ue_attached= m_rrc->GetUeMap(); //list of attached UEs
  NS_LOG_DEBUG ("N ues attached: "<<ue_attached.size());
  for(auto it = ue_attached.cbegin(); it != ue_attached.cend(); ++it)
    {
      //yes, UEs are attached during simulation
      //std::map<uint64_t, std::map<uint16_t, long double>> -- <imsi, <cellid, sinr>
    std::map<uint64_t, std::map<uint16_t, long double>> m_l3sinrMap=mmDev->Getl3sinrMap();
    double sinrThisCell = 10 * std::log10(m_l3sinrMap[it->second->GetImsi()][mmDev->GetCellId()]);
      double convertedSinr = L3RrcMeasurements::ThreeGppMapSinr (sinrThisCell);
    NS_LOG_DEBUG ( "sinrThisCell: "<< convertedSinr);
    if (convertedSinr > sinrTh)//over 13 is a good SINR range = over 73 convertedSinr
        {
      uint16_t NUeGoodSinr =  mmDev->GetNUeGoodSinr();
          NUeGoodSinr++;
      mmDev->SetNUeGoodSinr(NUeGoodSinr);
        }
    }
  NS_LOG_DEBUG ( "NUeGoodSinr for BS "<< mmDev->GetCellId()<<" is: "<< mmDev->GetNUeGoodSinr()); //number of UEs with a good SINR value
}

void
EnergyHeuristic::ProbabilityState (double p1, double p2, double p3, double p4,
                                   Ptr<MmWaveEnbNetDevice> mmDev, Ptr<LteEnbNetDevice> ltedev)
{
  uint16_t nodeId = mmDev->GetCellId ();
  NS_LOG_DEBUG ("Sim time " << Simulator::Now ().GetSeconds ());
  NS_LOG_DEBUG ("Enb name " << nodeId);
  Ptr<LteEnbRrc> m_rrc = ltedev->GetRrc ();
  Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
  double r = x->GetValue();
  NS_LOG_DEBUG ("Prob " << r);
  if (r <= p1)
    {
      NS_LOG_DEBUG ("BS state " << mmDev->enumModeEnergyBs::ON);
      m_rrc->SetSecondaryCellHandoverAllowedStatus (nodeId, mmDev->enumModeEnergyBs::ON);
      mmDev->SetCellState(mmDev->enumModeEnergyBs::ON);
      EnHeuristicTrace(mmDev);
    }
  else if (r > p1 && r <= (p1 + p2))
    {
      NS_LOG_DEBUG ("BS state " << mmDev->enumModeEnergyBs::Idle);
      m_rrc->SetSecondaryCellHandoverAllowedStatus (nodeId, mmDev->enumModeEnergyBs::Idle);
      mmDev->SetCellState(mmDev->enumModeEnergyBs::Idle);
      EnHeuristicTrace(mmDev);
    }
  else if (r > (p1 + p2) && r <= (p1 + p2 + p3))
    {
      NS_LOG_DEBUG ("BS state " << mmDev->enumModeEnergyBs::Sleep);
      m_rrc->SetSecondaryCellHandoverAllowedStatus (nodeId, mmDev->enumModeEnergyBs::Sleep);
      m_rrc->EvictUsersFromSecondaryCell ();
      mmDev->SetCellState(mmDev->enumModeEnergyBs::Sleep);
      EnHeuristicTrace(mmDev);
    }
  else if (r > (p1 + p2 + p3) && r <= 1)
    {
      NS_LOG_DEBUG ("BS state " << mmDev->enumModeEnergyBs::OFF);
      m_rrc->SetSecondaryCellHandoverAllowedStatus (nodeId, mmDev->enumModeEnergyBs::OFF);
      m_rrc->EvictUsersFromSecondaryCell ();
      mmDev->SetCellState(mmDev->enumModeEnergyBs::OFF);
      EnHeuristicTrace(mmDev);
    }
  else
    NS_LOG_DEBUG ("BS state"
                  << " keep same state");
}

std::vector<std::pair<Ptr<MmWaveEnbNetDevice>, double>>
EnergyHeuristic::HeuristicDynamic (int bsToTurnOn[], int bsOn, int bsIdle, int bsSleep, int bsOff,
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
      Ptr<LteEnbRrc> m_rrc = mmDev->GetRrc ();
      NS_LOG_DEBUG ("UEs time distances from cell " << m_rrc->GetCellId ());
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
                  //save the UE closest position of this BS (index j)
                  double relativeSpeed =
                      node->GetObject<MobilityModel> ()->GetRelativeSpeed (
                          mmDev->GetNode ()->GetObject<MobilityModel> ());
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
                  double time = ueDist / relativeSpeed;
                  double ClosestUeTime = mmDev->GetClosestUeTime (); // get the attribute where is saved the closest attached UE position
                  //if the ue in the scenario (nodeIMSI) is closer compared to the one already saved, save it
                  if (time < ClosestUeTime)
                    {
                      mmDev->SetClosestUeTime (time);
                    }
                  NS_LOG_DEBUG ("BS: " << m_rrc->GetCellId () << " UE:" << nodeIMSI
                                        << " distance:" << ueDist << " pos: " << pos << " relativeSpeed: "
                                        << relativeSpeed << " time: " << time);
                }
            }
        }
      NS_LOG_DEBUG ("The closest for BS: " << m_rrc->GetCellId ()
                                    << " is time: " << mmDev->GetClosestUeTime());
    }
  //save all closest UEs of all the BS in the scenario into a Map
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
      //reset attribute position of closest UE
      mmDev->SetClosestUeTime (10000.0);
    }

  //order the vector or pairs
  std::sort (UeTime.begin (), UeTime.end (),
             [] (const std::pair<Ptr<MmWaveEnbNetDevice>, double> &left,
                 const std::pair<Ptr<MmWaveEnbNetDevice>, double> &right) {
               return left.second < right.second;
             });

  NS_LOG_DEBUG ("Print the map of BS-time ordered");
  for (std::pair<Ptr<MmWaveEnbNetDevice>, double> i : UeTime)
    NS_LOG_DEBUG ("BS " << i.first->GetCellId () << " with time " << i.second);

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
      Ptr<LteEnbRrc> m_rrc = mmDev->GetRrc ();
      NS_LOG_DEBUG ("UEs distances from cell " << m_rrc->GetCellId ());
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
                  NS_LOG_DEBUG ("BS: " << m_rrc->GetCellId () << " UE:" << nodeIMSI
                                        << " distance:" << ueDist << " pos: " << pos);
                }
            }
        }
      NS_LOG_DEBUG ("The closest for BS: " << m_rrc->GetCellId ()
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
  Ptr<LteEnbRrc> m_rrc = ltedev->GetRrc ();
  if (bsOn + bsIdle + bsSleep + bsOff != nMmWaveEnbNodes)
    {
      NS_FATAL_ERROR ("ERROR: number of BS we interact with( "
                      << bsOn + bsIdle + bsSleep + bsOff
                      << ") is different to the total number of BS in the scenario ("
                      << nMmWaveEnbNodes << ")");
    }
  int BShighestValues[bsOn];
  int bsToTurnOn[bsOn];
  for (int i = 0; i < bsOn; i++)
    {
      BShighestValues[i] = -1;
    }
  for (int j = 0; j < nMmWaveEnbNodes; j++)
    {
      //get the mmwave BS
      Ptr<MmWaveEnbNetDevice> mmDev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));

      for (int i = 0; i < bsOn; i++)
        {
          //save the bests (number equal to bsOn) BSs with highest NUeGoodSINR
          if (BShighestValues[i] < mmDev->GetNUeGoodSinr ())
            {
              BShighestValues[i] = mmDev->GetNUeGoodSinr (); // save N UEs value
              bsToTurnOn[i] = j; //save BS index
              break; //if this value is the highest exit the loop and check next BS
            }
        }
    }

  //turn on first N BSs
  for (int j = 0; j < bsOn; j++)
    {
      Ptr<MmWaveEnbNetDevice> mmDev =
          DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (bsToTurnOn[j]));
      NS_LOG_DEBUG ("BS to turn on ID: " << mmDev->GetCellId ());
      mmDev->TurnOn (mmDev->GetCellId (), m_rrc);
      EnHeuristicTrace(mmDev);
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

  //turn idle first N BSs
  for (int j = 0; j < bsIdle; j++)
    {
      Ptr<MmWaveEnbNetDevice> mmDev = UEVectorPairs[j].first;
      NS_LOG_DEBUG ("BS to turn Idle ID: " << UEVectorPairs[j].first->GetCellId ());
      mmDev->TurnIdle (mmDev->GetCellId (), m_rrc);
      EnHeuristicTrace(mmDev);
    }

  //turn sleep first N BSs
  for (int j = bsIdle; j < bsIdle + bsSleep; j++)
    {
      Ptr<MmWaveEnbNetDevice> mmDev = UEVectorPairs[j].first;
      NS_LOG_DEBUG ("BS to turn sleep ID: " << UEVectorPairs[j].first->GetCellId ());
      mmDev->TurnSleep (mmDev->GetCellId (), m_rrc);
      EnHeuristicTrace(mmDev);
    }

  //turn off first N BSs
  for (int j = bsIdle + bsSleep; j < bsIdle + bsSleep + bsOff; j++)
    {
      Ptr<MmWaveEnbNetDevice> mmDev = UEVectorPairs[j].first;
      NS_LOG_DEBUG ("BS to turn off ID: " << UEVectorPairs[j].first->GetCellId ());
      mmDev->TurnOff (mmDev->GetCellId (), m_rrc);
      EnHeuristicTrace(mmDev);
    }
}

void
EnergyHeuristic::EnHeuristicTrace(Ptr<MmWaveEnbNetDevice> mmDev)
{
  NS_LOG_LOGIC("EnHeuristicSizeTrace " << Simulator::Now().GetSeconds() << " " << mmDev->GetCellId() << " " << mmDev->GetBsState());
  // write to file
  if (!m_enHeuristicFile.is_open ())
    {
      m_enHeuristicFile.open (m_enHeuristicFilename);
      NS_LOG_LOGIC ("File opened");
      m_enHeuristicFile << "Timestamp" << " " << "cellId" << " " << "CellModeEnergy" << std::endl;
    }
  m_enHeuristicFile << Simulator::Now ().GetSeconds () << " " << mmDev->GetCellId() << " " << mmDev->GetBsState() << std::endl;
}

} // namespace mmwave

} // namespace ns3