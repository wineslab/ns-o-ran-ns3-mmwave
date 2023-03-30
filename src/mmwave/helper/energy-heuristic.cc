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
    }
  else if (r > p1 && r <= (p1 + p2))
    {
      NS_LOG_DEBUG ("BS state " << mmDev->enumModeEnergyBs::Idle);
      m_rrc->SetSecondaryCellHandoverAllowedStatus (nodeId, mmDev->enumModeEnergyBs::Idle);
      mmDev->SetCellState(mmDev->enumModeEnergyBs::Idle);
    }
  else if (r > (p1 + p2) && r <= (p1 + p2 + p3))
    {
      NS_LOG_DEBUG ("BS state " << mmDev->enumModeEnergyBs::Sleep);
      m_rrc->SetSecondaryCellHandoverAllowedStatus (nodeId, mmDev->enumModeEnergyBs::Sleep);
      m_rrc->EvictUsersFromSecondaryCell ();
      mmDev->SetCellState(mmDev->enumModeEnergyBs::Sleep);
    }
  else if (r > (p1 + p2 + p3) && r <= 1)
    {
      NS_LOG_DEBUG ("BS state " << mmDev->enumModeEnergyBs::OFF);
      m_rrc->SetSecondaryCellHandoverAllowedStatus (nodeId, mmDev->enumModeEnergyBs::OFF);
      m_rrc->EvictUsersFromSecondaryCell ();
      mmDev->SetCellState(mmDev->enumModeEnergyBs::OFF);
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
      std::map<uint16_t, Ptr<UeManager>> ue_attached =
          m_rrc->GetUeMap (); //list of attached UEs
      NS_LOG_DEBUG ("N ues attached: " << ue_attached.size ());
      uint64_t ListConnectedIMSI[ue_attached.size ()];
      int index = 0;
      //save IMSI of connected UE to the BS into ListConnectedIMSI array
      for (auto it = ue_attached.cbegin (); it != ue_attached.cend (); ++it)
        {
          ListConnectedIMSI[index] = it->second->GetImsi ();
          index++;
        }
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
                  for (uint64_t iarray = 0; iarray < ue_attached.size (); iarray++)
                    {
                      //save the UE closest position of this BS (index j)
                      if (ListConnectedIMSI[iarray] ==
                          nodeIMSI) //if the jDevs node is one of the connected UEs for this BS (index j)
                        {
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
                                         2)); //distance between UE (index iarray) and BS
                          double time = ueDist / relativeSpeed;
                          double ClosestUeTime =
                              mmDev
                                  ->GetClosestUeTime (); // get the attribute where is saved the closest attached UE position
                          //if the ue in the scenario (index iarray) is closer compared to the one already saved, save it
                          if (time < ClosestUeTime)
                            {
                              mmDev->SetClosestUeTime (time);
                            }
                          NS_LOG_DEBUG ("BS: " << m_rrc->GetCellId () << " UE:" << nodeIMSI
                                               << " distance:" << ueDist << " relativeSpeed: "
                                               << relativeSpeed << " time: " << time);
                        }
                    }
                }
            }
        }
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
      std::map<uint16_t, Ptr<UeManager>> ue_attached = m_rrc->GetUeMap (); //list of attached UEs
      NS_LOG_DEBUG ("N ues attached: " << ue_attached.size ());
      uint64_t ListConnectedIMSI[ue_attached.size ()];
      int index = 0;
      //save IMSI of connected UE to the BS into ListConnectedIMSI array
      for (auto it = ue_attached.cbegin (); it != ue_attached.cend (); ++it)
        {
          ListConnectedIMSI[index] = it->second->GetImsi ();
          index++;
        }
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
                  for (uint64_t iarray = 0; iarray < ue_attached.size (); iarray++)
                    {
                      //save the UE closest position of this BS (index j)
                      if (ListConnectedIMSI[iarray] ==
                          nodeIMSI) //if the jDevs node is one of the connected UEs for this BS (index j)
                        {
                          //get scenario ue pos
                          Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
                          //get the position of the GnB
                          Vector posGnB =
                              mmDev->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();
                          //getDistance between BS and UE
                          double ueDist =
                              sqrt (pow (pos.x - posGnB.x, 2) +
                                    pow (pos.y - posGnB.y,
                                         2)); //distance between UE (index iarray) and BS
                          std::pair<double, double> ClosestUepos =
                              mmDev
                                  ->GetClosestUePos (); // get the attribute where is saved the closest attached UE position
                          double ClosestUeposDist =
                              sqrt (pow (ClosestUepos.first - posGnB.x, 2) +
                                    pow (ClosestUepos.second - posGnB.y,
                                         2)); // actual distance (BS-UE) of the closest saved UE
                          //if the ue in the scenario (index iarray) is closer compared to the one already saved, save it
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
    }

  //turn sleep first N BSs
  for (int j = bsIdle; j < bsIdle + bsSleep; j++)
    {
      Ptr<MmWaveEnbNetDevice> mmDev = UEVectorPairs[j].first;
      NS_LOG_DEBUG ("BS to turn sleep ID: " << UEVectorPairs[j].first->GetCellId ());
      mmDev->TurnSleep (mmDev->GetCellId (), m_rrc);
    }

  //turn off first N BSs
  for (int j = bsIdle + bsSleep; j < bsIdle + bsSleep + bsOff; j++)
    {
      Ptr<MmWaveEnbNetDevice> mmDev = UEVectorPairs[j].first;
      NS_LOG_DEBUG ("BS to turn off ID: " << UEVectorPairs[j].first->GetCellId ());
      mmDev->TurnOff (mmDev->GetCellId (), m_rrc);
    }
}


//transform the input string of cluster into a vector of vector of values, substituting the id with their mmdev value
std::vector<std::vector<Ptr<MmWaveEnbNetDevice>>> EnergyHeuristic::ReadClusters( std::string clusters, uint8_t nMmWaveEnbNodes, NetDeviceContainer mmWaveEnbDevs){

  //int n_clusters=2;
  //std::string input = "[[5,6,7],[1],[2,3,4],[]]";
  std::string input = clusters;
  NS_LOG_DEBUG ("Input cluster as string: " << clusters);
  std::vector<std::vector<Ptr<MmWaveEnbNetDevice>>> arrays;
  std::stringstream ss(input);
  std::string item;
  
  while (getline(ss, item, ']')) {
    if (item.empty()) {
      continue;
    }
    item = item.substr(2);
    std::vector<Ptr<MmWaveEnbNetDevice>> array;
    std::stringstream arrayStream(item);
    std::string arrayItem;
    while (getline(arrayStream, arrayItem, ',')) {
      if (!arrayItem.empty() && isdigit(arrayItem[0])) {
        try {
          int num = std::stoi(arrayItem);

          for (int j = 0; j < nMmWaveEnbNodes; j++)
              {
                Ptr<MmWaveEnbNetDevice> mmdev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
                if(num==mmdev->GetCellId()){
                  array.push_back(mmdev);
                }
              }
        } catch (const std::invalid_argument &e) {
          NS_FATAL_ERROR ( "Error: " << arrayItem << " is not a valid integer.");
        }
      }
    }
    arrays.push_back(array);
  }
  
  // NS_LOG_DEBUG ("Cluster array");
  // for (const auto &array : arrays) {
  //   for (const auto &element : array) {
  //       NS_LOG_DEBUG (element << " ");
  //   }
  //   NS_LOG_DEBUG ("/n");
  // }

  for (uint i = 0; i < arrays.size(); i++)
  {
    for (uint j = 0; j < arrays[i].size(); j++)
    {
        NS_LOG_DEBUG( arrays[i][j]->GetCellId());
    }
  NS_LOG_DEBUG ("/n");
  }

  return arrays;
}


void EnergyHeuristic::MavenirHeuristic(uint8_t nMmWaveEnbNodes, NetDeviceContainer mmWaveEnbDevs, Ptr<LteEnbNetDevice> ltedev, int numberOfClusters, std::vector<std::vector<Ptr<MmWaveEnbNetDevice>>> clusters){
  //every time periodicity
  Ptr<MmWaveEnbNetDevice> smallestMmDev; //leaving it empty is not a problem since in the worst case is not used
  Ptr<LteEnbRrc> m_rrc = ltedev->GetRrc ();
  double smallestEekpi = 2200; // I set it higher than the c'threshold so in extreme cases it doesn't pass the next if control to turn off the Cell
  for (int j = 0; j < nMmWaveEnbNodes; j++) //for every cell
  {
    //get the mmwave BS
    Ptr<MmWaveEnbNetDevice> mmDev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
    if(mmDev->GetBsState()==1){//if the cell is turned ON
      NS_LOG_DEBUG ("EEKPI1 BS ID " << mmDev->GetCellId()<<" and state (turned ON) "<< mmDev->GetBsState());
      //compute eekpi and get the smallest one c'
      NS_LOG_DEBUG ("macPduInitialCellSpecific for EEKPI1 "<< mmDev->GetmacPduInitialCellSpecificAttr());
      NS_LOG_DEBUG ("prbUtilizationDl for EEKPI1 "<< mmDev->GetprbUtilizationDlAttr());
      double eekpi = 2200;
      if(mmDev->GetprbUtilizationDlAttr()!=0){
        eekpi= (double)mmDev->GetmacPduInitialCellSpecificAttr()/(mmDev->GetprbUtilizationDlAttr()/139);
        NS_LOG_DEBUG ("EEKPI1 "<< eekpi << " for cell "<< mmDev->GetCellId());
      }
      else{
        NS_LOG_DEBUG ("EEKPI1 "<< eekpi << " because prbUtilizationDl=0");
      }
      //mmDev->Seteekpi(eekpi); //save the eekpi for each BS to use it on the second part of the fuction (do we really need it??)
      if(eekpi<smallestEekpi){
        smallestEekpi= eekpi;
        smallestMmDev= mmDev;
      }
    }
  }
  NS_LOG_DEBUG ("Smallest EEKPI1 "<< smallestEekpi);
  //if c'< threshold value (2200.0) -> turn off BS (energy saving)
  if(smallestEekpi<2200.0){
    smallestMmDev->TurnOff(smallestMmDev->GetCellId(), m_rrc);
    smallestMmDev->SetturnOffTime(Simulator::Now().GetSeconds());
    NS_LOG_DEBUG ("Turn off the smallest EEKPI1 BS ID "<< smallestMmDev->GetCellId());
  }

  double smallestEekpi2 = 1800; // I set it higher than the c' threshold so in extreme cases it doesn't pass the next if control to turn on the Cell
  Ptr<MmWaveEnbNetDevice> smallestMmDev2; //leaveing it empty is not a problem since in the worst case is not used
  //for every cell turned off (except the c')
  for (int j = 0; j < nMmWaveEnbNodes; j++) //for every cell
  {
    //get the mmwave BS
    Ptr<MmWaveEnbNetDevice> mmDev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
     
    if(mmDev!= smallestMmDev && mmDev->GetBsState()==0){//if the cell is turned OFF (except the c', so the one just turned off)
      NS_LOG_DEBUG ("EEKPI2 BS ID (except the one just turned OFF) " << mmDev->GetCellId()<<" and state (turned OFF) "<< mmDev->GetBsState());
      
      for (uint i1 = 0; i1 < clusters.size(); i1++) {
        //check if the subject cell is inside this cluster
        if (std::find(clusters[i1].begin(), clusters[i1].end(), mmDev) != clusters[i1].end()){ //if the cell is here, perform the operation for all the neighbors
          double sumWeightedEekpi2=0;
          int cellToCount=0; // number of cells neighbors, turned ON and prbUtilizationDl!=0 (important to compute the eekpi formula) 
          //calculate the eekpi       
          for (uint i2 = 0; i2 < clusters[i1].size(); i2++) {
            Ptr<MmWaveEnbNetDevice> neighMmDev=clusters[i1][i2];
            //skip the mmdev cell (subject cell) as neighbor
            //moreover the neighbor cell has to be turned ON
            if(neighMmDev != mmDev && neighMmDev->GetBsState()==1){
              NS_LOG_DEBUG ("EEKPI2 Cell ID " << mmDev->GetCellId() << "has the following neighbor " << neighMmDev->GetCellId());
              //calculate eekpi2 new formula for each neighbour
              NS_LOG_DEBUG ("macPduInitialCellSpecific for EEKPI2 "<< mmDev->GetmacPduInitialCellSpecificAttr());
              NS_LOG_DEBUG ("prbUtilizationDl for EEKPI2 "<< mmDev->GetprbUtilizationDlAttr());
              double eekpi = 0;
              if(mmDev->GetprbUtilizationDlAttr()!=0){
                eekpi= (double)mmDev->GetmacPduInitialCellSpecificAttr()/(mmDev->GetprbUtilizationDlAttr()/139);
                cellToCount++;
              }
              else{
                NS_LOG_DEBUG ("In EEKPI2 prbUtilizationDl=0");
              }
              //eekpi is about the neighbour cell
              double weightedEekpi2=eekpi* 1.0* exp(-0.1*(Simulator::Now().GetSeconds() - neighMmDev->GetturnOffTime()) ); 
              sumWeightedEekpi2=sumWeightedEekpi2+weightedEekpi2;
            }
          }
          //do the average and save it to the subject cell in variable "j" (the one turned off)
          //cellToSkip is substracted in case eekpi can't be computed due to prbUtilizationDl=0
          double avgEekpi2=sumWeightedEekpi2/cellToCount;
          NS_LOG_DEBUG ("AVG weighted EEKPI2 "<< avgEekpi2 << " for cell "<< mmDev->GetCellId());       
          if(avgEekpi2<smallestEekpi2){
            smallestEekpi2=avgEekpi2;
            smallestMmDev2=mmDev;
          }
        }
        //else, move to the next cluster
      }
    }
  }
  NS_LOG_DEBUG ("Smallest AVG weighted EEKPI2 "<< smallestEekpi2);
  //obtain all the smallest eekpi2 avg values and if eekpi2 < threshold value ( 1800 ) -> turn on BS
  if(smallestEekpi2<1800.0){
    smallestMmDev2->TurnOn (smallestMmDev2->GetCellId (), m_rrc);
    NS_LOG_DEBUG ("Turn on the smallest EEKPI2 BS ID "<< smallestMmDev2->GetCellId());
  }

}

}

} // namespace ns3