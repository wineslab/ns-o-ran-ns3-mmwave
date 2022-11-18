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
#include <ns3/lte-ue-net-device.h>
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
}

TypeId EnergyHeuristic::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::EnergyHeuristic")
    .SetParent<Object>()
    .AddConstructor<EnergyHeuristic>();
  return tid;
}

void EnergyHeuristic::CountBestUesSinr(double SINRth, Ptr<MmWaveEnbNetDevice> mmdev){
  //reset parameter
  mmdev->SetNUeGoodSinr(0);
  Ptr<LteEnbRrc> m_rrc= mmdev->GetRrc();
  //get connected UEs from BS
  std::map<uint16_t, Ptr<UeManager>>ue_attached= m_rrc->GetUeMap(); //list of attached UEs
  NS_LOG_DEBUG ("N ues attached: "<<ue_attached.size());
  for(auto it = ue_attached.cbegin(); it != ue_attached.cend(); ++it)
  {
    //yes, UEs are attached during simulation
    //std::map<uint64_t, std::map<uint16_t, long double>> -- <imsi, <cellid, sinr>
    std::map<uint64_t, std::map<uint16_t, long double>> m_l3sinrMap=mmdev->Getl3sinrMap();
    double sinrThisCell = 10 * std::log10(m_l3sinrMap[it->second->GetImsi()][mmdev->GetCellId()]);
    double convertedSinr = L3RrcMeasurements::ThreeGppMapSinr (sinrThisCell);
    NS_LOG_DEBUG ( "sinrThisCell: "<< convertedSinr);
    if (convertedSinr > SINRth)//over 13 is a good SINR range = over 73 convertedSinr
    {
      uint16_t NUeGoodSinr =  mmdev->GetNUeGoodSinr();
      NUeGoodSinr++;
      mmdev->SetNUeGoodSinr(NUeGoodSinr);
    }   
  }
  NS_LOG_DEBUG ( "NUeGoodSinr for BS "<< mmdev->GetCellId()<<" is: "<< mmdev->GetNUeGoodSinr()); //number of UEs with a good SINR value
}

void
EnergyHeuristic::ProbabilityState (double p1, double p2, double p3, double p4, uint16_t nodeId, Ptr<MmWaveEnbNetDevice> mmdev)
{
  NS_LOG_DEBUG ("Sim time " << Simulator::Now ().GetSeconds ());
  NS_LOG_DEBUG ("Enb name " << nodeId);
  Ptr<LteEnbRrc> m_rrc = mmdev->GetRrc();
  Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
  double r = x->GetValue();
  NS_LOG_DEBUG ("Prob " << r);
  if (r <= p1)
    {
      NS_LOG_DEBUG ("BS state " << mmdev->enum_state_BS::ON);
      m_rrc->SetSecondaryCellHandoverAllowedStatus (nodeId, mmdev->enum_state_BS::ON);
      mmdev->SetCellState(mmdev->enum_state_BS::ON);
    }
  else if (r > p1 && r <= (p1 + p2))
    {
      NS_LOG_DEBUG ("BS state " << mmdev->enum_state_BS::Idle);
      m_rrc->SetSecondaryCellHandoverAllowedStatus (nodeId, mmdev->enum_state_BS::Idle);
      mmdev->SetCellState(mmdev->enum_state_BS::Idle);
    }
  else if (r > (p1 + p2) && r <= (p1 + p2 + p3))
    {
      NS_LOG_DEBUG ("BS state " << mmdev->enum_state_BS::Sleep);
      m_rrc->SetSecondaryCellHandoverAllowedStatus (nodeId, mmdev->enum_state_BS::Sleep);
      m_rrc->EvictUsersFromSecondaryCell ();
      mmdev->SetCellState(mmdev->enum_state_BS::Sleep);
    }
  else if (r > (p1 + p2 + p3) && r <= 1)
    {
      NS_LOG_DEBUG ("BS state " << mmdev->enum_state_BS::OFF);
      m_rrc->SetSecondaryCellHandoverAllowedStatus (nodeId, mmdev->enum_state_BS::OFF);
      m_rrc->EvictUsersFromSecondaryCell ();
      mmdev->SetCellState(mmdev->enum_state_BS::OFF);
    }
  else
    NS_LOG_DEBUG ("BS state"
                  << " keep same state");
}

std::vector<std::pair<Ptr<MmWaveEnbNetDevice>, double>> EnergyHeuristic::HeuristicDynamic(int BStoTurnON[], int BsON, int BsIdle, int BsSleep, int BsOFF, int nMmWaveEnbNodes, NetDeviceContainer mmWaveEnbDevs)
{
  //for others BS get position of closest UE
  for (int j = 0; j < nMmWaveEnbNodes; j++)
    {
      //if BS is already turned ON (from previous piece of code) skip this BS
      bool found = false;
      for (int i = 0; i < BsON; i++)
        {
          if (BStoTurnON[i] == j)
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
      Ptr<MmWaveEnbNetDevice> mmdev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
      Ptr<LteEnbRrc> m_rrc = mmdev->GetRrc ();
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
                                  mmdev->GetNode ()->GetObject<MobilityModel> ());
                          //get scenario ue pos
                          Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
                          //get the position of the GnB
                          Vector posGnB =
                              mmdev->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();
                          //getDistance between BS and UE
                          double ueDist =
                              sqrt (pow (pos.x - posGnB.x, 2) +
                                    pow (pos.y - posGnB.y,
                                         2)); //distance between UE (index iarray) and BS
                          double time = ueDist / relativeSpeed;
                          double ClosestUeTime =
                              mmdev->GetClosestUeTime (); // get the attribute where is saved the closest attached UE position
                          //if the ue in the scenario (index iarray) is closer compared to the one already saved, save it
                          if (time < ClosestUeTime)
                            {
                              mmdev->SetClosestUeTime( time);

                            }
                          NS_LOG_DEBUG ( "BS: " << m_rrc->GetCellId () << " UE:" << nodeIMSI
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
      for (int i = 0; i < BsON; i++)
        {
          if (j == BStoTurnON[i])
            {
              found = true;
              break;
            }
        }
      if (found)
        {
          continue;
        }
      Ptr<MmWaveEnbNetDevice> mmdev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
      UeTime.push_back (std::pair<Ptr<MmWaveEnbNetDevice>, double> (mmdev, mmdev->GetClosestUeTime ()));
      //reset attribute position of closest UE
      mmdev->SetClosestUeTime (10000.0);
    }

  //order the vector or pairs
  std::sort (UeTime.begin (), UeTime.end (),
             [] (const std::pair<Ptr<MmWaveEnbNetDevice>, double> &left,
                 const std::pair<Ptr<MmWaveEnbNetDevice>, double> &right) {
               return left.second < right.second;
             });

  NS_LOG_DEBUG ( "Print the map of BS-time ordered");
  for (std::pair<Ptr<MmWaveEnbNetDevice>, double> i : UeTime)
    NS_LOG_DEBUG ( "BS " << i.first->GetCellId () << " with time " << i.second);

  return UeTime;
}

std::vector<std::pair<Ptr<MmWaveEnbNetDevice>, double>> EnergyHeuristic::HeuristicStatic (int BStoTurnON[], int BsON, int BsIdle, int BsSleep, int BsOFF, int nMmWaveEnbNodes, NetDeviceContainer mmWaveEnbDevs)
{
  //for others BS get position of closest UE
  for (int j = 0; j < nMmWaveEnbNodes; j++)
    {
      //if BS is already turned ON (from previous piece of code) skip this BS
      bool found = false;
      for (int i = 0; i < BsON; i++)
        {
          if (BStoTurnON[i] == j)
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
      Ptr<MmWaveEnbNetDevice> mmdev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
      Ptr<LteEnbRrc> m_rrc = mmdev->GetRrc ();
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
                          //get scenario ue pos
                          Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
                          //get the position of the GnB
                          Vector posGnB =
                              mmdev->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();
                          //getDistance between BS and UE
                          double ueDist =
                              sqrt (pow (pos.x - posGnB.x, 2) +
                                    pow (pos.y - posGnB.y,
                                         2)); //distance between UE (index iarray) and BS
                          std::pair<double, double> ClosestUepos =
                              mmdev->GetClosestUePos (); // get the attribute where is saved the closest attached UE position
                          double ClosestUeposDist =
                              sqrt (pow (ClosestUepos.first - posGnB.x, 2) +
                                    pow (ClosestUepos.second - posGnB.y,
                                         2)); // actual distance (BS-UE) of the closest saved UE
                          //if the ue in the scenario (index iarray) is closer compared to the one already saved, save it
                          if (ueDist < ClosestUeposDist)
                            {
                              std::pair<double, double> uePos = {pos.x, pos.y};
                              mmdev->SetClosestUePos (uePos);
                            }
                          NS_LOG_DEBUG ("BS: " << m_rrc->GetCellId () << " UE:" << nodeIMSI
                                               << " distance:" << ueDist << " pos: " << pos);
                        }
                    }
                }
            }
        }
      NS_LOG_DEBUG ("The closest for BS: " << m_rrc->GetCellId ()
                                           << " is pos: " << mmdev->GetClosestUePos ());
    }
  //save all closest UEs of all the BS in the scenario into a Map
  std::vector<std::pair<Ptr<MmWaveEnbNetDevice>, double>> UeDistances; //cellID, UEdistance
  for (int j = 0; j < nMmWaveEnbNodes; j++)
    {
      //if BS is already turned ON skip this BS
      bool found = false;
      for (int i = 0; i < BsON; i++)
        {
          if (j == BStoTurnON[i])
            {
              found = true;
              break;
            }
        }
      if (found)
        {
          continue;
        }
      Ptr<MmWaveEnbNetDevice> mmdev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
      std::pair<double, double> ClosestUepos = mmdev->GetClosestUePos (); // get the saved UE pos
      Vector posGnB = mmdev->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();
      double ClosestUeposDist =
          sqrt (pow (ClosestUepos.first - posGnB.x, 2) + pow (ClosestUepos.second - posGnB.y, 2));
      UeDistances.push_back (std::pair<Ptr<MmWaveEnbNetDevice>, double> (mmdev, ClosestUeposDist));
      //reset attribute position of closest UE
      std::pair<double, double> uePos = {10000.0,10000.0};
      mmdev->SetClosestUePos (uePos);
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

void EnergyHeuristic::TurnOnBsSinrPos (uint8_t nMmWaveEnbNodes, NetDeviceContainer mmWaveEnbDevs, std::string heuristic, int BSstatus[])
{
  int BsON = BSstatus[0];
  int BsIdle = BSstatus[1];
  int BsSleep = BSstatus[2];
  int BsOFF= BSstatus[3];
  if (BsON + BsIdle + BsSleep + BsOFF != nMmWaveEnbNodes)
    {
      NS_FATAL_ERROR ("ERROR: number of BS we interact with( "
                      << BsON + BsIdle + BsSleep + BsOFF
                      << ") is different to the total number of BS in the scenario ("
                      << nMmWaveEnbNodes << ")");
    }
  int BShighestValues[BsON];
  int BStoTurnON[BsON];
  for (int i = 0; i < BsON; i++)
    {
      BShighestValues[i] = -1;
    }
  for (int j = 0; j < nMmWaveEnbNodes; j++)
    {
      //get the mmwave BS
      Ptr<MmWaveEnbNetDevice> mmdev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));

      for (int i = 0; i < BsON; i++)
        {
          //save the bests (number equal to BsON) BSs with highest NUeGoodSINR
          if (BShighestValues[i] < mmdev->GetNUeGoodSinr ())
            {
              BShighestValues[i] = mmdev->GetNUeGoodSinr (); // save N UEs value
              BStoTurnON[i] = j; //save BS index
              break; //if this value is the highest exit the loop and check next BS
            }
        }
    }

  //turn on first N BSs
  for (int j = 0; j < BsON; j++)
    {
      Ptr<MmWaveEnbNetDevice> mmdev =
          DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (BStoTurnON[j]));
      NS_LOG_DEBUG ("BS to turn on ID: " << mmdev->GetCellId ());
      mmdev->TurnOn (mmdev->GetCellId ());
    }

  std::vector<std::pair<Ptr<MmWaveEnbNetDevice>, double>> UEVectorPairs;
  if (heuristic=="static"){
    UEVectorPairs=HeuristicStatic(BStoTurnON, BsON, BsIdle, BsSleep, BsOFF, nMmWaveEnbNodes, mmWaveEnbDevs);
  }
  else if (heuristic=="dynamic"){
    UEVectorPairs=HeuristicDynamic(BStoTurnON, BsON, BsIdle, BsSleep, BsOFF, nMmWaveEnbNodes, mmWaveEnbDevs);
  }

    //turn idle first N BSs
  for (int j = 0; j < BsIdle; j++)
    {
      Ptr<MmWaveEnbNetDevice> mmdev = UEVectorPairs[j].first;
      NS_LOG_DEBUG ( "BS to turn Idle ID: " << UEVectorPairs[j].first->GetCellId ());
      mmdev->TurnIdle (mmdev->GetCellId ());
    }

  //turn sleep first N BSs
  for (int j = BsIdle; j < BsIdle + BsSleep; j++)
    {
      Ptr<MmWaveEnbNetDevice> mmdev = UEVectorPairs[j].first;
      NS_LOG_DEBUG ( "BS to turn sleep ID: " << UEVectorPairs[j].first->GetCellId ());
      mmdev->TurnSleep (mmdev->GetCellId ());
    }

  //turn off first N BSs
  for (int j = BsIdle + BsSleep; j < BsIdle + BsSleep + BsOFF; j++)
    {
      Ptr<MmWaveEnbNetDevice> mmdev = UEVectorPairs[j].first;
      NS_LOG_DEBUG ( "BS to turn off ID: " << UEVectorPairs[j].first->GetCellId ());
      mmdev->TurnOff (mmdev->GetCellId ());
    }
}


// void EnergyHeuristic::MavenirHeuristic(uint8_t nMmWaveEnbNodes, NetDeviceContainer mmWaveEnbDevs){
//   //every time t = to the time stamp of cu_cp ----------------------------------------------------
//   Ptr<MmWaveEnbNetDevice> smallestmmdev;
//   double smalleekpi = 2300; // I set it higher than the c'threshold so in extreme cases it doesn't pass the next if control
//   for (int j = 0; j < nMmWaveEnbNodes; j++) //for every cell
//   {
//     //get the mmwave BS
//     Ptr<MmWaveEnbNetDevice> mmdev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
     
//     if(mmdev->GetBsState()==1){//if the cell is turned ON
//       //compute eekpi and get the smallest one c'
//       double eekpi= (double)mmdev->GetmacPduInitialCellSpecificAttr()/mmdev->GetprbUtilizationDlAttr()/139;
//       if(eekpi<smalleekpi){
//         smalleekpi= eekpi;
//         smallestmmdev= mmdev;
//       }
//     }
//   }
//   //if c'< threshold value (2200.0) -> turn off BS
//   if(smalleekpi<2200.0){
//     smallestmmdev->TurnOff();
//   }

//   //for every cell turned off (except the c')
//   for (int j = 0; j < nMmWaveEnbNodes; j++) //for every cell
//   {
//     //get the mmwave BS
//     Ptr<MmWaveEnbNetDevice> mmdev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
     
//     if(mmdev!= smallestmmdev && mmdev->GetBsState()==0){//if the cell is turned OFF
//         //get neghbors of the subject cell that are turned on
        
//         //calculate eekpi2 new formula for each neighbour
//         //do the average and save it to the subject cell (the one turned off)
//     }
//   }


//   //obtain all the eekpi2 avg values and keep the smallest one = k'
//   //if k'< threshold value ( 1800 ) -> turn on BS

// }

}

}