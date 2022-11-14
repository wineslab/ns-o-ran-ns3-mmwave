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
#include <random>

using namespace ns3;
using namespace mmwave;

/**
 * Scenario Three
 * 
 */

NS_LOG_COMPONENT_DEFINE ("ScenarioThree");

namespace ns3 {

class EnergyHeuristic : public Object{
  public:
  static TypeId GetTypeId (void);

  EnergyHeuristic ();

  virtual ~EnergyHeuristic (void);
  std::vector<std::pair<Ptr<MmWaveEnbNetDevice>, double>> HeuristicDynamic(int BStoTurnON[], int BsON, int BsIdle, int BsSleep, int BsOFF, int nMmWaveEnbNodes, NetDeviceContainer mmWaveEnbDevs);
  std::vector<std::pair<Ptr<MmWaveEnbNetDevice>, double>> HeuristicStatic (int BStoTurnON[], int BsON, int BsIdle, int BsSleep, int BsOFF, int nMmWaveEnbNodes, NetDeviceContainer mmWaveEnbDevs);
  void TurnOnBSSinrPos (uint8_t nMmWaveEnbNodes, NetDeviceContainer mmWaveEnbDevs, std::string heuristic, int BSstatus[]);
  void Probability_state (double p1, double p2, double p3, double p4, uint16_t nodeId, Ptr<MmWaveEnbNetDevice> mmdev);
  void CountBestUesSINR(double SINRth, Ptr<MmWaveEnbNetDevice> mmdev);

};

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

void EnergyHeuristic::CountBestUesSINR(double SINRth, Ptr<MmWaveEnbNetDevice> mmdev){
  //reset parameter
  mmdev->SetNUeGoodSINR(0);
  ns3::Ptr<ns3::LteEnbRrc> m_rrc= mmdev->GetRrc();
  //get connected UEs from BS
  std::map<uint16_t, ns3::Ptr<ns3::UeManager>>ue_attached= m_rrc->GetUeMap(); //list of attached UEs
  NS_LOG_DEBUG ("N ues attached: "<<ue_attached.size());
  for(auto it = ue_attached.cbegin(); it != ue_attached.cend(); ++it)
  {
    //yes, UEs are attached during simulation
    ImsiCellIdPair_t cid {it->second->GetImsi(), mmdev->GetCellId()};
    std::map<ImsiCellIdPair_t, long double> m_l3sinrMap=mmdev->getl3sinrMap();
    double sinrThisCell = 10 * std::log10(m_l3sinrMap[cid]);
    double convertedSinr = L3RrcMeasurements::ThreeGppMapSinr (sinrThisCell);
    NS_LOG_DEBUG ( "sinrThisCell: "<< convertedSinr);
    if (convertedSinr > SINRth)//over 13 is a good SINR range = over 73 convertedSinr
    {
      uint16_t NUeGoodSINR =  mmdev->GetNUeGoodSINR();
      NUeGoodSINR++;
      mmdev->SetNUeGoodSINR(NUeGoodSINR);
    }   
  }
  NS_LOG_DEBUG ( "NUeGoodSINR for BS "<< mmdev->GetCellId()<<" is: "<< mmdev->GetNUeGoodSINR()); //number of UEs with a good SINR value
}

void
EnergyHeuristic::Probability_state (double p1, double p2, double p3, double p4, uint16_t nodeId, Ptr<MmWaveEnbNetDevice> mmdev)
{
  NS_LOG_DEBUG ("Sim time " << Simulator::Now ().GetSeconds ());
  NS_LOG_DEBUG ("Enb name " << nodeId);
  Ptr<ns3::LteEnbRrc> m_rrc = mmdev->GetRrc();
  std::random_device rd;
  std::default_random_engine eng (rd ());
  std::uniform_real_distribution<double> distr (0, 1);
  double r = distr (eng);
  NS_LOG_DEBUG ("Prob " << r << "\n");
  if (r <= p1)
    {
      NS_LOG_DEBUG ("BS state " << mmdev->enum_state_BS::ON);
      m_rrc->SetSecondaryCellHandoverAllowedStatus (nodeId, mmdev->enum_state_BS::ON);
      mmdev->setCellState(mmdev->enum_state_BS::ON);
    }
  else if (r > p1 && r <= (p1 + p2))
    {
      NS_LOG_DEBUG ("BS state " << mmdev->enum_state_BS::Idle);
      m_rrc->SetSecondaryCellHandoverAllowedStatus (nodeId, mmdev->enum_state_BS::Idle);
      mmdev->setCellState(mmdev->enum_state_BS::Idle);
    }
  else if (r > (p1 + p2) && r <= (p1 + p2 + p3))
    {
      NS_LOG_DEBUG ("BS state " << mmdev->enum_state_BS::Sleep);
      m_rrc->SetSecondaryCellHandoverAllowedStatus (nodeId, mmdev->enum_state_BS::Sleep);
      m_rrc->EvictUsersFromSecondaryCell ();
      mmdev->setCellState(mmdev->enum_state_BS::Sleep);
    }
  else if (r > (p1 + p2 + p3) && r <= 1)
    {
      NS_LOG_DEBUG ("BS state " << mmdev->enum_state_BS::OFF);
      m_rrc->SetSecondaryCellHandoverAllowedStatus (nodeId, mmdev->enum_state_BS::OFF);
      m_rrc->EvictUsersFromSecondaryCell ();
      mmdev->setCellState(mmdev->enum_state_BS::OFF);
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
      ns3::Ptr<ns3::LteEnbRrc> m_rrc = mmdev->GetRrc ();
      std::map<uint16_t, ns3::Ptr<ns3::UeManager>> ue_attached =
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
      //reset closest UE time
      mmdev->SetClosestUETime(10000.0);
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
                              mmdev->GetClosestUETime (); // get the attribute where is saved the closest attached UE position
                          //if the ue in the scenario (index iarray) is closer compared to the one already saved, save it
                          if (time < ClosestUeTime)
                            {
                              mmdev->SetClosestUETime( time);

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
      UeTime.push_back (std::pair<Ptr<MmWaveEnbNetDevice>, double> (mmdev, mmdev->GetClosestUETime ()));
      //reset attribute position of closest UE
      mmdev->SetClosestUETime (10000.0);
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
      ns3::Ptr<ns3::LteEnbRrc> m_rrc = mmdev->GetRrc ();
      std::map<uint16_t, ns3::Ptr<ns3::UeManager>> ue_attached =
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
      //reset position of closest UE
      mmdev->SetClosestUEPos({10000.0,10000.0});
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
                              mmdev->GetClosestUEPos (); // get the attribute where is saved the closest attached UE position
                          double ClosestUeposDist =
                              sqrt (pow (ClosestUepos.first - posGnB.x, 2) +
                                    pow (ClosestUepos.second - posGnB.y,
                                         2)); // actual distance (BS-UE) of the closest saved UE
                          //if the ue in the scenario (index iarray) is closer compared to the one already saved, save it
                          if (ueDist < ClosestUeposDist)
                            {
                              std::pair<double, double> uePos = {pos.x, pos.y};
                              mmdev->SetClosestUEPos (uePos);
                            }
                          NS_LOG_DEBUG ("BS: " << m_rrc->GetCellId () << " UE:" << nodeIMSI
                                               << " distance:" << ueDist << " pos: " << pos);
                        }
                    }
                }
            }
        }
      NS_LOG_DEBUG ("The closest for BS: " << m_rrc->GetCellId ()
                                           << " is pos: " << mmdev->GetClosestUEPos ());
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
      std::pair<double, double> ClosestUepos = mmdev->GetClosestUEPos (); // get the saved UE pos
      Vector posGnB = mmdev->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();
      double ClosestUeposDist =
          sqrt (pow (ClosestUepos.first - posGnB.x, 2) + pow (ClosestUepos.second - posGnB.y, 2));
      UeDistances.push_back (std::pair<Ptr<MmWaveEnbNetDevice>, double> (mmdev, ClosestUeposDist));
      //reset attribute position of closest UE
      std::pair<double, double> uePos = {10000.0,10000.0};
      mmdev->SetClosestUEPos (uePos);
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

void EnergyHeuristic::TurnOnBSSinrPos (uint8_t nMmWaveEnbNodes, NetDeviceContainer mmWaveEnbDevs, std::string heuristic, int BSstatus[])
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
          if (BShighestValues[i] < mmdev->GetNUeGoodSINR ())
            {
              BShighestValues[i] = mmdev->GetNUeGoodSINR (); // save N UEs value
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
      mmdev->TurnON (mmdev->GetCellId ());
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
      mmdev->TurnOFF (mmdev->GetCellId ());
    }
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
//     smallestmmdev->TurnOFF();
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
  NS_LOG_UNCOND ("Position +****************************** " << model->GetPosition () << " at time "
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
    g_hoSinrDifference ("hoSinrDifference",
                        "The value for which an handover between MmWave eNB is triggered",
                        ns3::DoubleValue (3), ns3::MakeDoubleChecker<double> ());

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
                                       ns3::UintegerValue (40), // this was the for ther TS use case, 52 is default, 30 is for ES
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
    " Random sleeping (0),"
    " Static sleeping (1),"
    " Dynamic sleeping (2)",
    ns3::UintegerValue (0), ns3::MakeUintegerChecker<uint8_t> ());
static ns3::GlobalValue g_prob_ON (
    "prob_ON",
    "Probabilities for eachy that BS in turning ON for the random sleeping heuristic",
    ns3::DoubleValue (0.6038), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_prob_Idle (
    "prob_Idle",
    "Probabilities for eachy that BS in turning Idle for the random sleeping heuristic",
    ns3::DoubleValue (0.3854), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_prob_Sleep (
    "prob_Sleep",
    "Probabilities for eachy that BS in turning Sleep for the random sleeping heuristic",
    ns3::DoubleValue (0.0107), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_prob_OFF (
    "prob_OFF",
    "Probabilities for eachy that BS in turning Off for the random sleeping heuristic",
    ns3::DoubleValue (0.0), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_SINRth (
    "SINRth",
    "SINR threshold for static and dynamic sleeping heuristic",
    ns3::DoubleValue (73.0), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_BsON (
    "BsON",
    "number of BS to turn ON for static and dynamic sleeping heuristic",
    ns3::UintegerValue (2), ns3::MakeUintegerChecker<uint8_t> ());
static ns3::GlobalValue BsIdle (
    "BsIdle",
    "number of BS to turn IDLE for static and dynamic sleeping heuristic",
    ns3::UintegerValue (2), ns3::MakeUintegerChecker<uint8_t> ());
static ns3::GlobalValue BsSleep (
    "BsSleep",
    "number of BS to turn Sleep for static and dynamic sleeping heuristic",
    ns3::UintegerValue (2), ns3::MakeUintegerChecker<uint8_t> ());
static ns3::GlobalValue BsOFF (
    "BsOFF",
    "number of BS to turn Off for static and dynamic sleeping heuristic",
    ns3::UintegerValue (1), ns3::MakeUintegerChecker<uint8_t> ());

int
main (int argc, char *argv[])
{
  LogComponentEnableAll (LOG_PREFIX_ALL);
  LogComponentEnable ("ScenarioThree", LOG_LEVEL_DEBUG);
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
  double maxXAxis = 4300;
  // The maximum Y coordinate of the scenario
  double maxYAxis = 4300;

  // Command line arguments
  CommandLine cmd;
  cmd.Parse (argc, argv);

  bool harqEnabled = true;

  UintegerValue uintegerValue;
  BooleanValue booleanValue;
  StringValue stringValue;
  DoubleValue doubleValue;

  GlobalValue::GetValueByName ("hoSinrDifference", doubleValue);
  double hoSinrDifference = doubleValue.Get ();
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
  //heuristic parameters
  GlobalValue::GetValueByName ("heuristicType", uintegerValue);
  uint8_t heuristicType = uintegerValue.Get ();
  GlobalValue::GetValueByName ("prob_ON", doubleValue);
  double prob_ON = doubleValue.Get ();
  GlobalValue::GetValueByName ("prob_Idle", doubleValue);
  double prob_Idle = doubleValue.Get ();
  GlobalValue::GetValueByName ("prob_Sleep", doubleValue);
  double prob_Sleep = doubleValue.Get ();
  GlobalValue::GetValueByName ("prob_OFF", doubleValue);
  double prob_OFF = doubleValue.Get ();
  GlobalValue::GetValueByName ("SINRth", doubleValue);
  double SINRth = doubleValue.Get ();
  GlobalValue::GetValueByName ("BsON", uintegerValue);
  int BsON = uintegerValue.Get ();
  GlobalValue::GetValueByName ("BsIdle", uintegerValue);
  int BsIdle = uintegerValue.Get ();
  GlobalValue::GetValueByName ("BsSleep", uintegerValue);
  int BsSleep = uintegerValue.Get ();
  GlobalValue::GetValueByName ("BsOFF", uintegerValue);
  int BsOFF = uintegerValue.Get ();

  NS_LOG_UNCOND ("rlcAmEnabled " << rlcAmEnabled << " bufferSize " << unsigned(bufferSize)
                                 << " OutageThreshold " << outageThreshold << " HandoverMode " << handoverMode
                                 << " BasicCellId " << unsigned(basicCellId) << " e2TermIp " << e2TermIp
                                 << " enableE2FileLogging " << enableE2FileLogging << " minSpeed "
                                 << minSpeed << " maxSpeed " << maxSpeed << " numberofRaPreambles " << unsigned(numberOfRaPreambles));

  // Get current time
  time_t rawtime;
  struct tm *timeinfo;
  char buffer[80];
  time (&rawtime);
  timeinfo = localtime (&rawtime);
  strftime (buffer, 80, "%d_%m_%Y_%I_%M_%S", timeinfo);
  std::string time_str (buffer);

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

  GlobalValue::GetValueByName ("scheduleControlMessages", booleanValue);
  bool scheduleControlMessages = booleanValue.Get ();

  NS_LOG_UNCOND("e2lteEnabled " << e2lteEnabled 
    << " e2nrEnabled " << e2nrEnabled
    << " e2du " << e2du
    << " e2cuCp " << e2cuCp
    << " e2cuUp " << e2cuUp
    << " reducedPmValues " << reducedPmValues
    << " controlFilename " << controlFilename
    << " indicationPeriodicity " << indicationPeriodicity
    << " ScheduleControlMessages " << scheduleControlMessages
  );

  Config::SetDefault ("ns3::LteEnbNetDevice::ControlFileName", StringValue(controlFilename));
  Config::SetDefault ("ns3::LteEnbNetDevice::E2Periodicity", DoubleValue (indicationPeriodicity));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::E2Periodicity", DoubleValue (indicationPeriodicity));

  Config::SetDefault ("ns3::MmWaveHelper::E2ModeLte", BooleanValue(e2lteEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::E2ModeNr", BooleanValue(e2nrEnabled));
  
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

  // set to false to use the 3GPP radiation pattern (proper configuration of the bearing and downtilt angles is needed)
  Config::SetDefault ("ns3::ThreeGppAntennaArrayModel::IsotropicElements", BooleanValue (true));
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
  Config::SetDefault ("ns3::LteEnbRrc::HoSinrDifference", DoubleValue (hoSinrDifference));


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
      float x= pow(-1,floor(i/2))*isd;
      float y= pow(-1,i)*isd;
      enbPositionAlloc->Add (Vector (centerPosition.x + x, centerPosition.y + y, 3));
    }

  // Square rotated by 90 degrees
  for (int8_t i = 0; i < 4; ++i)
    {
      x = isd*2 * cos ((2 * M_PI * i) / (4));
      y = isd*2 * sin ((2 * M_PI * i) / (4));
      enbPositionAlloc->Add (Vector (centerPosition.x + x, centerPosition.y + y, 3));
    }

  // Bigger square
  for (int8_t i = 0; i < 4; ++i)
    {
      float x= pow(-1,floor(i/2))*isd*2;
      float y= pow(-1,i)*isd*2;
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

int BSstatus[4]={BsON, BsIdle, BsSleep, BsOFF};

  EnergyHeuristic energyheur;

  switch (heuristicType)
  {
  case 0:
  //random sleeping
  for (double i = 0.0; i < simTime; i = i + indicationPeriodicity)
    {
      for (int j = 0; j < nMmWaveEnbNodes; j++)
        {
          Ptr<MmWaveEnbNetDevice> mmdev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
          Simulator::Schedule (Seconds (i), &EnergyHeuristic::Probability_state, &energyheur, prob_ON,
                               prob_Idle, prob_Sleep, prob_OFF, mmdev->GetCellId (), mmdev);
          NS_LOG_DEBUG ("BS with id " << mmdev->GetCellId () << " has the following state "
                                      << mmdev->GetBsState ());
          std::cout<<"it's working"<<std::endl;
        }
    }
  break;
  
  case 1:
  //static sleeping
  for (double i = 0.0; i < simTime; i = i + indicationPeriodicity - 0.01)
    {
      for (int j = 0; j < nMmWaveEnbNodes; j++)
        {
          Ptr<MmWaveEnbNetDevice> mmdev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
          Simulator::Schedule (Seconds (i), &EnergyHeuristic::CountBestUesSINR, &energyheur, SINRth, mmdev);
        }
      i = i + 0.01; //making sure to execute the next function after the previous one
      Simulator::Schedule (Seconds (i), &EnergyHeuristic::TurnOnBSSinrPos, &energyheur, nMmWaveEnbNodes, mmWaveEnbDevs, "static", BSstatus);
    }
  break;

  case 2:
  //dynamic sleeping
  for (double i = 0.0; i < simTime; i = i + indicationPeriodicity - 0.01)
    {
      for (int j = 0; j < nMmWaveEnbNodes; j++)
        {
          Ptr<MmWaveEnbNetDevice> mmdev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
          Simulator::Schedule (Seconds (i), &EnergyHeuristic::CountBestUesSINR, &energyheur, SINRth, mmdev);
        }
      i = i + 0.01; //making sure to execute the next function after the previous one
      Simulator::Schedule (Seconds (i), &EnergyHeuristic::TurnOnBSSinrPos, &energyheur, nMmWaveEnbNodes, mmWaveEnbDevs, "dynamic", BSstatus);
    }
  break;

  
  default:
  NS_FATAL_ERROR (
          "Heuristic type not recognized, the only possible values are [0,1,2]. Value passed: "
          << heuristicType);
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