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
#include "mavenir-heuristic.h"
#include <vector>
#include <iostream>
#include <sstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MavenirHeuristic");

namespace mmwave {

//NS_OBJECT_ENSURE_REGISTERED (MavenirHeuristic);

MavenirHeuristic::MavenirHeuristic ()
{
  NS_LOG_FUNCTION (this);
}


MavenirHeuristic::~MavenirHeuristic ()
{
  NS_LOG_FUNCTION (this);
}

TypeId MavenirHeuristic::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::MavenirHeuristic")
    .SetParent<Object>()
    .AddConstructor<MavenirHeuristic>();
  return tid;
}

//transform the input string of cluster into a vector of vector of values, substituting the id with their mmdev value
std::vector<std::vector<Ptr<MmWaveEnbNetDevice>>> MavenirHeuristic::ReadClusters( std::string clustersString, uint8_t nMmWaveEnbNodes, NetDeviceContainer mmWaveEnbDevs){

  //int n_clusters=2;
  //std::string input = "[[5,6,7],[1],[2,3,4],[]]";
  NS_LOG_DEBUG ("Input cluster as string: " << clustersString);
  std::vector<std::vector<Ptr<MmWaveEnbNetDevice>>> clusters;
  std::stringstream ss(clustersString);
  std::string itemCell;
  
  while (getline(ss, itemCell, ']')) {
    if (itemCell.empty()) {
      continue;
    }
    itemCell = itemCell.substr(2);
    std::vector<Ptr<MmWaveEnbNetDevice>> cluster;
    std::stringstream clusterStream(itemCell);
    std::string clusterItem;
    while (getline(clusterStream, clusterItem, ',')) {
      if (!clusterItem.empty() && isdigit(clusterItem[0])) {
        try {
          int num = std::stoi(clusterItem);

          for (int j = 0; j < nMmWaveEnbNodes; j++)
              {
                Ptr<MmWaveEnbNetDevice> mmdev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
                if(num==mmdev->GetCellId()){
                  cluster.push_back(mmdev);
                }
              }
        } catch (const std::invalid_argument &e) {
          NS_FATAL_ERROR ( "Error: " << clusterItem << " is not a valid integer.");
        }
      }
    }
    clusters.push_back(cluster);
  }
  
  // NS_LOG_DEBUG ("Cluster cluster");
  // for (const auto &cluster : clusters) {
  //   for (const auto &element : cluster) {
  //       NS_LOG_DEBUG (element << " ");
  //   }
  //   NS_LOG_DEBUG ("/n");
  // }

  for (uint i = 0; i < clusters.size(); i++)
  {
    for (uint j = 0; j < clusters[i].size(); j++)
    {
        NS_LOG_DEBUG( clusters[i][j]->GetCellId());
    }
  NS_LOG_DEBUG ("/n");
  }

  return clusters;
}


void MavenirHeuristic::MavenirHeur(uint8_t nMmWaveEnbNodes, NetDeviceContainer mmWaveEnbDevs, Ptr<LteEnbNetDevice> ltedev, std::vector<std::vector<Ptr<MmWaveEnbNetDevice>>> clusters, double eekpiTh,double avgWeightedEekpiTh){
  //every time periodicity
  Ptr<MmWaveEnbNetDevice> smallestMmDev; //leaving it empty is not a problem since in the worst case is not used
  Ptr<LteEnbRrc> m_rrc = ltedev->GetRrc ();
  double smallestEekpi = eekpiTh; // I set it higher than the c'threshold so in extreme cases it doesn't pass the next if control to turn off the Cell
  for (int j = 0; j < nMmWaveEnbNodes; j++) //for every cell
  {
    //get the mmwave BS
    Ptr<MmWaveEnbNetDevice> mmDev = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
    if(mmDev->GetBsState()==1){//if the cell is turned ON
      NS_LOG_DEBUG ("EEKPI1 BS ID " << mmDev->GetCellId()<<" and state (turned ON) "<< mmDev->GetBsState());
      //compute eekpi and get the smallest one c'
      double txPowerWatts=pow(10,mmDev->GetPhy()->GetTxPower()/10)/1000;
      NS_LOG_DEBUG ("macVolumeCellSpecific for EEKPI1 "<< mmDev->GetmacVolumeCellSpecific());
      NS_LOG_DEBUG ("macPduCellSpecific for EEKPI1 "<< mmDev->GetmacPduCellSpecific());
      NS_LOG_DEBUG ("txPowerWatts for EEKPI1 "<< txPowerWatts);
      double eekpi = eekpiTh;
      if((mmDev->GetmacPduCellSpecific()*txPowerWatts)!=0 ){
        eekpi= (double)mmDev->GetmacVolumeCellSpecific()/(mmDev->GetmacPduCellSpecific()*txPowerWatts);
        NS_LOG_DEBUG ("EEKPI1 "<< eekpi << " for cell "<< mmDev->GetCellId());
      }
      else{
        NS_LOG_DEBUG ("EEKPI1 "<< eekpi << " because macPduCellSpecific*txPowerWatts=0");
      }
      //mmDev->Seteekpi(eekpi); //save the eekpi for each BS to use it on the second part of the fuction (do we really need it??)
      if(eekpi<smallestEekpi){
        smallestEekpi= eekpi;
        smallestMmDev= mmDev;
      }
    }
  }
  NS_LOG_DEBUG ("Smallest EEKPI1 "<< smallestEekpi);
  //if c'< threshold value (eekpiTh) -> turn off BS (energy saving)
  if(smallestEekpi<eekpiTh){
    smallestMmDev->TurnOff(smallestMmDev->GetCellId(), m_rrc);
    smallestMmDev->SetturnOffTime(Simulator::Now().GetSeconds());
    NS_LOG_DEBUG ("Turn off the smallest EEKPI1 BS ID "<< smallestMmDev->GetCellId());
  }

  double smallestEekpi2 = avgWeightedEekpiTh; // I set it higher than the c' threshold so in extreme cases it doesn't pass the next if control to turn on the Cell
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
              NS_LOG_DEBUG ("EEKPI2 Cell ID " << mmDev->GetCellId() << " has the following neighbor turned ON " << neighMmDev->GetCellId());
              //calculate eekpi2 new formula for each neighbour
              double txPowerWatts=pow(10,neighMmDev->GetPhy()->GetTxPower()/10)/1000;
              NS_LOG_DEBUG ("macVolumeCellSpecific for EEKPI2 "<< neighMmDev->GetmacVolumeCellSpecific());
              NS_LOG_DEBUG ("macPduCellSpecific for EEKPI2 "<< neighMmDev->GetmacPduCellSpecific());
              NS_LOG_DEBUG ("txPowerWatts for EEKPI2 "<< txPowerWatts);
              double eekpi = 0;
              if((neighMmDev->GetmacPduCellSpecific()*txPowerWatts)!=0 ){
                eekpi= (double)neighMmDev->GetmacVolumeCellSpecific()/(neighMmDev->GetmacPduCellSpecific()*txPowerWatts);
                cellToCount++;
              }
              else{
                NS_LOG_DEBUG ("In EEKPI2 macPduCellSpecific*txPowerWatts=0");
              }
              //eekpi is about the neighbour cell
              double weightedEekpi2=eekpi* 1.0* exp(-0.1*(Simulator::Now().GetSeconds() - mmDev->GetturnOffTime()) ); 
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
  //obtain all the smallest eekpi2 avg values and if eekpi2 < threshold value ( avgWeightedEekpiTh ) -> turn on BS
  if(smallestEekpi2<avgWeightedEekpiTh){
    smallestMmDev2->TurnOn (smallestMmDev2->GetCellId (), m_rrc);
    NS_LOG_DEBUG ("Turn on the smallest EEKPI2 BS ID "<< smallestMmDev2->GetCellId());
  }

}

}

} // namespace ns3