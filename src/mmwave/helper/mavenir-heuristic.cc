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

#include "mavenir-heuristic.h"

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/lte-ue-net-device.h"
#include "ns3/mmwave-helper.h"
#include "ns3/network-module.h"

#include <iostream>
#include <sstream>
#include <vector>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MavenirHeuristic");

namespace mmwave
{

// NS_OBJECT_ENSURE_REGISTERED (MavHeurParameters);

MavHeurParameters::MavHeurParameters()
{
    NS_LOG_FUNCTION(this);
}

MavHeurParameters::~MavHeurParameters()
{
    NS_LOG_FUNCTION(this);
}

TypeId
MavHeurParameters::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::MavHeurParameters").SetParent<Object>().AddConstructor<MavHeurParameters>();
    return tid;
}

MavHeurParameters::MavHeurParameters(double a, double b, int c, double d, double e)
{
    m_eekpiTh = a;
    m_avgWeightedEekpiTh = b;
    m_kCells = c;
    m_eekpiB = d;
    m_eekpiLambda = e;
};

double
MavHeurParameters::GetEekpiTh()
{
    return m_eekpiTh;
}

double
MavHeurParameters::GetAvgWeightedEekpiTh()
{
    return m_avgWeightedEekpiTh;
}

int
MavHeurParameters::GetKCells()
{
    return m_kCells;
}

double
MavHeurParameters::GetEekpiB()
{
    return m_eekpiB;
}

double
MavHeurParameters::GetEekpiLambda()
{
    return m_eekpiLambda;
}

// NS_OBJECT_ENSURE_REGISTERED (MavenirHeuristic);

MavenirHeuristic::MavenirHeuristic()
{
    NS_LOG_FUNCTION(this);
}

MavenirHeuristic::~MavenirHeuristic()
{
    NS_LOG_FUNCTION(this);
}

TypeId
MavenirHeuristic::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::MavenirHeuristic").SetParent<Object>().AddConstructor<MavenirHeuristic>();
    return tid;
}

// transform the input string of cluster into a vector of vector of values, substituting the id with
// their mmdev value
std::vector<std::vector<Ptr<MmWaveEnbNetDevice>>>
MavenirHeuristic::ReadClusters(std::string clustersString,
                               uint8_t nMmWaveEnbNodes,
                               NetDeviceContainer mmWaveEnbDevs)
{
    NS_LOG_DEBUG("Input cluster as string: " << clustersString);
    std::vector<std::vector<Ptr<MmWaveEnbNetDevice>>> clusters;
    std::stringstream ss(clustersString);
    std::string itemCell;

    while (getline(ss, itemCell, ']'))
    {
        if (itemCell.empty())
        {
            continue;
        }
        itemCell = itemCell.substr(2);
        std::vector<Ptr<MmWaveEnbNetDevice>> cluster;
        std::stringstream clusterStream(itemCell);
        std::string clusterItem;
        while (getline(clusterStream, clusterItem, ','))
        {
            if (!clusterItem.empty() && isdigit(clusterItem[0]))
            {
                try
                {
                    int num = std::stoi(clusterItem);
                    for (int j = 0; j < nMmWaveEnbNodes; j++)
                    {
                        Ptr<MmWaveEnbNetDevice> mmdev =
                            DynamicCast<MmWaveEnbNetDevice>(mmWaveEnbDevs.Get(j));
                        if (num == mmdev->GetCellId())
                        {
                            cluster.push_back(mmdev);
                        }
                    }
                }
                catch (const std::invalid_argument& e)
                {
                    NS_FATAL_ERROR("Error: " << clusterItem << " is not a valid integer.");
                }
            }
        }
        clusters.push_back(cluster);
    }
    for (uint i = 0; i < clusters.size(); i++)
    {
        for (uint j = 0; j < clusters[i].size(); j++)
        {
            NS_LOG_DEBUG(clusters[i][j]->GetCellId());
        }
        NS_LOG_DEBUG("/n");
    }

    return clusters;
}

void
MavenirHeuristic::MavenirHeur(uint8_t nMmWaveEnbNodes,
                              NetDeviceContainer mmWaveEnbDevs,
                              Ptr<LteEnbNetDevice> ltedev,
                              std::vector<std::vector<Ptr<MmWaveEnbNetDevice>>> clusters,
                              Ptr<MavHeurParameters> mavenirHeurPar)
{
    // every time periodicity
    // get attributes from the object
    double eekpiTh = mavenirHeurPar->GetEekpiTh();
    double avgWeightedEekpiTh = mavenirHeurPar->GetAvgWeightedEekpiTh();
    int kCells = mavenirHeurPar->GetKCells();
    double eekpiB = mavenirHeurPar->GetEekpiB();
    double eekpiLambda = mavenirHeurPar->GetEekpiLambda();
    // initialize basic parameters
    Ptr<LteEnbRrc> m_rrc = ltedev->GetRrc();
    Ptr<MmWaveEnbNetDevice> smallestMmDev[kCells] =
        {}; // leaving it empty is not a problem since in the worst case is not used
    double smallestEekpi[kCells] = {};
    for (int i = 0; i < kCells; i++)
    {
        smallestEekpi[i] = eekpiTh; // I initialize it equal to the c' threshold so in extreme cases
                                    // it doesn't pass the next IF control to turn off the Cell
    }
    for (int j = 0; j < nMmWaveEnbNodes; j++) // for every cell
    {
        // get the mmwave BS
        Ptr<MmWaveEnbNetDevice> mmDev = DynamicCast<MmWaveEnbNetDevice>(mmWaveEnbDevs.Get(j));
        if (mmDev->GetBsState() == 1)
        { // if the cell is turned ON
            NS_LOG_DEBUG("EEKPI1 BS ID " << mmDev->GetCellId() << " and state (turned ON) "
                                         << mmDev->GetBsState());
            // compute eekpi and get the smallest one c'
            double txPowerWatts = pow(10, mmDev->GetPhy()->GetTxPower() / 10) / 1000;
            NS_LOG_DEBUG("macVolumeCellSpecific for EEKPI1 " << mmDev->GetMacVolumeCellSpecific());
            NS_LOG_DEBUG("macPduCellSpecific for EEKPI1 " << mmDev->GetMacPduCellSpecific());
            NS_LOG_DEBUG("txPowerWatts for EEKPI1 " << txPowerWatts);
            double eekpi = eekpiTh;
            if ((mmDev->GetMacPduCellSpecific() * txPowerWatts) != 0)
            {
                eekpi = (double)mmDev->GetMacVolumeCellSpecific() /
                        (mmDev->GetMacPduCellSpecific() * txPowerWatts);
                NS_LOG_DEBUG("EEKPI1 " << eekpi << " for cell " << mmDev->GetCellId());
            }
            else
            {
                NS_LOG_DEBUG("EEKPI1 " << eekpi << " because macPduCellSpecific*txPowerWatts=0");
            }
            // we want to keep n (kCells variable) smallest eekpi values inside the array
            // (smallestEekpi) find the highest value in array smallestEekpi[]
            double highestValueEekpiArray = 0;
            int indexhighestValueEekpiArray = 0;
            for (int i = 0; i < kCells; i++)
            {
                if (smallestEekpi[i] > highestValueEekpiArray)
                {
                    highestValueEekpiArray = smallestEekpi[i];
                    indexhighestValueEekpiArray = i;
                }
            }
            // check if the actual eekpi is smaller than the bigger eekpi saved in the array and in
            // case substitute it
            if (eekpi < highestValueEekpiArray)
            {
                smallestEekpi[indexhighestValueEekpiArray] = eekpi;
                smallestMmDev[indexhighestValueEekpiArray] = mmDev;
            }
        }
    }
    NS_LOG_DEBUG("The smallest " << kCells << " EEKPI1 values are ");
    for (int i = 0; i < kCells; i++)
    {
        NS_LOG_DEBUG("   " << smallestEekpi[i]);
    }

    for (int i = 0; i < kCells; i++)
    {
        // if c'< threshold value (eekpiTh) -> turn off BS (energy saving)
        if (smallestEekpi[i] < eekpiTh)
        {
            smallestMmDev[i]->TurnOff(smallestMmDev[i]->GetCellId(), m_rrc);
            smallestMmDev[i]->SetTurnOffTime(Simulator::Now().GetSeconds());
            NS_LOG_DEBUG("Turn off the smallest EEKPI1 BS ID " << smallestMmDev[i]->GetCellId());
        }
    }

    // SECOND part of the heuristic

    double smallestEekpi2[kCells] = {};
    for (int i = 0; i < kCells; i++)
    {
        smallestEekpi2[i] =
            avgWeightedEekpiTh; // I set it higher than the c' threshold so in extreme cases it
                                // doesn't pass the next IF control to turn on the Cell
    }
    Ptr<MmWaveEnbNetDevice> smallestMmDev2[kCells] =
        {}; // leaveing it empty is not a problem since in the worst case is not used
    // for every cell turned off (except the c')
    for (int j = 0; j < nMmWaveEnbNodes; j++) // for every cell
    {
        // get the mmwave BS
        Ptr<MmWaveEnbNetDevice> mmDev = DynamicCast<MmWaveEnbNetDevice>(mmWaveEnbDevs.Get(j));
        // check if the cell is just turned OFF, if yes skip it
        bool justTurnedOff = false;
        for (int i = 0; i < kCells; i++)
        {
            if (smallestMmDev[i] == mmDev)
            {
                justTurnedOff = true;
                break; // if found (just turned off) exit
            }
        }
        if (justTurnedOff == false && mmDev->GetBsState() == 0)
        { // if the cell is turned OFF (except the c', so the one just turned off)
            NS_LOG_DEBUG("EEKPI2 BS ID (except the one just turned OFF) "
                         << mmDev->GetCellId() << " and state (turned OFF) "
                         << mmDev->GetBsState());

            for (uint i1 = 0; i1 < clusters.size(); i1++)
            {
                // check if the subject cell is inside this cluster
                if (std::find(clusters[i1].begin(), clusters[i1].end(), mmDev) !=
                    clusters[i1].end())
                { // if the cell is here, perform the operation for all the neighbors
                    double sumWeightedEekpi2 = 0;
                    int cellToCount =
                        0; // number of cells neighbors turned ON and prbUtilizationDl!=0 (important
                           // variable to compute the eekpi formula)
                    // calculate the eekpi
                    for (uint i2 = 0; i2 < clusters[i1].size(); i2++)
                    {
                        Ptr<MmWaveEnbNetDevice> neighMmDev = clusters[i1][i2];
                        // skip the mmdev cell (subject cell) as neighbor
                        // moreover the neighbor cell has to be turned ON
                        if (neighMmDev != mmDev && neighMmDev->GetBsState() == 1)
                        {
                            NS_LOG_DEBUG("EEKPI2 Cell ID "
                                         << mmDev->GetCellId()
                                         << " has the following neighbor turned ON "
                                         << neighMmDev->GetCellId());
                            // calculate eekpi2 new formula for each neighbour
                            double txPowerWatts =
                                pow(10, neighMmDev->GetPhy()->GetTxPower() / 10) / 1000;
                            NS_LOG_DEBUG("macVolumeCellSpecific for EEKPI2 "
                                         << neighMmDev->GetMacVolumeCellSpecific());
                            NS_LOG_DEBUG("macPduCellSpecific for EEKPI2 "
                                         << neighMmDev->GetMacPduCellSpecific());
                            NS_LOG_DEBUG("txPowerWatts for EEKPI2 " << txPowerWatts);
                            double eekpi = 0;
                            if ((neighMmDev->GetMacPduCellSpecific() * txPowerWatts) != 0)
                            {
                                eekpi = (double)neighMmDev->GetMacVolumeCellSpecific() /
                                        (neighMmDev->GetMacPduCellSpecific() * txPowerWatts);
                                cellToCount++;
                            }
                            else
                            {
                                NS_LOG_DEBUG("In EEKPI2 macPduCellSpecific*txPowerWatts=0");
                            }
                            // eekpi is about the neighbour cell
                            double weightedEekpi2 =
                                eekpi * eekpiB *
                                exp(-eekpiLambda *
                                    (Simulator::Now().GetSeconds() - mmDev->GetTurnOffTime()));
                            sumWeightedEekpi2 = sumWeightedEekpi2 + weightedEekpi2;
                        }
                    }
                    // do the average and save it to the subject cell in variable "j" (the one
                    // turned off)
                    double avgEekpi2 = avgWeightedEekpiTh;
                    if (cellToCount == 0)
                    {
                        NS_LOG_DEBUG("AVG weighted EEKPI2 " << avgEekpi2
                                                            << " because no neighbours ON");
                    }
                    else
                    {
                        avgEekpi2 = sumWeightedEekpi2 / cellToCount;
                        NS_LOG_DEBUG("AVG weighted EEKPI2 " << avgEekpi2 << " for cell "
                                                            << mmDev->GetCellId());
                    }

                    // find the highest value in array smallestEekpi2[]
                    double highestValueEekpi2Array = 0;
                    int indexhighestValueEekpi2Array = 0;
                    for (int i = 0; i < kCells; i++)
                    {
                        if (smallestEekpi2[i] > highestValueEekpi2Array)
                        {
                            highestValueEekpi2Array = smallestEekpi2[i];
                            indexhighestValueEekpi2Array = i;
                        }
                    }
                    // check if the actual eekpi2 is smaller than the bigger eekpi2 saved in the
                    // array and in case substitute it
                    if (avgEekpi2 < highestValueEekpi2Array)
                    {
                        smallestEekpi2[indexhighestValueEekpi2Array] = avgEekpi2;
                        smallestMmDev2[indexhighestValueEekpi2Array] = mmDev;
                    }
                }
                // else, move to the next cluster
            }
        }
    }
    NS_LOG_DEBUG("The smallest " << kCells << " AVG weighted EEKPI2 are ");
    for (int i = 0; i < kCells; i++)
    {
        NS_LOG_DEBUG("   " << smallestEekpi2[i]);
    }
    for (int i = 0; i < kCells; i++)
    {
        // obtain all the smallest eekpi2 avg values and if eekpi2 < threshold value (
        // avgWeightedEekpiTh ) -> turn on BS
        if (smallestEekpi2[i] < avgWeightedEekpiTh)
        {
            smallestMmDev2[i]->TurnOn(smallestMmDev2[i]->GetCellId(), m_rrc);
            NS_LOG_DEBUG("Turn on the smallest EEKPI2 BS ID " << smallestMmDev2[i]->GetCellId());
        }
    }
}

} // namespace mmwave

} // namespace ns3