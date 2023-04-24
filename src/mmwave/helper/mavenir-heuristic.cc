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
    if (m_energyHeuristicFile.is_open()){
        m_energyHeuristicFile.close();
    }
}

TypeId
MavenirHeuristic::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::MavenirHeuristic")
            .SetParent<Object>()
            .AddConstructor<MavenirHeuristic>()
            .AddAttribute("EnergyHeuristicFilename",
                          "Name of the file where the energy heuristic information will be "
                          "periodically written.",
                          StringValue("EnergyHeuristic.txt"),
                          MakeStringAccessor(&MavenirHeuristic::SetEnergyHeuristicFilename),
                          MakeStringChecker());
    return tid;
}

void
MavenirHeuristic::MavHeuristicTrace(std::string trace, Ptr<LteEnbNetDevice> ltedev)
{
    // write to file
    if (!m_energyHeuristicFile.is_open())
    {
        NS_LOG_DEBUG(GetEnergyHeuristicFilename().c_str());
        m_energyHeuristicFile.open(GetEnergyHeuristicFilename().c_str(),
                                   std::ofstream::out | std::ofstream::trunc);
        NS_LOG_LOGIC("File opened");
        m_energyHeuristicFile << "Timestamp"
                              << " "
                              << "UNIX"
                              << " "
                              << "Info" << std::endl;
    }
    uint64_t timestamp = ltedev->GetStartTime() + Simulator::Now().GetMilliSeconds();
    m_energyHeuristicFile << Simulator::Now().GetSeconds() << " " << timestamp << " " << trace
                          << std::endl;
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


    return clusters;
}

std::string
MavenirHeuristic::GetEnergyHeuristicFilename()
{
    return m_energyHeuristicFilename;
}

void
MavenirHeuristic::SetEnergyHeuristicFilename(std::string filename)
{
    m_energyHeuristicFilename = filename;
}

void
MavenirHeuristic::MavenirHeur(uint8_t nMmWaveEnbNodes,
                              NetDeviceContainer mmWaveEnbDevs,
                              Ptr<LteEnbNetDevice> ltedev,
                              std::vector<std::vector<Ptr<MmWaveEnbNetDevice>>> clusters,
                              Ptr<MavHeurParameters> mavenirHeurPar)
{
    std::string trace;
    // Print input cluster
    for (uint i = 0; i < clusters.size(); i++)
    {
        for (uint j = 0; j < clusters[i].size(); j++)
        {
            trace = std::to_string(clusters[i][j]->GetCellId());
            MavHeuristicTrace(trace, ltedev);
            NS_LOG_DEBUG(trace);
        }
        trace = " ";
        MavHeuristicTrace(trace, ltedev);
        NS_LOG_DEBUG(trace);
    }
    // Every time periodicity
    // Get attributes from the object
    double eekpiTh = mavenirHeurPar->GetEekpiTh();
    double avgWeightedEekpiTh = mavenirHeurPar->GetAvgWeightedEekpiTh();
    int kCells = mavenirHeurPar->GetKCells();
    double eekpiB = mavenirHeurPar->GetEekpiB();
    double eekpiLambda = mavenirHeurPar->GetEekpiLambda();
    // Initialize basic parameters
    Ptr<LteEnbRrc> m_rrc = ltedev->GetRrc();
    Ptr<MmWaveEnbNetDevice> smallestMmDev[kCells] = {}; // Leaving it empty is not a problem since in the worst case is not used
    double smallestEekpi[kCells] = {};
    for (int i = 0; i < kCells; i++)
    {
        smallestEekpi[i] = eekpiTh; // I initialize it equal to the c' threshold so in extreme cases
                                    // it doesn't pass the next IF control to turn off the Cell
    }
    for (int j = 0; j < nMmWaveEnbNodes; j++) // for every cell
    {
        // Get the mmwave BS
        Ptr<MmWaveEnbNetDevice> mmDev = DynamicCast<MmWaveEnbNetDevice>(mmWaveEnbDevs.Get(j));
        if (mmDev->GetBsState() == 1)
        { // If the cell is turned ON
            trace = "EEKPI1 BS ID " + std::to_string(mmDev->GetCellId()) +
                    " and state (turned ON) " + std::to_string(mmDev->GetBsState());
            MavHeuristicTrace(trace, ltedev);
            NS_LOG_DEBUG(trace);
            // Compute eekpi and get the smallest one c'
            double txPowerWatts = pow(10, mmDev->GetPhy()->GetTxPower() / 10) / 1000;
            trace = "macVolumeCellSpecific for EEKPI1 " +
                    std::to_string(mmDev->GetMacVolumeCellSpecific());
            MavHeuristicTrace(trace, ltedev);
            NS_LOG_DEBUG(trace);
            trace =
                "macPduCellSpecific for EEKPI1 " + std::to_string(mmDev->GetMacPduCellSpecific());
            MavHeuristicTrace(trace, ltedev);
            NS_LOG_DEBUG(trace);
            trace = "txPowerWatts for EEKPI1 " + std::to_string(txPowerWatts);
            MavHeuristicTrace(trace, ltedev);
            NS_LOG_DEBUG(trace);
            double eekpi = eekpiTh;
            if ((mmDev->GetMacPduCellSpecific() * txPowerWatts) != 0)
            {
                eekpi = (double)mmDev->GetMacVolumeCellSpecific() /
                        (mmDev->GetMacPduCellSpecific() * txPowerWatts);
                trace = "EEKPI1 " + std::to_string(eekpi) + " for cell " +
                        std::to_string(mmDev->GetCellId());
                MavHeuristicTrace(trace, ltedev);
                NS_LOG_DEBUG(trace);
            }
            else
            {
                trace = "EEKPI1 " + std::to_string(eekpi) +
                        " because macPduCellSpecific*txPowerWatts=0";
                MavHeuristicTrace(trace, ltedev);
                NS_LOG_DEBUG(trace);
            }
            // We want to keep n (kCells variable) smallest eekpi values inside the array
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
            // Check if the actual eekpi is smaller than the bigger eekpi saved in the array and in
            // case substitute it
            if (eekpi < highestValueEekpiArray)
            {
                smallestEekpi[indexhighestValueEekpiArray] = eekpi;
                smallestMmDev[indexhighestValueEekpiArray] = mmDev;
            }
        }
    }
    trace = "The smallest " + std::to_string(kCells) + " EEKPI1 values are ";
    MavHeuristicTrace(trace, ltedev);
    NS_LOG_DEBUG(trace);
    for (int i = 0; i < kCells; i++)
    {
        trace = "   " + std::to_string(smallestEekpi[i]);
        MavHeuristicTrace(trace, ltedev);
        NS_LOG_DEBUG(trace);
    }

    for (int i = 0; i < kCells; i++)
    {
        // If c'< threshold value (eekpiTh) -> turn off BS (energy saving)
        if (smallestEekpi[i] < eekpiTh)
        {
            smallestMmDev[i]->TurnOff(smallestMmDev[i]->GetCellId(), m_rrc);
            smallestMmDev[i]->SetTurnOffTime(Simulator::Now().GetSeconds());
            trace = "Turn off the smallest EEKPI1 BS ID " +
                    std::to_string(smallestMmDev[i]->GetCellId());
            MavHeuristicTrace(trace, ltedev);
            NS_LOG_DEBUG(trace);
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
    Ptr<MmWaveEnbNetDevice> smallestMmDev2[kCells] = {}; // Leaveing it empty is not a problem since in the worst case is not used
    // For every cell turned off (except the c')
    for (int j = 0; j < nMmWaveEnbNodes; j++) // for every cell
    {
        // Get the mmwave BS
        Ptr<MmWaveEnbNetDevice> mmDev = DynamicCast<MmWaveEnbNetDevice>(mmWaveEnbDevs.Get(j));
        // Check if the cell is just turned OFF, if yes skip it
        bool justTurnedOff = false;
        for (int i = 0; i < kCells; i++)
        {
            if (smallestMmDev[i] == mmDev)
            {
                justTurnedOff = true;
                break; // If found (just turned off) exit
            }
        }
        if (justTurnedOff == false && mmDev->GetBsState() == 0)
        { // If the cell is turned OFF (except the c', so the one just turned off)
            trace = "EEKPI2 BS ID (except the one just turned OFF) " +
                    std::to_string(mmDev->GetCellId()) + " and state (turned OFF) " +
                    std::to_string(mmDev->GetBsState());
            MavHeuristicTrace(trace, ltedev);
            NS_LOG_DEBUG(trace);

            for (uint i1 = 0; i1 < clusters.size(); i1++)
            {
                // Check if the subject cell is inside this cluster
                if (std::find(clusters[i1].begin(), clusters[i1].end(), mmDev) !=
                    clusters[i1].end())
                { // If the cell is here, perform the operation for all the neighbors
                    double sumWeightedEekpi2 = 0;
                    int cellToCount =
                        0; // Number of cells neighbors turned ON and prbUtilizationDl!=0 (important
                           // Variable to compute the eekpi formula)
                    // Calculate the eekpi
                    for (uint i2 = 0; i2 < clusters[i1].size(); i2++)
                    {
                        Ptr<MmWaveEnbNetDevice> neighMmDev = clusters[i1][i2];
                        // Skip the mmdev cell (subject cell) as neighbor
                        // Moreover the neighbor cell has to be turned ON
                        if (neighMmDev != mmDev && neighMmDev->GetBsState() == 1)
                        {
                            trace = "EEKPI2 Cell ID " + std::to_string(mmDev->GetCellId()) +
                                    " has the following neighbor turned ON " +
                                    std::to_string(neighMmDev->GetCellId());
                            MavHeuristicTrace(trace, ltedev);
                            NS_LOG_DEBUG(trace);
                            // Calculate eekpi2 new formula for each neighbour
                            double txPowerWatts =
                                pow(10, neighMmDev->GetPhy()->GetTxPower() / 10) / 1000;
                            trace = "macVolumeCellSpecific for EEKPI2 " +
                                    std::to_string(neighMmDev->GetMacVolumeCellSpecific());
                            MavHeuristicTrace(trace, ltedev);
                            NS_LOG_DEBUG(trace);
                            trace = "macPduCellSpecific for EEKPI2 " +
                                    std::to_string(neighMmDev->GetMacPduCellSpecific());
                            MavHeuristicTrace(trace, ltedev);
                            NS_LOG_DEBUG(trace);
                            trace = "txPowerWatts for EEKPI2 " + std::to_string(txPowerWatts);
                            MavHeuristicTrace(trace, ltedev);
                            NS_LOG_DEBUG(trace);
                            double eekpi = 0;
                            if ((neighMmDev->GetMacPduCellSpecific() * txPowerWatts) != 0)
                            {
                                eekpi = (double)neighMmDev->GetMacVolumeCellSpecific() /
                                        (neighMmDev->GetMacPduCellSpecific() * txPowerWatts);
                                cellToCount++;
                            }
                            else
                            {
                                trace = "In EEKPI2 macPduCellSpecific*txPowerWatts=0";
                                MavHeuristicTrace(trace, ltedev);
                                NS_LOG_DEBUG(trace);
                            }
                            // eekpi is about the neighbour cell
                            double weightedEekpi2 =
                                eekpi * eekpiB *
                                exp(-eekpiLambda *
                                    (Simulator::Now().GetSeconds() - mmDev->GetTurnOffTime()));
                            sumWeightedEekpi2 = sumWeightedEekpi2 + weightedEekpi2;
                        }
                    }
                    // Do the average and save it to the subject cell in variable "j" (the one
                    // turned off)
                    double avgEekpi2 = avgWeightedEekpiTh;
                    if (cellToCount == 0)
                    {
                        trace = "AVG weighted EEKPI2 " + std::to_string(avgEekpi2) +
                                " because no neighbours ON";
                        MavHeuristicTrace(trace, ltedev);
                        NS_LOG_DEBUG(trace);
                    }
                    else
                    {
                        avgEekpi2 = sumWeightedEekpi2 / cellToCount;
                        trace = "AVG weighted EEKPI2 " + std::to_string(avgEekpi2) + " for cell " +
                                std::to_string(mmDev->GetCellId());
                        MavHeuristicTrace(trace, ltedev);
                        NS_LOG_DEBUG(trace);
                    }

                    // Find the highest value in array smallestEekpi2[]
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
                    // Check if the actual eekpi2 is smaller than the bigger eekpi2 saved in the
                    // Array and in case substitute it
                    if (avgEekpi2 < highestValueEekpi2Array)
                    {
                        smallestEekpi2[indexhighestValueEekpi2Array] = avgEekpi2;
                        smallestMmDev2[indexhighestValueEekpi2Array] = mmDev;
                    }
                }
                // Else, move to the next cluster
            }
        }
    }
    trace = "The smallest " + std::to_string(kCells) + " AVG weighted EEKPI2 are ";
    MavHeuristicTrace(trace, ltedev);
    NS_LOG_DEBUG(trace);
    for (int i = 0; i < kCells; i++)
    {
        trace = "   " + std::to_string(smallestEekpi2[i]);
        MavHeuristicTrace(trace, ltedev);
        NS_LOG_DEBUG(trace);
    }
    for (int i = 0; i < kCells; i++)
    {
        // Obtain all the smallest eekpi2 avg values and if eekpi2 < threshold value (
        // avgWeightedEekpiTh ) -> turn on BS
        if (smallestEekpi2[i] < avgWeightedEekpiTh)
        {
            smallestMmDev2[i]->TurnOn(smallestMmDev2[i]->GetCellId(), m_rrc);
            trace = "Turn on the smallest EEKPI2 BS ID " +
                    std::to_string(smallestMmDev2[i]->GetCellId());
            MavHeuristicTrace(trace, ltedev);
            NS_LOG_DEBUG(trace);
        }
    }
}

} // namespace mmwave

} // namespace ns3