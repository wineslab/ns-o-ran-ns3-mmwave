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
#include "ns3/lte-ue-net-device.h"

namespace ns3
{
namespace mmwave
{
/**
 * @brief Class that contains all methods related to heuristics on energy efficiency simulations
 *
 */
class EnergyHeuristic : public Object
{
  public:
    static TypeId GetTypeId(void);

    EnergyHeuristic();

    virtual ~EnergyHeuristic(void);


    /**
     * @brief Method randomly decide one action to apply to the scenario within the list filter_list
     * 
     * @param mmdevArray Array of mmdev of each cell in the scenario
     */
    void RandomAction(std::vector<Ptr<MmWaveEnbNetDevice>> mmdevArray, Ptr<LteEnbNetDevice> ltedev);

    /**
     * @brief Method that decides which mmWave BSs to turn idle, sleep and off based on the smallest
     * time that connected UEs take to reach them
     *
     * @param bsToTurnOn Array that contains a list mmWave BS indexes that are turned on. Each BS is
     * identified with the index value used to store them in mmWaveEnbDevs
     * @param bsOn Number of BSs that are turned on
     * @param bsIdle Number of BSs that are turned idle
     * @param bsSleep Number of BSs that are turned sleep
     * @param bsOff Number of BSs that are turned off
     * @param nMmWaveEnbNodes Number of mmWave BSs in the scenario
     * @param mmWaveEnbDevs Object that contains all the mmWave BSs
     * @return std::vector<std::pair<Ptr<MmWaveEnbNetDevice>, double>> Vector of pairs: the mmWave
     * Bs and the time that takes the closest UE to reach it
     */
    std::vector<std::pair<Ptr<MmWaveEnbNetDevice>, double>> HeuristicDynamic(
        int bsToTurnOn[],
        int bsOn,
        int bsIdle,
        int bsSleep,
        int bsOff,
        int nMmWaveEnbNodes,
        NetDeviceContainer mmWaveEnbDevs);
    /**
     * @brief Method that decides which mmWave BSs to turn idle, sleep and off based on the distance
     * of the closest ue
     *
     * @param bsToTurnOn Array that contains a list mmWave BS indexes that are turned on. Each BS is
     * identified with the index used to store them in mmWaveEnbDevs
     * @param bsOn Number of BSs that are turned on
     * @param bsIdle Number of BSs that are turned idle
     * @param bsSleep Number of BSs that are turned sleep
     * @param bsOff Number of BSs that are turned off
     * @param nMmWaveEnbNodes Number of mmWave BSs in the scenario
     * @param mmWaveEnbDevs Object that contains all the mmWave BSs
     * @return std::vector<std::pair<Ptr<MmWaveEnbNetDevice>, double>> Vector of pairs: the mmWave
     * Bs and the distance of the closest UE
     */
    std::vector<std::pair<Ptr<MmWaveEnbNetDevice>, double>> HeuristicStatic(
        int bsToTurnOn[],
        int bsOn,
        int bsIdle,
        int bsSleep,
        int bsOff,
        int nMmWaveEnbNodes,
        NetDeviceContainer mmWaveEnbDevs);
    /**
     * @brief Turn on BSs with the highest number of connected ues with a high SINR
     *
     * @param nMmWaveEnbNodes Number of mmWave BSs in the scenario
     * @param mmWaveEnbDevs Object that contains all the mmWave BSs
     * @param heuristic Type of heuristic, Dynamic or Static
     * @param BsStatus Array of 4 elements: each cell of the array contains the number of BSs we want to:
     * BsStatus[0]=turn on, BsStatus[1]=turn idle, BsStatus[2]=turn sleep, BsStatus[3]=turn off
     */
    void TurnOnBsSinrPos(uint8_t nMmWaveEnbNodes,
                         NetDeviceContainer mmWaveEnbDevs,
                         std::string heuristic,
                         int BsStatus[],
                         Ptr<LteEnbNetDevice> ltedev);
    /**
     * @brief Turn on, idle, sleep and off Bss with a certain probability
     *
     * @param p1 Probability to turn BS on
     * @param p2 Probability to turn BS idle
     * @param p3 Probability to turn BS sleep
     * @param p4 Probability to turn BS off
     * @param nodeId id of the node to apply the action
     * @param mmDev
     */
    void ProbabilityState(double p1,
                          double p2,
                          double p3,
                          double p4,
                          Ptr<MmWaveEnbNetDevice> mmDev,
                          Ptr<LteEnbNetDevice> ltedev);
    /**
     * @brief Count the number of UEs with SINR higher than threshold
     *
     * @param sinrTh SINR threshold to define if a UE has a high SINR value
     * @param mmDev
     */
    void CountBestUesSinr(double sinrTh, Ptr<MmWaveEnbNetDevice> mmDev);
};

} // namespace mmwave
} // namespace ns3