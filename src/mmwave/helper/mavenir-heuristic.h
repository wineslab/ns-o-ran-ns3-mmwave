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
#include "ns3/mmwave-enb-net-device.h"
#include "ns3/net-device-container.h"

namespace ns3
{
namespace mmwave
{

class MavHeurParameters : public Object
{
  public:
    static TypeId GetTypeId(void);
    MavHeurParameters();
    MavHeurParameters(double a, double b, int c, double d, double e);
    virtual ~MavHeurParameters(void);
    double GetEekpiTh();
    double GetAvgWeightedEekpiTh();
    int GetKCells();
    double GetEekpiB();
    double GetEekpiLambda();

  private:
    double m_eekpiTh;
    double m_avgWeightedEekpiTh;
    int m_kCells;
    double m_eekpiB;
    double m_eekpiLambda;
};

/**
 * @brief Class that contains all methods related to heuristics on energy efficiency simulations
 *
 */
class MavenirHeuristic : public Object
{
  public:
    static TypeId GetTypeId(void);

    MavenirHeuristic();

    virtual ~MavenirHeuristic(void);

    /**
     * @brief implementation of Mavenir heuristic
     *
     * @param nMmWaveEnbNodes Number of mmWave BSs in the scenario
     * @param mmWaveEnbDevs Object that contains all the mmWave BSs
     * @param ltedev pointer to the LteEnbNetDevice used to obtain the m_rrc to change the state of
     * the cells
     * @param numberOfClusters Number of clusters in the scenario
     * @param clusters The vector of clusters
     */
    void MavenirHeur(uint8_t nMmWaveEnbNodes,
                     NetDeviceContainer mmWaveEnbDevs,
                     Ptr<LteEnbNetDevice> ltedev,
                     std::vector<std::vector<Ptr<MmWaveEnbNetDevice>>> clusters,
                     Ptr<MavHeurParameters> mavenirHeurPar);

    /**
     * @brief it read an string that represent a cluster in the form [[1,2,3],[4,5,6]] where [1,2,3]
     * is one cluster and [4,5,6] the other. it returns a vector of vectors that contains the
     * MmWaveEnbNetDevice of each cell
     *
     * @param clusters String that represnt the cluster
     * @param nMmWaveEnbNodes Number of mmWave BSs in the scenario
     * @param mmWaveEnbDevs Object that contains all the mmWave BSs
     * @return Vector of vectors that contains the MmWaveEnbNetDevice of each cell
     */
    std::vector<std::vector<Ptr<MmWaveEnbNetDevice>>> ReadClusters(
        std::string clustersString,
        uint8_t nMmWaveEnbNodes,
        NetDeviceContainer mmWaveEnbDevs);
};

} // namespace mmwave
} // namespace ns3