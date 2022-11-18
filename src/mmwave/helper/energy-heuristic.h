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
#include <ns3/lte-ue-net-device.h>


namespace ns3 {
namespace mmwave {
class EnergyHeuristic : public Object{
  public:
  static TypeId GetTypeId (void);

  EnergyHeuristic ();

  virtual ~EnergyHeuristic (void);
  std::vector<std::pair<Ptr<MmWaveEnbNetDevice>, double>> HeuristicDynamic(int BStoTurnON[], int BsON, int BsIdle, int BsSleep, int BsOFF, int nMmWaveEnbNodes, NetDeviceContainer mmWaveEnbDevs);
  std::vector<std::pair<Ptr<MmWaveEnbNetDevice>, double>> HeuristicStatic (int BStoTurnON[], int BsON, int BsIdle, int BsSleep, int BsOFF, int nMmWaveEnbNodes, NetDeviceContainer mmWaveEnbDevs);
  void TurnOnBsSinrPos (uint8_t nMmWaveEnbNodes, NetDeviceContainer mmWaveEnbDevs, std::string heuristic, int BSstatus[]);
  void ProbabilityState (double p1, double p2, double p3, double p4, uint16_t nodeId, Ptr<MmWaveEnbNetDevice> mmdev);
  void CountBestUesSinr(double SINRth, Ptr<MmWaveEnbNetDevice> mmdev);

};

}
}