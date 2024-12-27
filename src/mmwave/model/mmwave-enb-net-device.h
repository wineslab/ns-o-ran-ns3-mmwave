/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *   Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *   Copyright (c) 2015, NYU WIRELESS, Tandon School of Engineering, New York University
 *   Copyright (c) 2016, 2018, University of Padova, Dep. of Information Engineering, SIGNET lab.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation;
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *   Author: Marco Miozzo <marco.miozzo@cttc.es>
 *           Nicola Baldo  <nbaldo@cttc.es>
 *
 *   Modified by: Marco Mezzavilla < mezzavilla@nyu.edu>
 *                         Sourjya Dutta <sdutta@nyu.edu>
 *                         Russell Ford <russell.ford@nyu.edu>
 *                         Menglei Zhang <menglei@nyu.edu>
 *
 *        Modified by: Tommaso Zugno <tommasozugno@gmail.com>
 *								 Integration of Carrier Aggregation
 */

#ifndef SRC_MMWAVE_MODEL_MMWAVE_ENB_NET_DEVICE_H_
#define SRC_MMWAVE_MODEL_MMWAVE_ENB_NET_DEVICE_H_

#include "mmwave-enb-mac.h"
#include "mmwave-enb-phy.h"
#include "mmwave-mac-scheduler.h"
#include "mmwave-net-device.h"
#include "mmwave-phy.h"

#include "ns3/event-id.h"
#include "ns3/mmwave-bearer-stats-calculator.h"
#include "ns3/nstime.h"
#include "ns3/traced-callback.h"
#include <ns3/lte-enb-rrc.h>
#include <ns3/mmwave-phy-trace.h>
#include <ns3/oran-interface.h>

#include <map>
#include <vector>

namespace ns3
{
/* Add forward declarations here */
class Packet;
class PacketBurst;
class Node;
class LteEnbComponentCarrierManager;

namespace mmwave
{
// class MmWavePhy;
class MmWaveEnbPhy;
class MmWaveEnbMac;

typedef std::pair<uint64_t, uint16_t> ImsiCellIdPair_t;

class MmWaveEnbNetDevice : public MmWaveNetDevice
{
  public:
    const static uint16_t E2SM_REPORT_MAX_NEIGH = 8;

    static TypeId GetTypeId(void);

    MmWaveEnbNetDevice();

    virtual ~MmWaveEnbNetDevice(void);
    virtual void DoDispose(void) override;
    virtual bool DoSend(Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber) override;

    Ptr<MmWaveEnbPhy> GetPhy(void) const;

    Ptr<MmWaveEnbPhy> GetPhy(uint8_t index);

    uint16_t GetCellId() const;

    bool HasCellId(uint16_t cellId) const;

    uint8_t GetBandwidth() const;

    void SetBandwidth(uint8_t bw);

    Ptr<MmWaveEnbMac> GetMac(void);

    Ptr<MmWaveEnbMac> GetMac(uint8_t index);

    void SetRrc(Ptr<LteEnbRrc> rrc);

    Ptr<LteEnbRrc> GetRrc(void);

    void SetE2Termination(Ptr<E2Termination> e2term);

    Ptr<E2Termination> GetE2Termination() const;

    void SetCcMap(std::map<uint8_t, Ptr<MmWaveComponentCarrier>> ccm) override;

    void BuildAndSendReportMessage(E2Termination::RicSubscriptionRequest_rval_s params);

    void KpmSubscriptionCallback(E2AP_PDU_t* sub_req_pdu);

    /**
     * @brief Turn the Cell On
     *
     * @param nodeId
     */
    void TurnOn(uint16_t nodeId, Ptr<LteEnbRrc> m_rrc);
    /**
     * @brief Turn the cell Idle
     *
     * @param nodeId
     */
    void TurnIdle(uint16_t nodeId, Ptr<LteEnbRrc> m_rrc);
    /**
     * @brief Turn the cell Sleep
     *
     * @param nodeId
     */
    void TurnSleep(uint16_t nodeId, Ptr<LteEnbRrc> m_rrc);
    /**
     * @brief Turn the cell Off
     *
     * @param nodeId
     */
    void TurnOff(uint16_t nodeId, Ptr<LteEnbRrc> m_rrc);

    bool GetBsState();

    void ControlMessageReceivedCallback(E2AP_PDU_t* sub_req_pdu);

    void SetStartTime(uint64_t);

    uint64_t GetStartTime();

    uint16_t GetNUeGoodSinr();

    void SetNUeGoodSinr(uint16_t value);

    std::pair<double, double> GetClosestUePos();

    void SetClosestUePos(std::pair<double, double>);

    uint32_t GetMacVolumeCellSpecific();

    uint32_t GetMacPduCellSpecific();

    void SetTurnOffTime(double value);

    double GetTurnOffTime();

    void SetClosestUeTime(double);

    double GetClosestUeTime();

    std::map<uint64_t, std::map<uint16_t, long double>> Getl3sinrMap();

    /**
     * @brief All the possible energy mode for the cell
     *
     */
    enum enumModeEnergyBs
    {
        ON = 1,
        Idle = 1,
        Sleep = 0,
        OFF = 0
    };

    void SetCellState(enumModeEnergyBs value);

    uint32_t GetmacPduInitialCellSpecificAttr();
    std::string GetImsiString(uint64_t imsi);

  protected:
    virtual void DoInitialize(void) override;
    void UpdateConfig();

    void GetPeriodicPdcpStats();
    uint32_t GetRlcBufferOccupancy(Ptr<LteRlc> rlc) const;

  private:
    /**
     * @brief Identify the state of the cell: ON, Idle, Sleep, OFF
     *
     */
    enumModeEnergyBs m_CellState = enumModeEnergyBs::ON;

    Ptr<MmWaveMacScheduler> m_scheduler;

    Ptr<LteEnbRrc> m_rrc;

    uint16_t m_cellId; /* Cell Identifer. To uniquely identify an E-nodeB  */

    uint8_t m_Bandwidth; /* bandwidth in RBs (?) */

    Ptr<LteEnbComponentCarrierManager>
        m_componentCarrierManager; ///< the component carrier manager of this eNb

    bool m_isConfigured;

    Ptr<E2Termination> m_e2term;
    Ptr<MmWaveBearerStatsCalculator> m_e2PdcpStatsCalculator;
    Ptr<MmWaveBearerStatsCalculator> m_e2RlcStatsCalculator;
    Ptr<MmWavePhyTrace> m_e2DuCalculator;

    double m_e2Periodicity;
    // TODO doxy
    Ptr<KpmIndicationHeader> BuildRicIndicationHeader(std::string plmId,
                                                      std::string gnbId,
                                                      uint16_t nrCellId);
    Ptr<KpmIndicationMessage> BuildRicIndicationMessageCuUp(std::string plmId);
    Ptr<KpmIndicationMessage> BuildRicIndicationMessageCuCp(std::string plmId);
    Ptr<KpmIndicationMessage> BuildRicIndicationMessageDu(std::string plmId, uint16_t nrCellId);

    /**
     * @brief Save at each granularity period of 10 ms the number of UEs connected to the cell
     *
     */
    void SaveActiveUes();
    /**
     * @brief Compute the RRC.ConnMean KPM from 3GGP TR xxxx
     *
     * @return long
     */
    long ComputeMeanUes();

    bool m_sendCuUp;
    bool m_sendCuCp;
    bool m_sendDu;

    static void RegisterNewSinrReadingCallback(Ptr<MmWaveEnbNetDevice> netDev,
                                               std::string context,
                                               uint64_t imsi,
                                               uint16_t cellId,
                                               long double sinr);
    void RegisterNewSinrReading(uint64_t imsi, uint16_t cellId, long double sinr);
    std::map<uint64_t, std::map<uint16_t, long double>> m_l3sinrMap;
    uint64_t m_startTime;
    std::map<uint64_t, double> m_drbThrDlPdcpBasedComputationUeid;
    std::map<uint64_t, double> m_drbThrDlUeid;
    bool m_isReportingEnabled; //! true is KPM reporting cycle is active, false otherwise
    bool m_reducedPmValues;    //< if true use a reduced subset of pmvalues

    uint16_t m_basicCellId;

    bool m_forceE2FileLogging; //< if true log PMs to files
    std::string m_cuUpFileName;
    std::string m_cuCpFileName;
    std::string m_duFileName;

    std::vector<size_t> m_ueRrcMean;

    /**
     * @brief The number of connected UEs that have a good SINR value, higher than a certain
     * threshold
     *
     */
    uint16_t m_nUeGoodSinr = 0;
    /**
     * @brief The position of the closet UE to the cell
     *
     */
    std::pair<double, double> m_closestUEPos = {10000.0, 10000.0};
    /**
     * @brief The smallest time for a connected UE to reach the cell
     *
     */
    double m_closestUETime = 10000.0;

    /**
     * @brief Attribute representing the macPduCellSpecific value for the cell 
     *
     */
    uint32_t m_macPduCellSpecific = 0;
    /**
     * @brief Attribute representing the macVolumeCellSpecific value for the cell
     *
     */
    uint32_t m_macVolumeCellSpecific = 0;
    /**
     * @brief at which time the cell is turned off
     *
     */
    double m_turnOffTime = 0;
};
} // namespace mmwave
} // namespace ns3

#endif /* SRC_MMWAVE_MODEL_MMWAVE_ENB_NET_DEVICE_H_ */
