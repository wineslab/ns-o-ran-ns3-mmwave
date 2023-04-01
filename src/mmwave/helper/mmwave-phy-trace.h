/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *   Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *   Copyright (c) 2015, NYU WIRELESS, Tandon School of Engineering, New York University
 *        Copyright (c) 2016, 2018, University of Padova, Dep. of Information Engineering, SIGNET
 *lab
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
 *                          Integration of Carrier Aggregation for the mmWave module
 */

#ifndef SRC_MMWAVE_HELPER_MMWAVE_PHY_TRACE_H_
#define SRC_MMWAVE_HELPER_MMWAVE_PHY_TRACE_H_
#include <ns3/mmwave-phy-mac-common.h>
#include <ns3/object.h>
#include <ns3/spectrum-value.h>

#include <fstream>
#include <iostream>

namespace ns3
{

namespace mmwave
{

typedef std::pair<uint16_t, uint16_t> RntiCellIdPair_t;

class MmWavePhyTrace : public Object
{
  public:
    MmWavePhyTrace();
    virtual ~MmWavePhyTrace();
    static TypeId GetTypeId(void);
    static void ReportCurrentCellRsrpSinrCallback(Ptr<MmWavePhyTrace> phyStats,
                                                  std::string path,
                                                  uint64_t imsi,
                                                  SpectrumValue& sinr,
                                                  SpectrumValue& power);
    static void ReportDownLinkTBSize(Ptr<MmWavePhyTrace> phyStats,
                                     std::string path,
                                     uint64_t imsi,
                                     uint64_t tbSize);
    static void RxPacketTraceUeCallback(Ptr<MmWavePhyTrace> phyStats,
                                        std::string path,
                                        RxPacketTraceParams param);
    static void RxPacketTraceEnbCallback(Ptr<MmWavePhyTrace> phyStats,
                                         std::string path,
                                         RxPacketTraceParams param);

    /**
     * Callback used to trace an UL PHY tranmission
     */
    static void ReportUlPhyTransmissionCallback(Ptr<MmWavePhyTrace> phyStats,
                                                PhyTransmissionTraceParams param);

    /**
     * Callback used to trace a DL PHY tranmission
     */
    static void ReportDlPhyTransmissionCallback(Ptr<MmWavePhyTrace> phyStats,
                                                PhyTransmissionTraceParams param);

    /**
     * Sets the filename of the PHY reception traces
     * \param fileName the file name
     */
    void SetPhyRxOutputFilename(std::string fileName);

    /**
     * Sets the filename of the UL PHY tranmission traces
     * \param fileName the file name
     */
    void SetUlPhyTxOutputFilename(std::string fileName);

    /**
     * Sets the filename of the DL PHY tranmission traces
     * \param fileName the file name
     */
    void SetDlPhyTxOutputFilename(std::string fileName);
    /**
     * Gets the number of MAC PDUs, UE specific
     * @param rnti
     * @param cellId
     * @return the number of MAC PDUs, UE specific
     */
    uint32_t GetMacPduUeSpecific(uint16_t rnti, uint16_t cellId);

    /**
     * Gets the number of MAC PDUs (initial tx), UE specific
     * @param rnti
     * @param cellId
     * @return the number of MAC PDUs (initial tx), UE specific
     */
    uint32_t GetMacPduInitialTransmissionUeSpecific(uint16_t rnti, uint16_t cellId);

    /**
     * Gets the number of MAC PDUs (retx), UE specific
     * @param rnti
     * @param cellId
     * @return the number of MAC PDUs (retx), UE specific
     */
    uint32_t GetMacPduRetransmissionUeSpecific(uint16_t rnti, uint16_t cellId);

    /**
     * Gets MAC volume (amount of TXed bytes), UE specific
     * @param rnti
     * @param cellId
     * @return amount of TXed bytes
     */
    uint32_t GetMacVolumeUeSpecific(uint16_t rnti, uint16_t cellId);

    /**
     * Gets the number of MAC PDUs with QPSK, UE specific
     * @param rnti
     * @param cellId
     * @return number of MAC PDUs with QPSK
     */
    uint32_t GetMacPduQpskUeSpecific(uint16_t rnti, uint16_t cellId);

    /**
     * Gets the number of MAC PDUs with 16QAM, UE specific
     * @param rnti
     * @param cellId
     * @return number of MAC PDUs with 16QAM
     */
    uint32_t GetMacPdu16QamUeSpecific(uint16_t rnti, uint16_t cellId);

    /**
     * Gets the number of MAC PDUs with 64QAM, UE specific
     * @param rnti
     * @param cellId
     * @return number of MAC PDUs with 64QAM
     */
    uint32_t GetMacPdu64QamUeSpecific(uint16_t rnti, uint16_t cellId);

    /**
     * Gets the number of symbols, UE specific
     * @param rnti
     * @param cellId
     * @return number of symbols
     */
    uint32_t GetMacNumberOfSymbolsUeSpecific(uint16_t rnti, uint16_t cellId);

    /**
     * Gets the number of TX with MCS 0-4
     * @param rnti
     * @param cellId
     * @return number of TX with MCS 0-4
     */
    uint32_t GetMacMcs04UeSpecific(uint16_t rnti, uint16_t cellId);

    /**
     * Gets the number of TX with MCS 5-9
     * @param rnti
     * @param cellId
     * @return number of TX with MCS 5-9
     */
    uint32_t GetMacMcs59UeSpecific(uint16_t rnti, uint16_t cellId);

    /**
     * Gets the number of TX with MCS 10-14
     * @param rnti
     * @param cellId
     * @return number of TX with MCS 10-14
     */
    uint32_t GetMacMcs1014UeSpecific(uint16_t rnti, uint16_t cellId);

    /**
     * Gets the number of TX with MCS 15-19
     * @param rnti
     * @param cellId
     * @return number of TX with MCS 15-19
     */
    uint32_t GetMacMcs1519UeSpecific(uint16_t rnti, uint16_t cellId);

    /**
     * Gets the number of TX with MCS 20-24
     * @param rnti
     * @param cellId
     * @return number of TX with MCS 20-24
     */
    uint32_t GetMacMcs2024UeSpecific(uint16_t rnti, uint16_t cellId);

    /**
     * Gets the number of TX with MCS 25-29
     * @param rnti
     * @param cellId
     * @return number of TX with MCS 25-29
     */
    uint32_t GetMacMcs2529UeSpecific(uint16_t rnti, uint16_t cellId);

    /**
     * Gets the number of TX with SINR < 6 dB
     * @param rnti
     * @param cellId
     * @return number of TX with SINR < 6 dB
     */
    uint32_t GetMacSinrBin1UeSpecific(uint16_t rnti, uint16_t cellId);

    /**
     * Gets the number of TX with SINR 0-6 dB
     * @param rnti
     * @param cellId
     * @return number of TX with SINR 0-6 dB
     */
    uint32_t GetMacSinrBin2UeSpecific(uint16_t rnti, uint16_t cellId);

    /**
     * Gets the number of TX with SINR 6-12 dB
     * @param rnti
     * @param cellId
     * @return number of TX with SINR 6-12 dB
     */
    uint32_t GetMacSinrBin3UeSpecific(uint16_t rnti, uint16_t cellId);

    /**
     * Gets the number of TX with SINR 12-18 dB
     * @param rnti
     * @param cellId
     * @return number of TX with SINR 12-18 dB
     */
    uint32_t GetMacSinrBin4UeSpecific(uint16_t rnti, uint16_t cellId);

    /**
     * Gets the number of TX with SINR 12-18 dB
     * @param rnti
     * @param cellId
     * @return number of TX with SINR 12-18 dB
     */
    uint32_t GetMacSinrBin5UeSpecific(uint16_t rnti, uint16_t cellId);

    /**
     * Gets the number of TX with SINR 18-24 dB
     * @param rnti
     * @param cellId
     * @return number of TX with SINR 18-24 dB
     */
    uint32_t GetMacSinrBin6UeSpecific(uint16_t rnti, uint16_t cellId);

    /**
     * Gets the number of TX with SINR > 24 dB
     * @param rnti
     * @param cellId
     * @return number of TX with SINR > 24 dB
     */
    uint32_t GetMacSinrBin7UeSpecific(uint16_t rnti, uint16_t cellId);

    /**
     * Reset the counters for a specific UE
     * @param rnti
     * @param cellId
     */
    void ResetPhyTracesForRntiCellId(uint16_t rnti, uint16_t cellId);

    /**
     * Get last reset time
     * @param rnti
     * @param cellId
     */
    Time GetLastResetTime(uint16_t rnti, uint16_t cellId);

  private:
    // void ReportInterferenceTrace (uint64_t imsi, SpectrumValue& sinr);
    // void ReportDLTbSize (uint64_t imsi, uint64_t tbSize);
    static std::ofstream m_rxPacketTraceFile;   //!< Output stream for the PHY reception trace
    static std::string m_rxPacketTraceFilename; //!< Output filename for the PHY reception trace

    static std::ofstream m_ulPhyTraceFile;   //!< Output stream for the UL PHY transmission trace
    static std::string m_ulPhyTraceFilename; //!< Output filename for the UL PHY transmission trace

    static std::ofstream m_dlPhyTraceFile;   //!< Output stream for the DL PHY transmission trace
    static std::string m_dlPhyTraceFilename; //!< Output filename for the DL PHY transmission trace

    std::map<RntiCellIdPair_t, uint32_t> m_macPduUeSpecific;
    std::map<RntiCellIdPair_t, uint32_t> m_macPduInitialTransmissionUeSpecific;
    std::map<RntiCellIdPair_t, uint32_t> m_macPduRetransmissionUeSpecific;
    std::map<RntiCellIdPair_t, uint32_t> m_macVolumeUeSpecific;   //!< UE specific MAC volume
    std::map<RntiCellIdPair_t, uint32_t> m_macPduQpskUeSpecific;  //!< UE specific MAC PDUs QPSK
    std::map<RntiCellIdPair_t, uint32_t> m_macPdu16QamUeSpecific; //!< UE specific MAC PDUs 16QAM
    std::map<RntiCellIdPair_t, uint32_t> m_macPdu64QamUeSpecific; //!< UE specific MAC PDUs 64QAM
    std::map<RntiCellIdPair_t, uint32_t> m_macMcs04UeSpecific;    //!< UE specific TX with MCS 0-4
    std::map<RntiCellIdPair_t, uint32_t> m_macMcs59UeSpecific;    //!< UE specific TX with MCS 5-9
    std::map<RntiCellIdPair_t, uint32_t> m_macMcs1014UeSpecific;  //!< UE specific TX with MCS 10-14
    std::map<RntiCellIdPair_t, uint32_t> m_macMcs1519UeSpecific;  //!< UE specific TX with MCS 15-19
    std::map<RntiCellIdPair_t, uint32_t> m_macMcs2024UeSpecific;  //!< UE specific TX with MCS 20-24
    std::map<RntiCellIdPair_t, uint32_t> m_macMcs2529UeSpecific;  //!< UE specific TX with MCS 25-29

    std::map<RntiCellIdPair_t, uint32_t>
        m_macSinrBin1UeSpecific; //!< UE specific TX with SINR < -6dB
    std::map<RntiCellIdPair_t, uint32_t>
        m_macSinrBin2UeSpecific; //!< UE specific TX with SINR -6dB to 0 dB
    std::map<RntiCellIdPair_t, uint32_t>
        m_macSinrBin3UeSpecific; //!< UE specific TX with SINR 0 dB to 6 dB
    std::map<RntiCellIdPair_t, uint32_t>
        m_macSinrBin4UeSpecific; //!< UE specific TX with SINR 6 dB to 12 dB
    std::map<RntiCellIdPair_t, uint32_t>
        m_macSinrBin5UeSpecific; //!< UE specific TX with SINR 12 dB to 18 dB
    std::map<RntiCellIdPair_t, uint32_t>
        m_macSinrBin6UeSpecific; //!< UE specific TX with SINR 18 dB to 24 dB
    std::map<RntiCellIdPair_t, uint32_t>
        m_macSinrBin7UeSpecific; //!< UE specific TX with SINR > 24 dB

    std::map<RntiCellIdPair_t, uint32_t> m_macNumberOfSymbols; //!< UE specific number of symbols

    std::map<RntiCellIdPair_t, Time> m_lastReset; //! last time UE was reset

    void UpdateTraces(RxPacketTraceParams params);

    /**
     * Update the value of an entry in a map
     * @param map
     * @param key
     * @param newValue
     */
    std::map<RntiCellIdPair_t, uint32_t> UpdateMapValue(std::map<RntiCellIdPair_t, uint32_t> map,
                                                        RntiCellIdPair_t key,
                                                        uint32_t newValue);
    /**
     * Increase the value of an entry in a map by 1
     * @param map
     * @param key
     * @param newValue
     */
    std::map<RntiCellIdPair_t, uint32_t> IncreaseMapValue(std::map<RntiCellIdPair_t, uint32_t> map,
                                                          RntiCellIdPair_t key,
                                                          uint32_t value);

    /**
     * Get an entry in a map
     * @param map
     * @param key
     */
    uint32_t GetMapValue(std::map<RntiCellIdPair_t, uint32_t> map, RntiCellIdPair_t key);

    /**
     * Erase an entry in a map
     * @param map
     * @param key
     */
    std::map<RntiCellIdPair_t, uint32_t> ResetMapValue(std::map<RntiCellIdPair_t, uint32_t> map,
                                                       RntiCellIdPair_t key);
};

} // namespace mmwave

} /* namespace ns3 */

#endif /* SRC_MMWAVE_HELPER_MMWAVE_PHY_RX_TRACE_H_ */
