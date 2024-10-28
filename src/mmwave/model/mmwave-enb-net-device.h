/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
*   Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
*   Copyright (c) 2015, NYU WIRELESS, Tandon School of Engineering, New York University
*   Copyright (c) 2016, 2018, University of Padova, Dep. of Information Engineering, SIGNET lab.
*   Copyright (c) 2024 Orange Innovation Egypt
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
*                         Mostafa Ashraf <mostafa.ashraf.ext@orange.com>
*                         Aya Kamal <aya.kamal.ext@orange.com>
*                         Abdelrhman Soliman <abdelrhman.soliman.ext@orange.com>
*
*        Modified by: Tommaso Zugno <tommasozugno@gmail.com>
*								 Integration of Carrier Aggregation
*/



#ifndef SRC_MMWAVE_MODEL_MMWAVE_ENB_NET_DEVICE_H_
#define SRC_MMWAVE_MODEL_MMWAVE_ENB_NET_DEVICE_H_


#include "mmwave-net-device.h"
#include "ns3/event-id.h"
#include "ns3/traced-callback.h"
#include "ns3/nstime.h"
#include "mmwave-phy.h"
#include "mmwave-enb-phy.h"
#include "mmwave-enb-mac.h"
#include "mmwave-mac-scheduler.h"
#include <vector>
#include <map>
#include <ns3/lte-enb-rrc.h>
#include <ns3/oran-interface.h>
#include "ns3/mmwave-bearer-stats-calculator.h"
#include <ns3/mmwave-phy-trace.h>


namespace ns3 {
/* Add forward declarations here */
    class Packet;

    class PacketBurst;

    class Node;

    class LteEnbComponentCarrierManager;

    namespace mmwave {
//class MmWavePhy;
        class MmWaveEnbPhy;

        class MmWaveEnbMac;

        typedef std::pair <uint64_t, uint16_t> ImsiCellIdPair_t;

        class MmWaveEnbNetDevice : public MmWaveNetDevice {
        public:
            const static uint16_t E2SM_REPORT_MAX_NEIGH = 8;

            static TypeId GetTypeId(void);

            MmWaveEnbNetDevice();

            virtual ~MmWaveEnbNetDevice(void);

            virtual void DoDispose(void) override;

            virtual bool DoSend(Ptr<Packet> packet, const Address &dest, uint16_t protocolNumber) override;

            Ptr<MmWaveEnbPhy> GetPhy(void) const;

            Ptr<MmWaveEnbPhy> GetPhy(uint8_t index);

            uint16_t GetCellId() const;

            std::map<uint16_t, Ptr<UeManager>> GetUeMap ();

            bool HasCellId(uint16_t cellId) const;

            uint8_t GetBandwidth() const;

            void SetBandwidth(uint8_t bw);

            Ptr<MmWaveEnbMac> GetMac(void);

            Ptr<MmWaveEnbMac> GetMac(uint8_t index);

            void SetRrc(Ptr<LteEnbRrc> rrc);

            Ptr<LteEnbRrc> GetRrc(void);

            void SetE2Termination(Ptr<E2Termination> e2term);

            Ptr<E2Termination> GetE2Termination() const;

            void SetCcMap(std::map <uint8_t, Ptr<MmWaveComponentCarrier>> ccm) override;

            void BuildAndSendReportMessage(E2Termination::RicSubscriptionRequest_rval_s params);

            void KpmSubscriptionCallback(E2AP_PDU_t *sub_req_pdu);

            void ControlMessageReceivedCallback(E2AP_PDU_t *sub_req_pdu);
            void SetStartTime(uint64_t);

            void stopSendingAndCancelSchedule();

        protected:
            virtual void DoInitialize(void) override;

            void UpdateConfig();

            void GetPeriodicPdcpStats();


        private:

            bool m_stopSendingMessages;

            Ptr<MmWaveMacScheduler> m_scheduler;

            Ptr<LteEnbRrc> m_rrc;

            uint16_t m_cellId;       /* Cell Identifer. To uniquely identify an E-nodeB  */

            uint8_t m_Bandwidth;       /* bandwidth in RBs (?) */

            Ptr<LteEnbComponentCarrierManager> m_componentCarrierManager; ///< the component carrier manager of this eNb

            bool m_isConfigured;

            Ptr<E2Termination> m_e2term;
            Ptr<MmWaveBearerStatsCalculator> m_e2PdcpStatsCalculator;
            Ptr<MmWaveBearerStatsCalculator> m_e2RlcStatsCalculator;
            Ptr<MmWavePhyTrace> m_e2DuCalculator;

            double m_e2Periodicity;

            // TODO doxy
            Ptr<KpmIndicationHeader> BuildRicIndicationHeader(std::string plmId, std::string gnbId, uint16_t nrCellId);

            Ptr<KpmIndicationMessage> BuildRicIndicationMessageCuUp(std::string plmId);

            Ptr<KpmIndicationMessage> BuildRicIndicationMessageCuCp(std::string plmId);

            Ptr<KpmIndicationMessage> BuildRicIndicationMessageDu(std::string plmId, uint16_t nrCellId);

            std::string GetImsiString(uint64_t imsi);

            uint32_t GetRlcBufferOccupancy(Ptr<LteRlc> rlc) const;

            bool m_sendCuUp;
            bool m_sendCuCp;
            bool m_sendDu;

            static void
            RegisterNewSinrReadingCallback(Ptr<MmWaveEnbNetDevice> netDev, std::string context, uint64_t imsi,
                                           uint16_t cellId, long double sinr);

            void RegisterNewSinrReading(uint64_t imsi, uint16_t cellId, long double sinr);

            std::map <uint64_t, std::map<uint16_t, long double>> m_l3sinrMap;
            uint64_t m_startTime;
            std::map <uint64_t, uint32_t> m_drbThrDlPdcpBasedComputationUeid;
            std::map <uint64_t, uint32_t> m_drbThrDlUeid;
            bool m_isReportingEnabled; //! true is KPM reporting cycle is active, false otherwise
            bool m_reducedPmValues; //< if true use a reduced subset of pmvalues

            uint16_t m_basicCellId;
            double  rc_e2_func_id ; // to RC
            double e2_func_id; //to pass kpm function id
            bool m_e2andlog; //if true, both e2 term and e2file logging will work
            bool m_forceE2FileLogging; //< if true log PMs to files
            std::string m_cuUpFileName;
            std::string m_cuCpFileName;
            std::string m_duFileName;

        };
    }
}


#endif /* SRC_MMWAVE_MODEL_MMWAVE_ENB_NET_DEVICE_H_ */
