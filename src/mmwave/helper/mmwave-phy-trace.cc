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
* Modified by: Michele Polese <michele.polese@gmail.com>
*                 Dual Connectivity and Handover functionalities
*
* Modified by: Tommaso Zugno <tommasozugno@gmail.com>
*							Integration of Carrier Aggregation for the mmWave module
*/


#include <ns3/log.h>
#include "mmwave-phy-trace.h"
#include <ns3/simulator.h>
#include <stdio.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MmWavePhyTrace");

namespace mmwave {

NS_OBJECT_ENSURE_REGISTERED (MmWavePhyTrace);

std::ofstream MmWavePhyTrace::m_rxPacketTraceFile;
std::string MmWavePhyTrace::m_rxPacketTraceFilename;

std::ofstream MmWavePhyTrace::m_ulPhyTraceFile {};
std::string MmWavePhyTrace::m_ulPhyTraceFilename {};

std::ofstream MmWavePhyTrace::m_dlPhyTraceFile {};
std::string MmWavePhyTrace::m_dlPhyTraceFilename {};

MmWavePhyTrace::MmWavePhyTrace ()
{
}

MmWavePhyTrace::~MmWavePhyTrace ()
{
  if (m_rxPacketTraceFile.is_open ())
    {
      m_rxPacketTraceFile.close ();
    }
}

TypeId
MmWavePhyTrace::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MmWavePhyTrace")
    .SetParent<Object> ()
    .AddConstructor<MmWavePhyTrace> ()
    .AddAttribute ("OutputFilename",
                   "Name of the file where the uplink results will be saved.",
                   StringValue ("RxPacketTrace.txt"),
                   MakeStringAccessor (&MmWavePhyTrace::SetPhyRxOutputFilename),
                   MakeStringChecker ())
    .AddAttribute ("UlPhyTransmissionFilename",
                   "Name of the file where the UL transmission info will be saved.",
                   StringValue ("UlPhyTransmissionTrace.txt"),
                   MakeStringAccessor (&MmWavePhyTrace::SetUlPhyTxOutputFilename),
                   MakeStringChecker ())
    .AddAttribute ("DlPhyTransmissionFilename",
                   "Name of the file where the DL transmission info will be saved.",
                   StringValue ("DlPhyTransmissionTrace.txt"),
                   MakeStringAccessor (&MmWavePhyTrace::SetDlPhyTxOutputFilename),
                   MakeStringChecker ())
          
  ;
  return tid;
}

void
MmWavePhyTrace::SetPhyRxOutputFilename (std::string fileName)
{
  NS_LOG_INFO ("RxPacketTrace main filename: " << fileName);
  m_rxPacketTraceFilename = fileName;
}

void
MmWavePhyTrace::SetUlPhyTxOutputFilename (std::string fileName)
{
  NS_LOG_INFO ("UL PHY transmission trace filename: " << fileName);
  m_ulPhyTraceFilename = fileName;
}

void
MmWavePhyTrace::SetDlPhyTxOutputFilename (std::string fileName)
{
  NS_LOG_INFO ("DL PHY transmission trace filename: " << fileName);
  m_dlPhyTraceFilename = fileName;
}

void
MmWavePhyTrace::ReportCurrentCellRsrpSinrCallback (Ptr<MmWavePhyTrace> phyStats, std::string path,
                                                     uint64_t imsi, SpectrumValue& sinr, SpectrumValue& power)
{
  NS_LOG_INFO ("UE" << imsi << "->Generate RsrpSinrTrace");
  //phyStats->ReportInterferenceTrace (imsi, sinr);
}



/*void
MmWavePhyTrace::ReportInterferenceTrace (uint64_t imsi, SpectrumValue& sinr)
{
        uint64_t slot_count = Now().GetMicroSeconds ()/125;
        uint32_t rb_count = 1;
        FILE* log_file;
        char fname[255];
        sprintf(fname, "UE_%llu_SINR_dB.txt", (long long unsigned ) imsi);
        log_file = fopen(fname, "a");
        Values::iterator it = sinr.ValuesBegin();
        while(it!=sinr.ValuesEnd())
        {
                //fprintf(log_file, "%d\t%d\t%f\t \n", slot_count/2, rb_count, 10*log10(*it));
                fprintf(log_file, "%llu\t%llu\t%d\t%f\t \n",(long long unsigned) slot_count/8+1, (long long unsigned) slot_count%8+1, rb_count, 10*log10(*it));
                rb_count++;
                it++;
        }
        fflush(log_file);
        fclose(log_file);
}*/

void
MmWavePhyTrace::ReportDownLinkTBSize (Ptr<MmWavePhyTrace> phyStats, std::string path,
                                        uint64_t imsi, uint64_t tbSize)
{
  //phyStats->ReportDLTbSize (imsi, tbSize);
}


/*
void
MmWavePhyTrace::ReportPacketCountEnb (EnbPhyPacketCountParameter param)
{
        FILE* log_file;
        char fname[255];
        sprintf (fname,"BS_%llu_Packet_Trace.txt",(long long unsigned) param.m_cellId);
        log_file = fopen (fname, "a");
        if (param.m_isTx)
        {
                fprintf (log_file, "%d\t%d\t%d\n", param.m_subframeno, param.m_noBytes, 0);
        }
        else
        {
                fprintf (log_file, "%d\t%d\t%d\n", param.m_subframeno, 0, param.m_noBytes);
        }

        fflush(log_file);
        fclose(log_file);
}

void
MmWavePhyTrace::ReportDLTbSize (uint64_t imsi, uint64_t tbSize)
{
        FILE* log_file;
        char fname[255];
        sprintf (fname,"UE_%llu_Tb_Size.txt", (long long unsigned) imsi);
        log_file = fopen (fname, "a");

        fprintf (log_file, "%llu \t %llu\n", Now().GetMicroSeconds (), tbSize);
        fprintf (log_file, "%lld \t %llu \n",(long long int) Now().GetMicroSeconds (), (long long unsigned) tbSize);
        fflush(log_file);
        fclose(log_file);
}
*/
void 
MmWavePhyTrace::ReportUlPhyTransmissionCallback (Ptr<MmWavePhyTrace> phyStats, PhyTransmissionTraceParams param)
{
  if (!m_ulPhyTraceFile.is_open ())
    {
      m_ulPhyTraceFile.open (m_ulPhyTraceFilename.c_str ());
      if (!m_ulPhyTraceFile.is_open ())
        {
          NS_FATAL_ERROR ("Could not open tracefile");
        }
      m_ulPhyTraceFile << "frame\tsubF\tslot\trnti\tfirstSym\tnumSym\ttype\ttddMode\tretxNum\tccId" << std::endl;
    }

  // Trace the UL PHY transmission info
  m_ulPhyTraceFile << +param.m_frameNum << "\t" << +param.m_sfNum << "\t"
                   << +param.m_slotNum << "\t" << +param.m_rnti << "\t" 
                   << +param.m_symStart << "\t" << +param.m_numSym << "\t" 
                   << +param.m_ttiType << "\t" << +param.m_tddMode << "\t" 
                   << +param.m_rv << "\t" << +param.m_ccId << std::endl;
}

void 
MmWavePhyTrace::ReportDlPhyTransmissionCallback (Ptr<MmWavePhyTrace> phyStats, PhyTransmissionTraceParams param)
{
  if (!m_dlPhyTraceFile.is_open ())
    {
      m_dlPhyTraceFile.open (m_dlPhyTraceFilename.c_str ());
      if (!m_dlPhyTraceFile.is_open ())
        {
          NS_FATAL_ERROR ("Could not open tracefile");
        }
      m_dlPhyTraceFile << "frame\tsubF\tslot\trnti\tfirstSym\tnumSym\ttype\ttddMode\tretxNum\tccId" << std::endl;
    }

  // Trace the DL PHY transmission info
  m_dlPhyTraceFile << +param.m_frameNum << "\t" << +param.m_sfNum << "\t"
                   << +param.m_slotNum << "\t" << +param.m_rnti << "\t" 
                   << +param.m_symStart << "\t" << +param.m_numSym << "\t" 
                   << +param.m_ttiType << "\t" << +param.m_tddMode << "\t" 
                   << +param.m_rv << "\t" << +param.m_ccId << std::endl;
}

void
MmWavePhyTrace::UpdateTraces(RxPacketTraceParams params)
{
  RntiCellIdPair_t pair {params.m_rnti, params.m_cellId};

  NS_LOG_LOGIC("Update trace rnti " << params.m_rnti << " cellId " << params.m_cellId);
  
  m_macPduUeSpecific = IncreaseMapValue (m_macPduUeSpecific, pair, 1);

  if (params.m_rv == 0)
  {
    m_macPduInitialTransmissionUeSpecific = IncreaseMapValue (m_macPduInitialTransmissionUeSpecific, pair, 1);
    
  }
  else
  {
    m_macPduRetransmissionUeSpecific = IncreaseMapValue (m_macPduRetransmissionUeSpecific, pair, 1);
    
  }
  
  // UE specific MAC volume 
  m_macVolumeUeSpecific = IncreaseMapValue (m_macVolumeUeSpecific, pair, params.m_tbSize);
  
  if (params.m_mcs >= 0 && params.m_mcs <= 9)
  {
    // UE specific MAC PDUs QPSK
    m_macPduQpskUeSpecific = IncreaseMapValue (m_macPduQpskUeSpecific, pair, 1);
  }
  else if (params.m_mcs >= 10 && params.m_mcs <= 16)
  {
    // UE specific MAC PDUs 16QAM
    m_macPdu16QamUeSpecific = IncreaseMapValue (m_macPdu16QamUeSpecific, pair, 1);
  }
  else if (params.m_mcs >= 17 && params.m_mcs <= 28)
  {
    // UE specific MAC PDUs 64QAM
    m_macPdu64QamUeSpecific = IncreaseMapValue (m_macPdu64QamUeSpecific, pair, 1);
  } 
  
  if (params.m_mcs <= 4)
  {
    m_macMcs04UeSpecific = IncreaseMapValue(m_macMcs04UeSpecific, pair, 1);
  }
  else if (params.m_mcs >= 5 && params.m_mcs <= 9)
  {
    m_macMcs59UeSpecific = IncreaseMapValue(m_macMcs59UeSpecific, pair, 1);
  }
  else if (params.m_mcs >= 10 && params.m_mcs <= 14)
  {
    m_macMcs1014UeSpecific = IncreaseMapValue(m_macMcs1014UeSpecific, pair, 1);
  }
  else if (params.m_mcs >= 15 && params.m_mcs <= 19)
  {
    m_macMcs1519UeSpecific = IncreaseMapValue(m_macMcs1519UeSpecific, pair, 1);
  }
  else if (params.m_mcs >= 20 && params.m_mcs <= 24)
  {
    m_macMcs2024UeSpecific = IncreaseMapValue(m_macMcs2024UeSpecific, pair, 1);
  }
  else if (params.m_mcs >= 25 && params.m_mcs <= 29)
  {
    m_macMcs2529UeSpecific = IncreaseMapValue(m_macMcs2529UeSpecific, pair, 1);
  }

  double sinrLog = 10 * std::log10 (params.m_sinr);
  if (sinrLog <= -6)
  {
    m_macSinrBin1UeSpecific = IncreaseMapValue(m_macSinrBin1UeSpecific, pair, 1);
  }
  else if (sinrLog <= 0)
  {
    m_macSinrBin2UeSpecific = IncreaseMapValue(m_macSinrBin2UeSpecific, pair, 1);
  }
  else if (sinrLog <= 6)
  {
    m_macSinrBin3UeSpecific = IncreaseMapValue(m_macSinrBin3UeSpecific, pair, 1);
  }
  else if (sinrLog <= 12)
  {
    m_macSinrBin4UeSpecific = IncreaseMapValue(m_macSinrBin4UeSpecific, pair, 1);
  }
  else if (sinrLog <= 18)
  {
    m_macSinrBin5UeSpecific = IncreaseMapValue(m_macSinrBin5UeSpecific, pair, 1);
  }
  else if (sinrLog <= 24)
  {
    m_macSinrBin6UeSpecific = IncreaseMapValue(m_macSinrBin6UeSpecific, pair, 1);
  }
  else
  {
    m_macSinrBin7UeSpecific = IncreaseMapValue(m_macSinrBin7UeSpecific, pair, 1);
  }

  // UE specific number of symbols
  m_macNumberOfSymbols = IncreaseMapValue(m_macNumberOfSymbols, pair, params.m_numSym);

}

std::map<RntiCellIdPair_t, uint32_t>
MmWavePhyTrace::UpdateMapValue (std::map<RntiCellIdPair_t, uint32_t> map, 
                                RntiCellIdPair_t key, uint32_t newValue)
{
  auto pair = map.find (key);
  if (pair != map.end ())
  {
    pair->second = newValue;
  }
  else
  {
    map [key] = newValue;
  }
  return map; 
}

std::map<RntiCellIdPair_t, uint32_t>
MmWavePhyTrace::IncreaseMapValue (std::map<RntiCellIdPair_t, uint32_t> map, 
                                  RntiCellIdPair_t key, uint32_t value)
{
  auto pair = map.find (key);
  if (pair != map.end ())
  {
    pair->second += value;
  }
  else
  {
    map [key] = value;
  }  
  return map;
}

uint32_t
MmWavePhyTrace::GetMapValue (std::map<RntiCellIdPair_t, uint32_t> map, 
                             RntiCellIdPair_t key)
{
  uint32_t ret = 0;
  auto pair = map.find (key);
  if (pair != map.end ())
  {
    ret = pair->second;
  }
  return ret;
}

std::map<RntiCellIdPair_t, uint32_t>
MmWavePhyTrace::ResetMapValue (std::map<RntiCellIdPair_t, uint32_t> map, 
                               RntiCellIdPair_t key)
{
  auto pair = map.find (key);
  if (pair != map.end ())
  {
    map.erase (pair);
  }
  m_lastReset [key] = Simulator::Now ();
  return map;
}

void
MmWavePhyTrace::RxPacketTraceUeCallback (Ptr<MmWavePhyTrace> phyStats, std::string path, RxPacketTraceParams params)
{
  if (!m_rxPacketTraceFile.is_open ())
    {
      m_rxPacketTraceFile.open (m_rxPacketTraceFilename.c_str ());
      m_rxPacketTraceFile << "DL/UL\ttime\tframe\tsubF\tslot\t1stSym\tsymbol#\tcellId\trnti\tccId\ttbSize\tmcs\trv\tSINR(dB)\tcorrupt\tTBler" << std::endl;
      if (!m_rxPacketTraceFile.is_open ())
        {
          NS_FATAL_ERROR ("Could not open tracefile");
        }
    }
  m_rxPacketTraceFile << "DL\t" << Simulator::Now ().GetSeconds () << "\t" 
                      << params.m_frameNum << "\t" << +params.m_sfNum << "\t" 
                      << +params.m_slotNum << "\t" << +params.m_symStart << "\t" 
                      << +params.m_numSym << "\t" << params.m_cellId << "\t" 
                      << params.m_rnti << "\t" << +params.m_ccId << "\t" 
                      << params.m_tbSize << "\t" << +params.m_mcs << "\t" 
                      << +params.m_rv << "\t" << 10 * std::log10 (params.m_sinr) << "\t" 
                      << params.m_corrupt << "\t" <<  params.m_tbler << std::endl;


  
  phyStats->UpdateTraces(params);

  if (params.m_corrupt)
    {
      NS_LOG_DEBUG ("DL TB error\t" << params.m_frameNum << "\t" << +params.m_sfNum << "\t" 
                                    << +params.m_slotNum << "\t" << +params.m_symStart << "\t" 
                                    << +params.m_numSym << "\t" << params.m_rnti << "\t" 
                                    << +params.m_ccId << "\t" << params.m_tbSize << "\t" 
                                    << +params.m_mcs << "\t" << +params.m_rv << "\t"
                                    << 10 * std::log10 (params.m_sinr) << "\t" << params.m_tbler << "\t" 
                                    << params.m_corrupt);
    }
}

void
MmWavePhyTrace::RxPacketTraceEnbCallback (Ptr<MmWavePhyTrace> phyStats, std::string path, RxPacketTraceParams params)
{
  if (!m_rxPacketTraceFile.is_open ())
    {
      m_rxPacketTraceFile.open (m_rxPacketTraceFilename.c_str ());
      m_rxPacketTraceFile << "DL/UL\ttime\tframe\tsubF\tslot\t1stSym\tsymbol#\tcellId\trnti\tccId\ttbSize\tmcs\trv\tSINR(dB)\tcorrupt\tTBler" << std::endl;
      if (!m_rxPacketTraceFile.is_open ())
        {
          NS_FATAL_ERROR ("Could not open tracefile");
        }
    }
  m_rxPacketTraceFile << "UL\t" << Simulator::Now ().GetSeconds () << "\t" 
                      << params.m_frameNum << "\t" << +params.m_sfNum << "\t" 
                      << +params.m_slotNum << "\t" << +params.m_symStart << "\t" 
                      << +params.m_numSym << "\t" << params.m_cellId << "\t" 
                      << params.m_rnti << "\t" << +params.m_ccId << "\t" 
                      << params.m_tbSize << "\t" << +params.m_mcs << "\t" 
                      << +params.m_rv << "\t" << 10 * std::log10 (params.m_sinr) << " \t" 
                      << params.m_corrupt << "\t" << params.m_tbler << std::endl;

  if (params.m_corrupt)
    {
      NS_LOG_DEBUG ("UL TB error\t" << params.m_frameNum << "\t" << +params.m_sfNum << "\t" 
                                    << +params.m_slotNum << "\t" << +params.m_symStart << "\t" 
                                    << +params.m_numSym << "\t" << params.m_rnti << "\t" 
                                    << +params.m_ccId << "\t" << params.m_tbSize << "\t" 
                                    << +params.m_mcs << "\t" << +params.m_rv << "\t"
                                    << 10 * std::log10 (params.m_sinr) << "\t" << params.m_tbler << "\t" 
                                    << params.m_corrupt << "\t" << params.m_sinrMin);
    }
}


void
MmWavePhyTrace::ResetPhyTracesForRntiCellId(uint16_t rnti, uint16_t cellId)
{
  NS_LOG_LOGIC("Reset rnti " << rnti << " cellId " << cellId);
  RntiCellIdPair_t pair {rnti, cellId};
  
  m_macPduUeSpecific = ResetMapValue (m_macPduUeSpecific, pair);
  
  m_macPduInitialTransmissionUeSpecific = ResetMapValue (m_macPduInitialTransmissionUeSpecific, pair);
  
  m_macPduRetransmissionUeSpecific = ResetMapValue (m_macPduRetransmissionUeSpecific, pair);
  
  m_macVolumeUeSpecific = ResetMapValue (m_macVolumeUeSpecific, pair);
  
  m_macPduQpskUeSpecific = ResetMapValue (m_macPduQpskUeSpecific, pair);
  
  m_macPdu16QamUeSpecific = ResetMapValue (m_macPdu16QamUeSpecific, pair);
  
  m_macPdu64QamUeSpecific = ResetMapValue (m_macPdu64QamUeSpecific, pair);
    
  m_macMcs04UeSpecific = ResetMapValue (m_macMcs04UeSpecific, pair);
  
  m_macMcs59UeSpecific = ResetMapValue (m_macMcs59UeSpecific, pair);
  
  m_macMcs1014UeSpecific = ResetMapValue (m_macMcs1014UeSpecific, pair);
  
  m_macMcs1519UeSpecific = ResetMapValue (m_macMcs1519UeSpecific, pair);
  
  m_macMcs2024UeSpecific = ResetMapValue (m_macMcs2024UeSpecific, pair);
  
  m_macMcs2529UeSpecific = ResetMapValue (m_macMcs2529UeSpecific, pair);

  m_macSinrBin1UeSpecific = ResetMapValue (m_macSinrBin1UeSpecific, pair);

  m_macSinrBin2UeSpecific = ResetMapValue (m_macSinrBin2UeSpecific, pair);

  m_macSinrBin3UeSpecific = ResetMapValue (m_macSinrBin3UeSpecific, pair);

  m_macSinrBin4UeSpecific = ResetMapValue (m_macSinrBin4UeSpecific, pair);

  m_macSinrBin5UeSpecific = ResetMapValue (m_macSinrBin5UeSpecific, pair);

  m_macSinrBin6UeSpecific = ResetMapValue (m_macSinrBin6UeSpecific, pair);

  m_macSinrBin7UeSpecific = ResetMapValue (m_macSinrBin7UeSpecific, pair);
  
  m_macNumberOfSymbols = ResetMapValue(m_macNumberOfSymbols, pair);
}

Time
MmWavePhyTrace::GetLastResetTime (uint16_t rnti, uint16_t cellId)
{
  Time ret = Seconds (0);
  RntiCellIdPair_t key {rnti, cellId};
  if (m_lastReset.find (key) != m_lastReset.end ())
  {
    ret = m_lastReset.at (key);
  }
  return ret;
}

uint32_t 
MmWavePhyTrace::GetMacPduUeSpecific (uint16_t rnti, uint16_t cellId)
{
  RntiCellIdPair_t pair {rnti, cellId};
  return GetMapValue (m_macPduUeSpecific, pair);
}

uint32_t 
MmWavePhyTrace::GetMacPduInitialTransmissionUeSpecific (uint16_t rnti, uint16_t cellId)
{
  RntiCellIdPair_t pair {rnti, cellId};
  return GetMapValue (m_macPduInitialTransmissionUeSpecific, pair);
}

uint32_t 
MmWavePhyTrace::GetMacPduRetransmissionUeSpecific (uint16_t rnti, uint16_t cellId)
{
  RntiCellIdPair_t pair {rnti, cellId};
  return GetMapValue (m_macPduRetransmissionUeSpecific, pair);
}

uint32_t
MmWavePhyTrace::GetMacVolumeUeSpecific (uint16_t rnti, uint16_t cellId)
{
  RntiCellIdPair_t pair {rnti, cellId};
  return GetMapValue (m_macVolumeUeSpecific, pair);
}

uint32_t 
MmWavePhyTrace::GetMacPduQpskUeSpecific (uint16_t rnti, uint16_t cellId)
{
  RntiCellIdPair_t pair {rnti, cellId};
  return GetMapValue (m_macPduQpskUeSpecific, pair);
}

uint32_t 
MmWavePhyTrace::GetMacPdu16QamUeSpecific (uint16_t rnti, uint16_t cellId)
{
  RntiCellIdPair_t pair {rnti, cellId};
  return GetMapValue (m_macPdu16QamUeSpecific, pair);
}

uint32_t 
MmWavePhyTrace::GetMacPdu64QamUeSpecific (uint16_t rnti, uint16_t cellId)
{
  RntiCellIdPair_t pair {rnti, cellId};
  return GetMapValue (m_macPdu64QamUeSpecific, pair);
}

uint32_t 
MmWavePhyTrace::GetMacNumberOfSymbolsUeSpecific (uint16_t rnti, uint16_t cellId)
{
  RntiCellIdPair_t pair {rnti, cellId};
  return GetMapValue (m_macNumberOfSymbols, pair);
}

uint32_t 
MmWavePhyTrace::GetMacMcs04UeSpecific (uint16_t rnti, uint16_t cellId)
{
  RntiCellIdPair_t pair {rnti, cellId};
  return GetMapValue (m_macMcs04UeSpecific, pair);
}

uint32_t 
MmWavePhyTrace::GetMacMcs59UeSpecific (uint16_t rnti, uint16_t cellId)
{
  RntiCellIdPair_t pair {rnti, cellId};
  return GetMapValue (m_macMcs59UeSpecific, pair);
}

uint32_t 
MmWavePhyTrace::GetMacMcs1014UeSpecific (uint16_t rnti, uint16_t cellId)
{
  RntiCellIdPair_t pair {rnti, cellId};
  return GetMapValue (m_macMcs1014UeSpecific, pair);
}

uint32_t 
MmWavePhyTrace::GetMacMcs1519UeSpecific (uint16_t rnti, uint16_t cellId)
{
  RntiCellIdPair_t pair {rnti, cellId};
  return GetMapValue (m_macMcs1519UeSpecific, pair);
}

uint32_t 
MmWavePhyTrace::GetMacMcs2024UeSpecific (uint16_t rnti, uint16_t cellId)
{
  RntiCellIdPair_t pair {rnti, cellId};
  return GetMapValue (m_macMcs2024UeSpecific, pair);
}

uint32_t 
MmWavePhyTrace::GetMacMcs2529UeSpecific (uint16_t rnti, uint16_t cellId)
{
  RntiCellIdPair_t pair {rnti, cellId};
  return GetMapValue (m_macMcs2529UeSpecific, pair);
}

uint32_t 
MmWavePhyTrace::GetMacSinrBin1UeSpecific (uint16_t rnti, uint16_t cellId)
{
  RntiCellIdPair_t pair {rnti, cellId};
  return GetMapValue (m_macSinrBin1UeSpecific, pair);
}

uint32_t 
MmWavePhyTrace::GetMacSinrBin2UeSpecific (uint16_t rnti, uint16_t cellId)
{
  RntiCellIdPair_t pair {rnti, cellId};
  return GetMapValue (m_macSinrBin2UeSpecific, pair);
}

uint32_t 
MmWavePhyTrace::GetMacSinrBin3UeSpecific (uint16_t rnti, uint16_t cellId)
{
  RntiCellIdPair_t pair {rnti, cellId};
  return GetMapValue (m_macSinrBin3UeSpecific, pair);
}

uint32_t 
MmWavePhyTrace::GetMacSinrBin4UeSpecific (uint16_t rnti, uint16_t cellId)
{
  RntiCellIdPair_t pair {rnti, cellId};
  return GetMapValue (m_macSinrBin4UeSpecific, pair);
}

uint32_t 
MmWavePhyTrace::GetMacSinrBin5UeSpecific (uint16_t rnti, uint16_t cellId)
{
  RntiCellIdPair_t pair {rnti, cellId};
  return GetMapValue (m_macSinrBin5UeSpecific, pair);
}

uint32_t 
MmWavePhyTrace::GetMacSinrBin6UeSpecific (uint16_t rnti, uint16_t cellId)
{
  RntiCellIdPair_t pair {rnti, cellId};
  return GetMapValue (m_macSinrBin6UeSpecific, pair);
}

uint32_t 
MmWavePhyTrace::GetMacSinrBin7UeSpecific (uint16_t rnti, uint16_t cellId)
{
  RntiCellIdPair_t pair {rnti, cellId};
  return GetMapValue (m_macSinrBin7UeSpecific, pair);
}


} // namespace mmwave

} /* namespace ns3 */
