from dataclasses import dataclass
from typing import Optional


@dataclass
class Ue:
    ue_id: int
    x_position: float
    y_position: float
    type: str
    LTE_Cell: int
    MMWave_Cell: int
    ErrTotalNbrDl: Optional[float] = None
    DRB_BufferSize_Qos: Optional[float] = None
    RRU_PrbUsedDl: Optional[float] = None
    TP_Combined_PDCP_ENDC_kbps: Optional[float] = None
    TP_Combined_RLC_ENDC_kbps: Optional[float] = None
    QosFlow_PdcpPduVolumeDl: Optional[float] = None
    TotNbrDlInitial: Optional[float] = None
    TotNbrDlInitial_16Qam: Optional[float] = None
    TotNbrDlInitial_64Qam: Optional[float] = None
    TotNbrDlInitial_Qpsk: Optional[float] = None
    TotNbrDl: Optional[float] = None
    PDCP_PDU_Volume: Optional[float] = None
    PdcpSduDelayDl_ms_x_0_1: Optional[float] = None
    Qos_PDCP_PDU: Optional[float] = None
    PDCP_PDU: Optional[float] = None
    PDCP_Throughput_kbps: Optional[float] = None
    Tx_Bytes: Optional[float] = None
    L3servingSINR_dB: Optional[float] = None
    L3servingSINR_CellID: Optional[float] = None
    L3neighSINR_dB: Optional[float] = None
    L3neighSINR_CellId: Optional[float] = None
    DRB_Estab_Succ_5QI: Optional[float] = None
