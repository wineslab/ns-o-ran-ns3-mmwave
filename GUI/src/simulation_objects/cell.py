from dataclasses import dataclass
from typing import Optional


@dataclass
class Cell:
    cell_id: int
    x_position: float
    y_position: float
    type: str
    ErrTotalNbrDl: Optional[int] = None
    MeanActiveUEsDownlink: Optional[int] = None
    DRB_BufferSize_Qos: Optional[float] = None
    RRU_PrbUsedDl: Optional[float] = None
    QosFlow_PdcpPduVolumeDl: Optional[float] = None
    TotNbrDlInitial: Optional[int] = None
    TotNbrDlInitial_16Qam: Optional[int] = None
    TotNbrDlInitial_64Qam: Optional[int] = None
    TotNbrDlInitial_Qpsk: Optional[int] = None
    TotNbrDl: Optional[int] = None
    dlPrbUsage_percentage: Optional[float] = None
    Cell_Average_Latency_ms_x_0_1: Optional[float] = None
    LTE_Cell_PDCP_Volume: Optional[float] = None
