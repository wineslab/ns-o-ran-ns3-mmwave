import os
import time

from influxdb import InfluxDBClient

from src.simulation_objects.cell import Cell
from src.simulation_objects.ue import Ue


class Simulation:

    def __init__(self, number_of_ues: int, number_of_cells: int):
        self.db_client = InfluxDBClient(
            host=os.getenv("INFLUXDB_HOST"),
            port=int(os.getenv("INFLUXDB_PORT")),
            username=os.getenv("INFLUXDB_USERNAME"),
            password=os.getenv("INFLUXDB_PASSWORD"),
            database=os.getenv("INFLUXDB_DATABASE"),
        )
        self.simulation_start_time = time.time_ns()
        self.number_of_ues = number_of_ues
        self.number_of_cells = number_of_cells
        if number_of_ues > 0 and number_of_cells > 0:
            self.max_x, self.max_y = self.get_charts_max_axis_value()
            self.ues, self.cells = self.get_simulation_data(self.number_of_ues, self.number_of_cells)
        else:
            self.max_x = 6000
            self.max_y = 6000
            self.ues = []
            self.cells = []

    def get_charts_max_axis_value(self):
        max_x = self.get_last_value_from_measurement('gnbs_x_0')
        if not max_x:
            max_x = 6000
        max_y = self.get_last_value_from_measurement('gnbs_y_0')
        if not max_y:
            max_y = 6000
        return (int(max_x), int(max_y))

    def get_simulation_data(self, number_of_ues: int, number_of_cells: int) -> (list[Ue], list[Cell]):
        ues = []
        cells = []
        for ue_id in range(1, number_of_ues + 1):
            ue = Ue(
                ue_id=ue_id,
                x_position=self.get_last_value_from_measurement(f'ue_position_x_{ue_id}'),
                y_position=self.get_last_value_from_measurement(f'ue_position_y_{ue_id}'),
                type=self.get_last_value_from_measurement(f'ue_position_type_{ue_id}'),
                MMWave_Cell=int(self.get_last_value_from_measurement(f'ue_position_cell_{ue_id}') or 0),
                LTE_Cell=int(1),
                ErrTotalNbrDl=self.get_last_value_from_measurement(f'ue_{ue_id}_tb.errtotalnbrdl.1.ueid'),
                DRB_BufferSize_Qos=self.get_last_value_from_measurement(f'ue_{ue_id}_drb.buffersize.qos.ueid'),
                RRU_PrbUsedDl=self.get_last_value_from_measurement(f'ue_{ue_id}_rru.prbuseddl'),
                TP_Combined_PDCP_ENDC_kbps=self.get_last_value_from_measurement(f'ue_{ue_id}_drb.uethpdlpdcpbased.ueid'),
                TP_Combined_RLC_ENDC_kbps=self.get_last_value_from_measurement(f'ue_{ue_id}_drb.uethpdl.ueid'),
                QosFlow_PdcpPduVolumeDl=self.get_last_value_from_measurement(
                    f'ue_{ue_id}_qosflow.pdcppduvolumedl_filter'),
                TotNbrDlInitial=self.get_last_value_from_measurement(f'ue_{ue_id}_tb.totnbrdlinitial'),
                TotNbrDlInitial_16Qam=self.get_last_value_from_measurement(f'ue_{ue_id}_tb.totnbrdlinitial.16qam'),
                TotNbrDlInitial_64Qam=self.get_last_value_from_measurement(f'ue_{ue_id}_tb.totnbrdlinitial.64qam'),
                TotNbrDlInitial_Qpsk=self.get_last_value_from_measurement(f'ue_{ue_id}_tb.totnbrdlinitial.qpsk.ueid'),
                TotNbrDl=self.get_last_value_from_measurement(f'ue_{ue_id}_tb.totnbrdl.1.ueid'),
                PDCP_PDU_Volume=self.get_last_value_from_measurement(
                    f'ue_{ue_id}_qosflow_pdcppduvolumedl_filter_ueid(txpdcppdubytesnrrlc)'),
                PdcpSduDelayDl_ms_x_0_1=self.get_last_value_from_measurement(
                    f'ue_{ue_id}_drb.pdcpsdudelaydl.ueid(pdcp latency)'),
                Qos_PDCP_PDU=self.get_last_value_from_measurement(
                    f'ue_{ue_id}_drb_pdcppdunbrdl_qos_ueid(txpdcppdunrrlc)'),
                PDCP_PDU=self.get_last_value_from_measurement(f'ue_{ue_id}_tot_pdcpsdunbrdl_ueid(txdlpackets)'),
                PDCP_Throughput_kbps=self.get_last_value_from_measurement(
                    f'ue_{ue_id}_drb.pdcpsdubitratedl.ueid(pdcpthroughput)'),
                Tx_Bytes=self.get_last_value_from_measurement(f'ue_{ue_id}_drb.pdcpsduvolumedl_filter.ueid(txbytes)'),
                L3servingSINR_dB=self.get_last_value_from_measurement(f'ue_{ue_id}_l3 serving sinr'),
                L3servingSINR_CellID=self.get_last_value_from_measurement(f'ue_position_cell_{ue_id}'),
                L3neighSINR_dB=self.get_max_value_from_multiple_measurements(f'ue_{ue_id}_l3 neigh sinr ', 8),
                L3neighSINR_CellId=self.get_max_value_from_multiple_measurements(
                    f'ue_{ue_id}_l3 neigh id ',
                    8,
                    '(cellid)'),
                DRB_Estab_Succ_5QI=self.get_last_value_from_measurement(f'ue_{ue_id}_drb.estabsucc.5qi.ueid')
            )
            ues.append(ue)
        for cell_id in range(1, number_of_cells + 1):
            cell_x, cell_y, cell_type = self.get_cell_position_and_type(cell_id)
            cell = Cell(
                cell_id=cell_id,
                x_position=cell_x,
                y_position=cell_y,
                type=cell_type,
                ErrTotalNbrDl=self.get_last_value_from_measurement(f'du-cell-{cell_id}_tb.errtotalnbrdl.1.ueid'),
                MeanActiveUEsDownlink=self.get_last_value_from_measurement(f'du-cell-{cell_id}_drb.meanactiveuedl'),
                DRB_BufferSize_Qos=self.get_last_value_from_measurement(f'du-cell-{cell_id}_drb.buffersize.qos.ueid'),
                RRU_PrbUsedDl=self.get_last_value_from_measurement(f'du-cell_{cell_id}_rru.prbuseddl'),
                QosFlow_PdcpPduVolumeDl=self.get_last_value_from_measurement(
                    f'du-cell-{cell_id}_qosflow.pdcppduvolumedl_filter'),
                TotNbrDlInitial=self.get_last_value_from_measurement(f'du-cell-{cell_id}_tb.totnbrdlinitial'),
                TotNbrDlInitial_16Qam=self.get_last_value_from_measurement(
                    f'du-cell-{cell_id}_tb.totnbrdlinitial.16qam'),
                TotNbrDlInitial_64Qam=self.get_last_value_from_measurement(
                    f'du-cell-{cell_id}_tb.totnbrdlinitial.64qam'),
                TotNbrDlInitial_Qpsk=self.get_last_value_from_measurement(
                    f'du-cell-{cell_id}_tb.totnbrdlinitial.qpsk.ueid'),
                TotNbrDl=self.get_last_value_from_measurement(f'du-cell-{cell_id}_tb.totnbrdl.1.ueid'),
                dlPrbUsage_percentage=self.get_last_value_from_measurement(f'du-cell-{cell_id}_dlprbusage'),
                Cell_Average_Latency_ms_x_0_1=self.get_last_value_from_measurement(
                    f'cu-up-cell-{cell_id}_drb.pdcpsdudelaydl (cellaveragelatency)'),
                LTE_Cell_PDCP_Volume=self.get_last_value_from_measurement(
                    f'cu-up-cell-{cell_id}_m_pdcpbytesdl (celldltxvolume)'),
            )
            cells.append(cell)
        return ues, cells

    def get_last_value_from_measurement(self, measurement_name: str) -> float | str | int | None:
        try:
            result = self.db_client.query(f'SELECT LAST("value") FROM "{measurement_name}" where time > {self.simulation_start_time}')
            if result:
                points = list(result.get_points(measurement=measurement_name))
                value = points[0].get('last')
                if isinstance(value, float) and value.is_integer():
                    return int(value)
                return value
            else:
                return None
        except Exception as e:
            raise Exception(f"Error querying InfluxDB: {e}")

    def get_max_value_from_multiple_measurements(
            self,
            measurement_name: str,
            ids: int,
            suffix='') -> float | int | None:
        values = []
        for i in range(1, ids + 1):
            if suffix != '':
                meas_name = f'{measurement_name}{i} {suffix}'
            else:
                meas_name = f'{measurement_name}{i}'
            new_value = self.get_last_value_from_measurement(meas_name)
            values.append(new_value)
        numerics = []
        for value in values:
            try:
                numeric_value = float(value)
                numerics.append(numeric_value)
            except (ValueError, TypeError):
                continue
        if numerics:
            max_value = max(numerics)
            return int(max_value) if max_value.is_integer() else max_value
        else:
            return None

    def get_cell_position_and_type(self, cell_id: int) -> (int, int, str):
        x_val = self.get_last_value_from_measurement(f'enbs_x_{cell_id}')
        if x_val:
            y_val = self.get_last_value_from_measurement(f'enbs_y_{cell_id}')
            cell_type = 'lte'
        else:
            x_val = self.get_last_value_from_measurement(f'gnbs_x_{cell_id}')
            y_val = self.get_last_value_from_measurement(f'gnbs_y_{cell_id}')
            cell_type = 'mmwave'

        return x_val, y_val, cell_type
