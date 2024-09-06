import os
import time
from influxdb import InfluxDBClient


def read_and_clear_csv(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    headers = lines[0].strip()
    data = [line.strip() for line in lines[1:]]

    result = [headers] + data

    with open(file_path, 'w') as file:
        file.write(headers + '\n')

    return result


def push_data_to_influx(
        file_path,
        db_name,
        core_files,
        influx_host='localhost',
        influx_port=8086,
        influx_user='root',
        influx_password='root'):
    data = read_and_clear_csv(file_path)
    ue_fields = {
        'l3 serving id(m_cellid)',
        'drb.estabsucc.5qi.ueid',
        'l3 neigh id 1 (cellid)',
        'l3 neigh id 2 (cellid)',
        'l3 neigh id 3 (cellid)',
        'l3 neigh id 4 (cellid)',
        'l3 neigh id 5 (cellid)',
        'l3 neigh id 6 (cellid)',
        'l3 neigh id 7 (cellid)',
        'l3 neigh id 8 (cellid)',
        'l3 serving sinr',
        'l3 neigh sinr 1',
        'l3 neigh sinr 2',
        'l3 neigh sinr 3',
        'l3 neigh sinr 4',
        'l3 neigh sinr 5',
        'l3 neigh sinr 6',
        'l3 neigh sinr 7',
        'l3 neigh sinr 8',
        'tb.errtotalnbrdl.1.ueid',
        'drb.buffersize.qos.ueid',
        'drb.uethpdl.ueid',
        'rru.prbuseddl',
        'drb.uethpdlpdcpbased.ueid',
        'qosflow.pdcppduvolumedl_filter',
        'tb.totnbrdlinitial',
        'tb.totnbrdlinitial.16qam',
        'tb.totnbrdlinitial.64qam',
        'tb.totnbrdlinitial.qpsk.ueid',
        'tb.totnbrdl.1.ueid',
        'dlprbusage',
        'qosflow_pdcppduvolumedl_filter_ueid(txpdcppdubytesnrrlc)',
        'drb.pdcpsdudelaydl.ueid(pdcp latency)',
        'drb_pdcppdunbrdl_qos_ueid(txpdcppdunrrlc)',
        'tot_pdcpsdunbrdl_ueid(txdlpackets)',
        'drb.pdcpsdubitratedl.ueid(pdcpthroughput)',
        'drb.pdcpsduvolumedl_filter.ueid(txbytes)'
    }

    cell_fields = {
        'tb.errtotalnbrdl.1.ueid',
        'drb.buffersize.qos.ueid',
        'rru.prbuseddl',
        'drb.meanactiveuedl',
        'qosflow.pdcppduvolumedl_filter',
        'tb.totnbrdlinitial',
        'tb.totnbrdlinitial.16qam',
        'tb.totnbrdlinitial.64qam',
        'tb.totnbrdlinitial.qpsk.ueid',
        'tb.totnbrdl.1.ueid',
        'drb.pdcpsdudelaydl (cellaveragelatency)',
        'm_pdcpbytesdl (celldltxvolume)',
        'dlprbusage'
    }

    client = InfluxDBClient(
        host=influx_host,
        port=influx_port,
        username=influx_user,
        password=influx_password,
        database=db_name)

    client.create_database(db_name)

    headers = data[0].split(',')
    headers = [header.strip().lower() for header in headers]
    records = data[1:]

    id_column = None
    possible_id_columns = ["imsi", "id", "ueimsicomplete", "ueImsiComplete"]

    for possible_id in possible_id_columns:
        if possible_id in headers:
            id_column = possible_id
            break

    influx_points = []
    filename = os.path.splitext(os.path.basename(file_path))[0]

    for record in records:
        fields = record.split(',')
        if file_path in core_files:
            if id_column is None:
                print(f"Error: Neither 'imsi', 'id', nor 'ueImsiComplete' found in the headers of {file_path}")
                return

            id_index = headers.index(id_column)
            record_id = fields[id_index]

            for i, field in enumerate(fields):
                if i == id_index or headers[i] == "timestamp":
                    continue

                try:
                    value = float(field) if field.replace('.', '', 1).isdigit() else field
                except ValueError:
                    print(f"Skipping invalid value in file '{file_path}' for field '{headers[i]}': {field}")
                    continue

                measurement = f"{filename}_{headers[i]}_{record_id}"

                influx_point = {
                    "measurement": measurement,
                    "tags": {
                        id_column: record_id
                    },
                    "fields": {
                        "value": value
                    }
                }
                influx_points.append(influx_point)

        else:
            for i, field in enumerate(fields):
                if headers[i] == "timestamp":
                    continue
                try:
                    if headers[i] == 'l3 serving sinr':
                        value = str(field)
                    else:
                        value = float(field) if field.replace('.', '', 1).isdigit() else field
                except ValueError:
                    print(f"Skipping invalid value in file '{file_path}' for field '{headers[i]}': {field}")
                    continue
                if headers[i] in ue_fields and headers[i] not in cell_fields:
                    if headers[i] in ('l3 neigh id 1 (cellid)', 'l3 neigh id 2 (cellid)', 'l3 neigh id 3 (cellid)',
                                      'l3 neigh id 4 (cellid)', 'l3 neigh id 5 (cellid)', 'l3 neigh id 6 (cellid)',
                                      'l3 neigh id 7 (cellid)', 'l3 neigh id 8 (cellid)'):
                        if value != '':
                            value = int(value)
                        else:
                            value = 0
                    if headers[i] in ('l3 neigh sinr 1', 'l3 neigh sinr 2', 'l3 neigh sinr 3', 'l3 neigh sinr 4'
                                      'l3 neigh sinr 5', 'l3 neigh sinr 6', 'l3 neigh sinr 7', 'l3 neigh sinr 8'):
                        if value == '':
                            value = -999
                    measurement = f"ue_{int(fields[1])}_{headers[i]}"
                    influx_point = {
                        "measurement": measurement,
                        "fields": {
                            "value": value
                        }
                    }
                    influx_points.append(influx_point)
                elif headers[i] in cell_fields and headers[i] not in ue_fields:
                    if headers[i] == 'drb.meanactiveuedl':
                        print(f'current_mean_val = {value}')
                        if not value or value == '':
                            value = 0
                    measurement = f"{filename}_{headers[i]}"
                    influx_point = {
                        "measurement": measurement,
                        "fields": {
                            "value": value
                        }
                    }
                    influx_points.append(influx_point)
                elif headers[i] in cell_fields and headers[i] in ue_fields:
                    measurement = f"ue_{int(fields[1])}_{headers[i]}"
                    influx_point = {
                        "measurement": measurement,
                        "fields": {
                            "value": value
                        }
                    }
                    influx_points.append(influx_point)
                    measurement = f"{filename}_{headers[i]}"
                    influx_point = {
                        "measurement": measurement,
                        "fields": {
                            "value": value
                        }
                    }
                    influx_points.append(influx_point)

    try:
        if influx_points:
            client.write_points(influx_points)
            print(f"Successfully pushed data to InfluxDB for {len(influx_points)} points from {file_path}.")
        else:
            print(f"No valid data to push for file {file_path}.")
    except Exception as e:
        print(f"Failed to write data from {file_path} to InfluxDB: {e}")


def main():
    influx_host = 'localhost'
    influx_port = 8086
    influx_user = 'root'
    influx_password = 'root'
    db_name = 'influx'

    core_files = [
        "ue_position.txt",
        "gnbs.txt",
        "enbs.txt"
    ]

    while True:
        additional_files = [
            file for file in os.listdir('.')
            if file.startswith(('cu-cp-cell-', 'cu-up-cell-', 'du-cell-')) and file.endswith('.txt')
        ]

        files_to_process = core_files + additional_files

        for file_path in files_to_process:
            if os.path.exists(file_path):
                push_data_to_influx(
                    file_path,
                    db_name,
                    core_files,
                    influx_host,
                    influx_port,
                    influx_user,
                    influx_password)

        time.sleep(3)


if __name__ == "__main__":
    main()
