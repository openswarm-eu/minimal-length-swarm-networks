# Based on https://www.datadoghq.com/blog/engineering/protobuf-parsing-in-python/

import os
from pprint import pprint
import sys
 
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "protos", "generated"))
import time_step_pb2

from google.protobuf.internal.decoder import _DecodeVarint32
 
def main():

    s = {}

    with open('<path-to-file>/minimal-length-swarm-networks/results/network_maintenance_comm/network_maintenance_5T_12R_0.60/log_data.pb', 'rb') as f:
        buf = f.read()
        n = 0
        while n < len(buf):
            msg_len, new_pos = _DecodeVarint32(buf, n)
            n = new_pos
            msg_buf = buf[n:n+msg_len]
            n += msg_len
            read_metric = time_step_pb2.TimeStep()
            read_metric.ParseFromString(msg_buf)
            # print(read_metric)
            s[read_metric.time] = read_metric
 
    print(s[1500].robots)

if __name__ == "__main__":
    main()
