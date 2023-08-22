#!/usr/bin/python3
import re
import sys
import math

import argparse

PRF_CONST_16MHZ = 113.8
PRF_CONST_64MHZ_STS = 120.7
PRF_CONST_64MHZ_IPATOV = 121.7

C0_2_POW = 21
D0_E0_2_POW = 17

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('-t', type=argparse.FileType('rt'), help='text trace file')
p.add_argument('-r', action='store_const', const=1, help='print raw lines')
p.add_argument('-v', action='store_const', const=1, help='verbose, print intermediate values after RSSI')
options = p.parse_args()

if options.t:
    trace = options.t
else:
    trace = sys.stdin

for line in trace:
    line = line.rstrip()
    if options.r:
        print(line)
    m = re.search(r"dw3000_rx_rssi: spi2.0, chip: ([C-E]0) sts: ([0-9]) cir_pwr: ([0-9]+) pacc_cnt: ([0-9]+) prf_64mhz: ([0|1]) dgc_dec: ([0-9])", line)
    if m:
        chip = m.group(1)
        sts = int(m.group(2), 10)
        cir_pwr = int(m.group(3), 10)
        pacc_cnt = int(m.group(4), 10)
        prf_64mhz = int(m.group(5), 10)
        dgc_dec = int(m.group(6), 10)

        if prf_64mhz:
            if sts > 0:
                prf_const = PRF_CONST_64MHZ_STS
            else:
                prf_const = PRF_CONST_64MHZ_IPATOV
        else:
            prf_const = PRF_CONST_16MHZ

        if chip == "C0":
            two_pow = 21
        elif chip == "D0" or chip == "E0":
            two_pow = 17
        else:
            print("Invalid chip version: ", chip)
            sys.exit(1)

        rssi = 10 * math.log((cir_pwr * math.pow(2, two_pow)) / math.pow(pacc_cnt, 2)) - prf_const
        #print("intermediate: ", rssi)

        if prf_64mhz:
            rssi += (6 * dgc_dec)

        print("RSSI: %.3f" % rssi, end='')
        if options.v:
            print(" (sts: %d cir_pwr %d pacc_cnt %d prf_64mhz %d dgc_dec %d)" %
                    (sts, cir_pwr, pacc_cnt, prf_64mhz, dgc_dec), end='')
        print()
