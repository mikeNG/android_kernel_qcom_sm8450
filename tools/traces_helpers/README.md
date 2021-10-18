# Traces Helpers

This folder contains helpers scripts that can parse and interpret kernel traces.

## RSSI - check_rssy_traces.py

### Enable traces

``` bash
echo 1 > /sys/kernel/tracing/events/dw3000/dw3000_rx_rssi/enable
echo 1 > /sys/kernel/tracing/tracing_on
tee -a traces.log < /sys/kernel/tracing/trace_pipe &
```

### Usage

From a file:

```
./check_rssi.py -t traces.log
RSSI: -23.165
RSSI: -23.499
RSSI: -24.735
RSSI: -15.914
```

From a pipe through `adb shell`, adding the parsed valued used for calculation:

```
adb shell "cat /sys/kernel/tracing/trace_pipe" | ./check_rss.py -v
RSSI: -23.165 (sts: 0 cir_pwr 53 pacc_cnt 47 prf_64mhz 1 dgc_dec 3)
RSSI: -23.499 (sts: 1 cir_pwr 86 pacc_cnt 64 prf_64mhz 1 dgc_dec 3)
RSSI: -24.735 (sts: 1 cir_pwr 76 pacc_cnt 64 prf_64mhz 1 dgc_dec 3)
RSSI: -15.914 (sts: 0 cir_pwr 208 pacc_cnt 48 prf_64mhz 1 dgc_dec 2)
```

Adding all raw lines, to interleave parsing with other traces for example:

```
./check_rssi.py -t traces.log -v -r
   dw3000-spi2.0-3496    [004] .... 62524.797453: dw3000_rx_rssi: spi2.0, chip: E0 sts: 0 cir_pwr: 53 pacc_cnt: 47 prf_64mhz: 1 dgc_dec: 3
RSSI: -23.165 (sts: 0 cir_pwr 53 pacc_cnt 47 prf_64mhz 1 dgc_dec 3)
   dw3000-spi2.0-3496    [004] .... 62524.799346: dw3000_rx_rssi: spi2.0, chip: E0 sts: 1 cir_pwr: 86 pacc_cnt: 64 prf_64mhz: 1 dgc_dec: 3
RSSI: -23.499 (sts: 1 cir_pwr 86 pacc_cnt 64 prf_64mhz 1 dgc_dec 3)
   dw3000-spi2.0-3496    [004] .... 62524.803344: dw3000_rx_rssi: spi2.0, chip: E0 sts: 1 cir_pwr: 76 pacc_cnt: 64 prf_64mhz: 1 dgc_dec: 3
RSSI: -24.735 (sts: 1 cir_pwr 76 pacc_cnt 64 prf_64mhz 1 dgc_dec 3)
   dw3000-spi2.0-3496    [004] .... 62524.805421: dw3000_rx_rssi: spi2.0, chip: E0 sts: 0 cir_pwr: 208 pacc_cnt: 48 prf_64mhz: 1 dgc_dec: 2
RSSI: -15.914 (sts: 0 cir_pwr 208 pacc_cnt 48 prf_64mhz 1 dgc_dec 2)
```
