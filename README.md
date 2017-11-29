# gr-wifi-dsss
A Gnuradio-based implementation of WiFi direct sequence spread spectrum communications

# Quick Start
1. Installation
```
mkdir build
cd build
cmake ..
make
sudo make install
```
2. Examples

`cd examples` and run the scripts in the folder.
# Supported Rates
The prototype supports both long and short preamble and the following data rates:
- 1Mbps (long preamble only)
- 2Mbps
- 5.5Mbps
- 11Mbps

# Notes
- The data rate depends on the sampling rate you specified in Gnuradio, and may not meet the nominal rates.
