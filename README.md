# gr-wifi-dsss
A Gnuradio-based implementation of WiFi direct sequence spread spectrum communications

# Demo

[![gr-wifi-dsss demo video](http://img.youtube.com/vi/u0h_xftyMqk/0.jpg)](http://www.youtube.com/watch?v=u0h_xftyMqk)

# Installation

- prerequisite:

    gr-gadget

- install gr-wifi-dsss:
```
cd gr-wifi-dsss/
mkdir build
cd build
cmake ..
make
sudo make install
```
# Quick start 

`cd examples` and run the scripts in the folder. 

# Examples
- 'usrp_selfloop.grc': A USRP transceiver script, for one USRP
- 'usrp_dsss_tx.grc', 'usrp_dsss_rx.grc': Operated with two USRPs

# Supported Rates
The prototype supports both long and short preamble and the following data rates:
- 1Mbps (long preamble only)
- 2Mbps
- 5.5Mbps
- 11Mbps

# Notes
- The data rate depends on the sampling rate you specified in Gnuradio, and may not meet the nominal rates.
