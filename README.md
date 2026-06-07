# gr-wifi-dsss
A Gnuradio-based implementation of Wi-Fi direct sequence spread spectrum communications

> **⚠️ WARNING:** 
> The upgraded code (for GNU Radio 3.10) has **only been tested in a simulation environment**. 
> It has not been verified on actual hardware (e.g., USRP). 
> Please exercise caution and expect potential issues if you intend to use this for over-the-air transmission.

# Demo

[![gr-wifi-dsss demo video](http://img.youtube.com/vi/u0h_xftyMqk/0.jpg)](http://www.youtube.com/watch?v=u0h_xftyMqk)

# Installation

- install Gnuradio 3.10
see [InstallingGR](https://wiki.gnuradio.org/index.php/InstallingGR) for details

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
- 'simulation_txrx.grc': For a simple simulation with a TX-RX loop.

# Supported Rates
The prototype supports both long and short preamble and the following data rates:
- 1Mbps (long preamble only)
- 2Mbps
- 5.5Mbps
- 11Mbps

# Notes
- The data rate depends on the sampling rate you specified in Gnuradio, and may not meet the nominal rates.
