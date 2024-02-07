
tbs-fusion-py
=============

Overview
--------

tbs-fusion-py is a Python package to control the [TBS Fusion](https://www.team-blacksheep.com/products/prod:tbs_fusion)
analog video receiver module. The package uses the "TBS Fusion Serial Protocol" to control one or multiple
TBS Fusion devices over a single wire or RS-485 serial connection. The protocol document on the page
linked above contains detailed information on how to connect and configure the TBS Fusion so that
it can be used with this package.

Functionality
-------------

The following Functionality is currently supported:

- Setting the operating frequency.
- Getting the current operating frequency and RSSI (Received Signal Strength Indicator) of each receiver.
- Acquiring the RSSI for a frequency range.
- Acquiring the RSSI for a list of frequencies.

Examples
--------

Examples can be found in the [examples directory](https://github.com/BrainFPV/tbs-fusion-py/blob/master/examples).

Installation
------------

You can use pip to install this package from PyPi:

    pip install tbs-fusion-py
