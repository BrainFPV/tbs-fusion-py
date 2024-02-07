"""Class to interface with the TBS Fusion over a serial connection"""

# Authors: Martin Luessi <mluessi@gmail.com>
#
# License: BSD-3-Clause
# Copyright BrainFPV LLC 2024


import time
import ctypes
from enum import Enum

import serial
from pycrc.algorithms import Crc

MSG_SYNC_VALUE_0 = 0xAA
MSG_SYNC_VALUE_1 = 0x55


class MsgHeader(ctypes.Structure):
    _pack_ = 1
    _fields_ = [('sync_0', ctypes.c_uint8),
                ('sync_1', ctypes.c_uint8),
                ('crc', ctypes.c_uint16),
                ('address', ctypes.c_uint8),
                ('msg_type', ctypes.c_uint8),
                ('length', ctypes.c_uint8)]


class MsgType(Enum):
    ACK = 0
    FREQUENCY_RSSI_REQUEST = 1
    FREQUENCY_RSSI_RESPONSE = 2
    COMMAND_SET_FREQUENCY = 3
    FREQUENCY_RANGE_SCAN_REQUEST = 4
    FREQUENCY_LIST_SCAN_REQUEST = 5
    FREQUENCY_SCAN_RESPONSE = 6


class MsgDataAck(ctypes.Structure):
    _pack_ = 1
    _fields_ = [('msg_type', ctypes.c_uint8),
                ('error_code', ctypes.c_uint8)]


class MsgDataFreqRssi(ctypes.Structure):
    _pack_ = 1
    _fields_ = [('frequency', ctypes.c_uint16),
                ('rssi_a', ctypes.c_uint8),
                ('rssi_b', ctypes.c_uint8)]


class MsgDataSetFreq(ctypes.Structure):
    _pack_ = 1
    _fields_ = [('frequency', ctypes.c_uint16)]


class MsgDataFreqRangeScan(ctypes.Structure):
    _pack_ = 1
    _fields_ = [('freq_start', ctypes.c_uint16),
                ('freq_stop', ctypes.c_uint16),
                ('freq_step', ctypes.c_uint8),
                ('rx_use', ctypes.c_uint8),
                ('delay_ms', ctypes.c_uint8)]


class MsgDataFreqListScan(ctypes.Structure):
    _pack_ = 1
    _fields_ = [('rx_use', ctypes.c_uint8),
                ('delay_ms', ctypes.c_uint8)]


class TBSFusion:
    """
    A class to control one or multiple TBS Fusion analog video
    receiver modules over a single wire or RS-485 serial connection.
    """
    def __init__(self, port, baudrate=115200, default_address=1,
                 timeout=0.1, discard_echo=False, debug=False):
        """
        Parameters
        ----------
        port : str
            Serial port to use. E.g. 'COM1' or '/dev/ttyUSB0'.
        baudrate :
            Baudrate to use for serial connection.
            The baudrate of the TBS Fusion can be configured
            under "Serial Baud" in the settings.
        default_address : int
            Address to use for TBS Fusion if not provided
            when calling methods. The address of the TBS Fusion
            can be configured under "Serial Addr" in the settings.
        timeout : float
            Default timeout in seconds when waiting for responses.
        discard_echo : bool
            Discard the bytes sent from the input buffer. Needs to be
            enabled if the serial interface receives the data it sends.
        debug : bool
            Print debug information.
        """
        self._timeout = timeout
        self._default_address = default_address
        self._debug = debug

        # Discards the received data that has just been transmitted
        self._discard_echo = discard_echo

        # Use CRC16_CCITT_FALSE
        self._crc_alg = Crc(width=16, poly=0x1021,
                            reflect_in=False, xor_in=0xFFFF,
                            reflect_out=False, xor_out=0x0000)

        # Open the serial port
        self._sio = serial.Serial(port=port, baudrate=baudrate,
                                  timeout=timeout, bytesize=8,
                                  parity=serial.PARITY_NONE, stopbits=1)

    def _send_message(self, address, msg_type, msg_data=None):
        """Send a message"""
        if msg_data is not None:
            msg_data = bytearray(msg_data)
        else:
            msg_data = bytearray()

        if address is None:
            address = self._default_address

        # Create header
        header = MsgHeader()
        header.sync_0 = MSG_SYNC_VALUE_0
        header.sync_1 = MSG_SYNC_VALUE_1
        header.address = address
        header.msg_type = msg_type.value
        header.length = len(msg_data)

        tx_data = bytearray(header)
        tx_data.extend(bytearray(msg_data))

        # Calculate CRC
        header.crc = self._crc_alg.bit_by_bit_fast(tx_data[4:])

        # Message with CRC
        tx_data = bytearray(header)
        tx_data.extend(bytearray(msg_data))

        if self._debug:
            print('TX MSG: %s data len: %d CRC: 0x%04X'
                  % (msg_type.name, header.length, header.crc))
            print('Data: %s' % (' '.join('%02X' % x for x in tx_data)))

        # Make sure input buffer is empty
        self._sio.reset_input_buffer()

        # Send the data
        n_sent = self._sio.write(tx_data)
        self._sio.flush()

        if self._discard_echo:
            self._sio.read(n_sent)

    def _receive_message(self, address, msg_type, payload_length,
                         timeout=None):
        """Receive a message"""
        if timeout is not None:
            # Change receive timeout for this message
            self._sio.timeout = timeout

        total_length = ctypes.sizeof(MsgHeader) + payload_length
        rx_data = self._sio.read(total_length)

        if self._debug:
            print('RX: len: %d data: %s'
                  % (len(rx_data), ' '.join('%02X' % x for x in rx_data)))

        if timeout is not None:
            # Change it back to original setting
            self._sio.timeout = self._timeout

        if rx_data is None or len(rx_data) < 2:
            raise RuntimeError('No data received')

        # Check sync bytes
        if rx_data[0] != MSG_SYNC_VALUE_0 or rx_data[1] != MSG_SYNC_VALUE_1:
            raise RuntimeError('Sync bytes invalid: %s'
                               % ' '.join('%02X' % x for x in rx_data[:2]))

        if len(rx_data) < ctypes.sizeof(MsgHeader):
            raise RuntimeError('Received data is too short (%d bytes)'
                               % len(rx_data))

        # Decode the message
        rx_header = MsgHeader.from_buffer_copy(rx_data)
        needed_length = ctypes.sizeof(MsgHeader) + rx_header.length
        if len(rx_data) < needed_length:
            raise RuntimeError('Received data is too short: %d bytes.'
                               'Expected from header: %d bytes'
                               % (len(rx_data), needed_length))

        # Calculate and check CRC
        crc = self._crc_alg.bit_by_bit_fast(rx_data[4:needed_length])

        if crc != rx_header.crc:
            raise RuntimeError('CRC check failed. Header: 0x%04X '
                               'Calculated: 0x%04X' % (rx_header.crc, crc))

        msg_data = rx_data[ctypes.sizeof(MsgHeader):]
        # Check if it is an ACK message with an error code:
        if rx_header.msg_type == MsgType.ACK.value and msg_data[1] != 0:
            raise RuntimeError('Received error code %d' % msg_data[1])

        # Check address
        if address is None:
            address = self._default_address
        if rx_header.address != address:
            raise RuntimeError('Incorrect address received: %d' % address)

        # Check message type
        if rx_header.msg_type != msg_type.value:
            raise RuntimeError('Incorrect message type received.'
                               'Expected %d received %d'
                               % (msg_type.value, rx_header.msg_type))

        if len(msg_data) < payload_length:
            raise RuntimeError('Payload data too short. '
                               'Expected: %d bytes. Received %d bytes'
                               % (payload_length, len(msg_data)))

        return msg_data

    def _check_ack(self, address, msg_type):
        """Check for ACK message and error code"""
        msg_data = self._receive_message(address, MsgType.ACK,
                                         ctypes.sizeof(MsgDataAck))
        ack = MsgDataAck.from_buffer_copy(msg_data)
        if ack.msg_type != msg_type.value:
            raise RuntimeError('ACK for different msg_type received.'
                               'Expected %d received %d'
                               % (msg_type.value, ack.msg_type))

        if ack.error_code != 0:
            raise RuntimeError('Received error code %d' % (ack.error_code))

    def set_frequency(self, frequency, address=None):
        """Set the operating frequency

        Parameters
        ----------
        frequency : int
            Frequency in MHz.
        address : int or None
            Address of the TBS Fusion. If None, the default_address is used.
        """
        freq_data = MsgDataSetFreq()
        freq_data.frequency = frequency

        self._send_message(address, MsgType.COMMAND_SET_FREQUENCY, freq_data)
        self._check_ack(address, MsgType.COMMAND_SET_FREQUENCY)

    def get_frequency_rssi(self, address=None):
        """Get the current operating frequency and RSSI

        Parameters
        ----------
        address : int or None
            Address of the TBS Fusion. If None, the default_address is used.

        Returns
        -------
        frequency : int
            Operating frequency in MHz.
        RSSI A: float.
            Received signal strength indicator for receiver A.
            Scaled to 0..1.
        RSSI B: float.
            Received signal strength indicator for receiver B.
            Scaled to 0..1.
        """
        self._send_message(address, MsgType.FREQUENCY_RSSI_REQUEST)

        msg_data = self._receive_message(address,
                                         MsgType.FREQUENCY_RSSI_RESPONSE,
                                         ctypes.sizeof(MsgDataFreqRssi))

        freq_rssi = MsgDataFreqRssi.from_buffer_copy(msg_data)

        # Scale to 0..1 range
        rssi_a = float(freq_rssi.rssi_a) / 255
        rssi_b = float(freq_rssi.rssi_b) / 255

        return freq_rssi.frequency, rssi_a, rssi_b

    def rssi_scan_range(self, freq_start, freq_stop, freq_step, rx_use=0,
                        delay_ms=25, timeout=1.0, address=None):
        """Measure the RSSI for a frequency range

        Parameters
        ----------
        freq_start : int
            Start frequency in MHz.
        freq_stop : int
            Stop frequency in MHz.
        freq_step : int
            Frequency step size in MHz.
        rx_use : int
            Which receiver to use (0: both, 1: A, 2: B).
        delay_ms : int
            Delay in milliseconds between setting frequency and measuring
            RSSI. Needs to be large enough for RSSI to stabilize.
        timeout : float
            Additional timeout to use when waiting for response.
        address : int or None
            Address of the TBS Fusion. If None, the default_address is used.

        Returns
        -------
        frequencies : list (int)
            List of frequencies in MHz at which the RSSI was acquired.
        RSSI : list (float)
            RSSI measurements. Note: When both receivers are used, the
            values are interleaved (A, B, A, B, ..).
        """
        freq_range_scan = MsgDataFreqRangeScan()
        freq_range_scan.freq_start = freq_start
        freq_range_scan.freq_stop = freq_stop
        freq_range_scan.freq_step = freq_step
        freq_range_scan.rx_use = rx_use
        freq_range_scan.delay_ms = delay_ms

        self._send_message(address, MsgType.FREQUENCY_RANGE_SCAN_REQUEST,
                           freq_range_scan)

        # These are the actual frequencies used
        frequencies = list(range(freq_start, freq_stop, freq_step))
        num_freqs = len(frequencies)

        # Approximate time needed
        t_scan = 1e-3 * (delay_ms + 4) * num_freqs
        if rx_use == 0:
            t_scan = t_scan / 2
        time.sleep(t_scan)

        rssi_data = self._receive_message(address,
                                          MsgType.FREQUENCY_SCAN_RESPONSE,
                                          num_freqs, timeout=timeout)

        # Scale to 0..1 range
        rssi = [float(x) / 255 for x in rssi_data]

        return frequencies, rssi

    def rssi_scan_list(self, frequencies, rx_use=0, delay_ms=40,
                       timeout=1.0, address=None):
        """Measure the RSSI for a list of frequencies

        Parameters
        ----------
        frequencies : list (int)
            List of frequencies in MHz at which the RSSI will be acquired.
        rx_use : int
            Which receiver to use (0: both, 1: A, 2: B).
        delay_ms : int
            Delay in milliseconds between setting frequency and measuring
            RSSI. Needs to be large enough for RSSI to stabilize.
        timeout : float
            Additional timeout to use when waiting for response.
        address : int or None
            Address of the TBS Fusion. If None, the default_address is used.

        Returns
        -------
        RSSI : list (float)
            RSSI measurements. Note: When both receivers are used, the
            values are interleaved (A, B, A, B, ..).
        """
        num_freqs = len(frequencies)
        freq_array_type = ctypes.c_uint16 * num_freqs
        freq_array = freq_array_type(*[int(f) for f in frequencies])

        freq_list_scan = MsgDataFreqListScan()
        freq_list_scan.rx_use = rx_use
        freq_list_scan.delay_ms = delay_ms

        msg_data = bytearray(freq_list_scan)
        msg_data.extend(bytearray(freq_array))

        self._send_message(address, MsgType.FREQUENCY_LIST_SCAN_REQUEST,
                           msg_data)

        # Approximate time needed
        t_scan = 1e-3 * (delay_ms + 4) * num_freqs
        if rx_use == 0:
            t_scan = t_scan / 2
        time.sleep(t_scan)

        rssi_data = self._receive_message(address,
                                          MsgType.FREQUENCY_SCAN_RESPONSE,
                                          num_freqs, timeout=timeout)

        # Scale to 0..1 range
        rssi = [float(x) / 255 for x in rssi_data]

        return rssi
