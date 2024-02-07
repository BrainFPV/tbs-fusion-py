"""Example how to use interface with a TBS Fusion"""

import time

from tbsfusion import TBSFusion

# Create the interface
fusion = TBSFusion('/dev/ttyUSB0',    # Serial port to use
                   baudrate=115200,   # Baudrate ("Serial Baud" in settings)
                   default_address=1, # Address ("Serial Addr" in settings)
                   discard_echo=True, # Set to False for RS-485
                   )

# Set the operating frequency
fusion.set_frequency(5800)

# Wait for RSSI to stabilize
time.sleep(0.1)

# Get the frequency and print the RSSI
freq_received, rssi_a, rssi_b = fusion.get_frequency_rssi()
print('Frequency: %d MHz RSSI A: %0.2f RSSI B: %0.2f'
        % (freq_received, rssi_a, rssi_b))

# Perform an RSSI scan in the "F" band
freqs, rssi = fusion.rssi_scan_range(5740, 5900, 20)
print(' '.join(['%d MHz: %0.2f' % (f, r) for f, r in zip(freqs, rssi)]))

# Do the same scan but using the frequency list, use only receiver B
rssi = fusion.rssi_scan_list(freqs, rx_use=2)
print(' '.join(['%d MHz: %0.2f' % (f, r) for f, r in zip(freqs, rssi)]))
