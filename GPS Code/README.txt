Author: Evan Anderson
Latest Edit: 5/11/2018

The purpose of this document is to describe the functionality of each of the files
in this repository. Each folder in the main directory is self-contained and is intended
to be run independently.

===========================================

//GPS > SD CARD (DEBUG VERSION)

evan_uart_gps_alldata_debug.ino
This program receives data from the GPS module and stores it into local SD card storage (Serial3).
The data includes Ross Hoffman's callsign, packet identifier (default to "BIGGIE", for the
booster), and the data along with the protocol. The purpose of the "debug" is because
the program also outputs to Serial for checking. This function is removed in the final
implementation to reduce clock cycles.

===========================================

//GPS > SD CARD + RADIO LINK (DEBUG VERSION)

evan_uart_gps_full_op.ino
This program is a direct copy of "evan_uart_gps_alldata.ino", but includes the functionality
of writing out to both the SD card on Serial3 and the TXRX link on Serial2. It still includes
the debug code writing out to Serial as well. This code includes all final functionality but
is NOT intended to be flight ready due to the debug lines left in the code.

===========================================

//SD CARD > RADIO LINK (DEBUG VERSION)

txCode.ino
This program takes in data from "data.txt" stored on the SD card (Serial3) and outputs it
to the TXRX module (Serial2) and Serial output. It does NOT gather data in any capacity,
and must be suppplied by a data file called "data.txt" explicitly.

===========================================

//TXRX FREQUENCY CHANGER (UNFINISHED)

TXRX_freq.ino
This program is unfinished as of the writing of this README, but is intended to change the
frequency of the TXRX module (Serial2) on the main board. This code is unfinished and it
is uncertain at this time whether it works as intended. However, the logic is intact.

===========================================

//THIS IS FLIGHT-READY FIRMWARE

firmware.ino
This program is the most recent version of firmware intended for flight onboard the rocket.
All serial prints are removed, so no clock cycles are wasted. The program includes LED
functionality to verify that the board is powered on and transmitting.
Some error checking (after initilization is finished) includes:
- If the green light is solid and the red light is flashing, the board is operating nominally.
- If the green light is solid and the red light is solid, the software has crashed while gathering data.
- If the green light is solid and the red light is off, the software has crashed attempting to write to the SD card or transmit.
- If both lights are off after initialization, there has been a power failure.
- If neither light turns on, initialization has failed.
- If the red light is on or flashing, but the green light is off, the green LED is blown, OR initialization failed but for some reason still entered the main loop. This should theoretically never happen.
This file will output a "data.txt" file to the SD card. The SD card can be removed anytime without damage to the file. However
the card must be in the slot upon powering on the system.

===========================================

Other files:
I have included several test files with good GPS data.
- A 3160 line test file of pure GPS data output (5 minute test)
- A shorter test file of pure GPS output (good for quick sanity checks)
- A file including callsign and packet identifiers in addition to GPS data.