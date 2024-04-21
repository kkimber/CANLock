# CBUS&reg; CANLock Module

This repository contains firmware for the Raspberry PICO-W to build a module with mechanical lever frame locking and a WiFi interface to CBUS.

The default pin mapping used by CANLock is follows.

| Pico Pin | Function      |/| Pico Pin | Function      |
|----------|---------------|-|----------|---------------|
| 1        | GP0           | | 40       | VBUS          |
| 2        | GP0           | | 39       | VSYS          |
| 3        | GND           | | 38       | GND           |
| 4        | GP2           | | 37       | 3V3_EN        |
| 5        | GP3           | | 36       | 3V3           |
| 6        | GP4           | | 35       | ADC_VREF      |
| 7        | GP5           | | 34       | GP28          |
| 8        | GND           | | 33       | GND           |
| 9        | GP6           | | 32       | GP27          |
| 10       | GP7           | | 31       | GP26          |
| 11       | Red LED       | | 30       | RUN           |
| 12       | Green LED     | | 29       | GP22          |
| 13       | GND           | | 28       | GND           |
| 14       | GP10          | | 27       | GP21          |
| 15       | GP11          | | 26       | GP20          |
| 16       | GP12          | | 25       | GP19 - MOSI   |
| 17       | CAN Tx        | | 24       | GP18 - SCLK   |
| 18       | GND           | | 23       | GND           |
| 19       | CAN Rx        | | 22       | GP17 - CS     |
| 20       | Yellow LED    | | 21       | GP16 - MISO   |

An SD Card is required for the WiFi configuration file and credentials, a "FAT" formatted SD card should be connected to the CANLock module on the SPI interface as identified above (MOSI/MISO/SCLK/CS).

An example configuration file can be found in the src/CANLock folder of this repository.  The example file needs modifying to provide the correct WiFi SSID and password for connnecting to a WiFi router.

CANLock uses the soft PIO based CAN2040 CAN controller, so no external CAN controller is required, however a CAN2562 transceiver or similar MUST be connected to the Pico-W in order to communicate on CAN.

\attention CBUS&reg; is a registered trademark of Dr. Michael Bolton.  See [CBUS](https://cbus-traincontrol.com/)
