# Configuration INI

CANLocking requires a configuration file on an SD card, which is formatted as a FAT or FAT32 file system.

This configuration file, allows for configuration of your WiFi credentials, along with options as to if the Grid Connect server is able, and if so on which port, and the similar for the Engine Driver throttle service.

The file must be named 'config.ini' and needs to be loaded in the root directory of the SD card.  Below is a template example for the config.ini contents.  You need to change (at a minimum) the ssid and password values:

    ; CANLocking WiFi configuration
    ; Example file:
    ; Modify and write to a FAT formatted SD Card

    [wifi]
    country = GB             ; Country code for WiFi
    ssid = XXXXXXXXX         ; WiFi router SSID
    password = XXXXXXXXXXXX  ; WiFi router password
    wpa_auth = false         ; Enable WiFi WPA authentication [WPA TPIP PSK]
    wpa2_auth = true         ; Enable WiFi WPA2 authentication [WP2 AES PSK]

    [gridconnect]
    enable = true            ; Enable CBUS Grid Connect server
    port = 5555              ; Port number for the Grid Connnect server

    [edthrottle]
    enable = true            ; Enable the Engine Driver Throttle server
    port = 4444              ; Port number for the ED Throttle server

## WiFi configuration

CANLocking currently only supports WiFi in station / router mode, i.e. the CANLocking module connects to a WiFi router, as such the CANLocking needs knowledge of the credentials to connect to the router.

The default config.ini, as above, is setup to authenticate with the router using WPA2, however this can be changed.

### Open mode

To connect to a router in open mode, aka no authentication, then set both wpa_auth and wpa2_auth false, i.e.:

      [wifi]
      country = GB             ; Country code for WiFi
      ssid = XXXXXXXXX         ; WiFi router SSID
      password = XXXXXXXXXXXX  ; WiFi router password
      wpa_auth = false         ; Enable WiFi WPA authentication [WPA TPIP PSK]
      wpa2_auth = false        ; Enable WiFi WPA2 authentication [WP2 AES PSK]

### WPA mode

To use the legacy WPA authentication method, set wpa_auth true and wpa2_auth false, i.e.:

      [wifi]
      country = GB             ; Country code for WiFi
      ssid = XXXXXXXXX         ; WiFi router SSID
      password = XXXXXXXXXXXX  ; WiFi router password
      wpa_auth = true          ; Enable WiFi WPA authentication [WPA TPIP PSK]
      wpa2_auth = false         ; Enable WiFi WPA2 authentication [WP2 AES PSK]

### WPA/WPA2 Mixed mode

To use mixed WPA/WPA2 authentication, set both wpa_auth and wpa2_auth true, i.e.:

      [wifi]
      country = GB             ; Country code for WiFi
      ssid = XXXXXXXXX         ; WiFi router SSID
      password = XXXXXXXXXXXX  ; WiFi router password
      wpa_auth = true          ; Enable WiFi WPA authentication [WPA TPIP PSK]
      wpa2_auth = true         ; Enable WiFi WPA2 authentication [WP2 AES PSK]

## WiFi Country configuration

The CANLocking WiFi configuration is specified by a two character country code in the configuration INI.  Valid country codes are:

| Country           |Code|
|-------------------|----|
| AUSTRALIA         | AU |
| AUSTRIA           | AT |
| BELGIUM           | BE |
| BRAZIL            | BR |
| CANADA            | CA |
| CHILE             | CL |
| CHINA             | CN |
| COLOMBIA          | CO |
| CZECH_REPUBLIC    | CZ |
| DENMARK           | DK |
| ESTONIA           | EE |
| FINLAND           | FI |
| FRANCE            | FR |
| GERMANY           | DE |
| GREECE            | GR |
| HONG_KONG         | HK |
| HUNGARY           | HU |
| ICELAND           | IS |
| INDIA             | IN |
| ISRAEL            | IL |
| ITALY             | IT |
| JAPAN             | JP |
| KENYA             | KE |
| LATVIA            | LV |
| LIECHTENSTEIN     | LI |
| LITHUANIA         | LT |
| LUXEMBOURG        | LU |
| MALAYSIA          | MY |
| MALTA             | MT |
| MEXICO            | MX |
| NETHERLANDS       | NL |
| NEW_ZEALAND       | NZ |
| NIGERIA           | NG |
| NORWAY            | NO |
| PERU              | PE |
| PHILIPPINES       | PH |
| POLAND            | PL |
| PORTUGAL          | PT |
| SINGAPORE         | SG |
| SLOVAKIA          | SK |
| SLOVENIA          | SI |
| SOUTH_AFRICA      | ZA |
| SOUTH_KOREA       | KR |
| SPAIN             | ES |
| SWEDEN            | SE |
| SWITZERLAND       | CH |
| TAIWAN            | TW |
| THAILAND          | TH |
| TURKEY            | TR |
| UK                | GB |
| USA               | US |

\note
For Worldwide compatiblity, the country code can be specified as XX, this means the WiFi will operate only on channels 12-14.

\attention
The CANLocking will indicate the status of the WiFi configuration on the PICO-W's LED.  If the LED is steady ON the CANLocking is connected to your WiFi router.  if the LED is flashing, then CANLocking could not connect to your router.

\warning
Rebooting of the CANLocking may result in the in ability to connect to the WiFi router, this appears to occur because the routers DHCP server rejects the connection as it believes the CANLocking is already connected.  If you cannot connect to WiFi after rebooting the CANLocking, reset the CANLocking again, after which you should be able to connect once more.

<div class="section_buttons">
 
|                       Next |
|---------------------------:|
| [Web Server](webserver.md) |
 
</div>
