/*
   CBUS Module Library - RasberryPi Pico SDK port
   Copyright (c) Kevin Kimber 2023

   Based on work by Duncan Greenwood
   Copyright (C) Duncan Greenwood 2017 (duncan_greenwood@hotmail.com)

   This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                  and indicate if changes were made. You may do so in any reasonable manner,
                  but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                 your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                 legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

*/

#include "CBUSWiFi.h"

#include <pico/stdlib.h>
#include <pico/cyw43_arch.h>
#include "lwip/apps/httpd.h"

// FatFs support
#include "f_util.h"
#include "ff.h"
#include "hw_config.h"

/// Helper for INI parsing
#define MATCH(s, n) strcmp(section, s) == 0 && strcmp(name, n) == 0

// @todo make export
extern int8_t leverStates[];

/// @brief Parser for config.ini
static int handler(void *user, const char *section, const char *name, const char *value)
{
   config_t *pconfig = (config_t *)user;

   // Configuration INI section headers
   constexpr const char *wifiSection = "wifi";
   constexpr const char *gcSection = "gridconnect";

   if (MATCH(wifiSection, "country"))
   {
      pconfig->country = strdup(value);
   }
   else if (MATCH(wifiSection, "ssid"))
   {
      pconfig->ssid = strdup(value);
   }
   else if (MATCH(wifiSection, "password"))
   {
      pconfig->passwd = strdup(value);
   }
   else if (MATCH(wifiSection, "wpa_auth"))
   {
      pconfig->wpaAuth = strcasecmp(value, "true") == 0 ? true : false;
   }
   else if (MATCH(wifiSection, "wpa2_auth"))
   {
      pconfig->wpa2Auth = strcasecmp(value, "true") == 0 ? true : false;
   }
   else if (MATCH(gcSection, "enable"))
   {
      pconfig->gcEnable = strcasecmp(value, "true") == 0 ? true : false;
   }
   else if (MATCH(gcSection, "port"))
   {
      pconfig->gcPort = atoi(value);
   }
   else
   {
      return 0; /* unknown section/name, error */
   }

   return 1;
}

////////////////////////////////
///// TEST WEB SERVER CODE

// SSI tags - tag length limited to 8 bytes by default
const char *ssi_tags[] = {
   "lever1",     // 0
   "lever2",     // 1
   "lever3",     // 2
   "lever4",     // 3
   "lever5",     // 4
   "lever6",     // 5
   "lever7",     // 6
   "lever8",     // 7
   "lever9",     // 8
   "lever10",    // 9
   "lever11",    // 10
   "lever12",    // 11
   "lever13",    // 12
   "lever14",    // 13
   "lever15",    // 14
   "lever16",    // 15
   "lever17",    // 16
   "lever18",    // 17
   "lever19",    // 18
   "lever20",    // 19
   "msg",        // 20
   "gcenable",   // 21
   "gcport",     // 22
   "country",    // 23
   "authmode"    // 24
};

u16_t ssi_handler(int iIndex, char *pcInsert, int iInsertLen)
{
   size_t printed;
   switch (iIndex)
   {
   case 0: // Lever States
   case 1:
   case 2:
   case 3:
   case 4:
   case 5:
   case 6:
   case 7:
   case 8:
   case 9:
   case 10:
   case 11:
   case 12:
   case 13:
   case 14:
   case 15:
   case 16:
   case 17:
   case 18:
   case 19:
   {
      printed = snprintf(pcInsert, iInsertLen, "%s", leverStates[iIndex] == -1 ? "0" : "1");
   }
   break;
   case 20: // Msg
   {
      printed = snprintf(pcInsert, iInsertLen, "%s", "Interlocking State Valid");
   }
   break;
   case 21: // Grid Connect server enabled
   {
      if (CBUSWiFi::isGridConnectEnabled())
      {
         printed = snprintf(pcInsert, iInsertLen, "%s", "Yes");
      }  
      else
      {
         printed = snprintf(pcInsert, iInsertLen, "%s", "No");
      }
   }
   break;
   case 22: // Grid Connect port
   {
      printed = snprintf(pcInsert, iInsertLen, "%d", CBUSWiFi::getGridConnectPort());
   }
   break;
   case 23: // WiFi country code
   {
      printed = snprintf(pcInsert, iInsertLen, "%s", CBUSWiFi::getCountryCode());
   }
   break;
   case 24: // WiFi authentication mode
   {
      if (CBUSWiFi::getWPAEnable() && !CBUSWiFi::getWPA2Enable())
      {
         // WPA Authentication
         printed = snprintf(pcInsert, iInsertLen, "%s", "WPA");
      }
      else if (!CBUSWiFi::getWPAEnable() && CBUSWiFi::getWPA2Enable())
      {
         // WPA2 Authentication
         printed = snprintf(pcInsert, iInsertLen, "%s", "WPA2");
      }
      else if (CBUSWiFi::getWPAEnable() && CBUSWiFi::getWPA2Enable())
      {
         // WPA/WPA2 Mixed Authentication
         printed = snprintf(pcInsert, iInsertLen, "%s", "WPA/WPA2");
      }
      else
      {
         // Open / No Authentication
         printed = snprintf(pcInsert, iInsertLen, "%s", "None");
      }
   }
   break;
   default:
      printed = 0;
      break;
   }

   return static_cast<u16_t>(printed);
}

// Initialise the SSI handler
void ssi_init(void)
{
   http_set_ssi_handler(ssi_handler, ssi_tags, LWIP_ARRAYSIZE(ssi_tags));
}

///// TEST WEB SERVER CODE
////////////////////////////////


///
/// A class to manage WiFi to CBUS connectivity
///
CBUSWiFi::CBUSWiFi()
{
}

///
/// @brief Reads a configuration file "config.ini" on the SD card
/// 
/// @return true if the SD card could be mounted and the configuration file parsed
/// @return false the SD card could not be mounted, or error parsing the configuration file
///
bool CBUSWiFi::ReadConfiguration()
{
   // Setup SPI based on configuration in hw_config.c
   sd_card_t *pSD = sd_get_by_num(0);

   // Mount the SD card as a FAT filesystem
   if (f_mount(&pSD->fatfs, pSD->pcName, 1) != FR_OK)
   {
      // Could not mount the file system
      return false;
   }

   // Parse the configuration file into our configuration
   int result = parseINI("config.ini", handler, &m_config);

   // Unmount SD card
   f_unmount(pSD->pcName);

   return result == 0;
}

///
/// @brief Initialize client connection to WiFi router
///
/// @return true Successfully connected
/// @return false Failed to connect to the WiFi router
bool CBUSWiFi::InitializeClient()
{
   // Read the configuration INI
   if (!ReadConfiguration())
   {
      // Could not setup SPI, mount the SD card or parse the configuration
      // Initialize cyw43 so we can flash the onboard LED
      if (cyw43_arch_init())
      {
         // WiFi Init failed - hang here
         while(1) {};
      }
      return false;
   }

   // Set the country code - special case for UK
   uint32_t country = CYW43_COUNTRY_WORLDWIDE;
   if (strcasecmp(m_config.country, "UK") == 0)
   {
      country = CYW43_COUNTRY_UK;
   }
   else
   {
      // Check for supplied string length, should be two char country code
      // take first two chars if more are specified
      if (strlen(m_config.country) >= 2)
      {
         country = CYW43_COUNTRY(toupper(m_config.country[0]), toupper(m_config.country[1]), 0);
      }
   }
   
   // Attempt to initialize with the specified country code
   if (cyw43_arch_init_with_country(country))
   {
      return false;
   }

   // We're operating in station mode
   cyw43_arch_enable_sta_mode();

   // Set power saving options for performance
   cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM);

   // Determine required authentication method from configuration
   // Default is no authentication
   uint32_t auth = CYW43_AUTH_OPEN;

   if (m_config.wpaAuth && !m_config.wpa2Auth)
   {
      // WPA Authentication
      auth = CYW43_AUTH_WPA_TKIP_PSK;
   }
   else if (!m_config.wpaAuth && m_config.wpa2Auth)
   {
      // WPA2 Authentication
      auth = CYW43_AUTH_WPA2_AES_PSK;
   }
   else if (m_config.wpaAuth && m_config.wpa2Auth)
   {
      // WPA/WPA2 Mixed Authentication
      auth = CYW43_AUTH_WPA2_MIXED_PSK;
   }

   // Attempt connection to the WiFi router - 5 second timeout
   int err = cyw43_arch_wifi_connect_timeout_ms(m_config.ssid, m_config.passwd, auth, 5000);

   // Allow for retries
   int nRetries = 4;
   while ((err != PICO_OK) && nRetries--)
   {
      // Retry - again 5 second timeout
      err = cyw43_arch_wifi_connect_timeout_ms(m_config.ssid, m_config.passwd, auth, 5000);

      // Break out on success
      if (err == PICO_OK)
      {
         break;
      }

      // Delay before retry
      sleep_ms(1000);
   }

   // Check for connection
   if (err != PICO_OK)
   {
      // Failed to connect
      return false;
   }

   // WiFi connected
   return true;
}

///
/// @brief Initialize WiFi as aa local hotspot access point
///
/// @return true
/// @return false
///
bool CBUSWiFi::InitializeAP()
{
   /// @todo AP setup
   return true;
}

///
/// @brief Initialize a web server to manage configuration
///
/// @return true if the web server was successfully initiaized
/// @return false if the web server was not initialized
///
bool CBUSWiFi::InitWebServer(void)
{
   // Initialise web server
   httpd_init();

   // Configure SSI and CGI handler
   ssi_init();

   return true;
}

int CBUSWiFi::parseINI(const char *filename, ini_handler handler, void *user)
{
   FRESULT result;
   FIL file;
   int error;

   result = f_open(&file, filename, FA_OPEN_EXISTING | FA_READ);

   if (result != FR_OK)
   {
      return -1;
   }

   error = ini_parse_stream((ini_reader)f_gets, &file, handler, user);

   f_close(&file);

   return error;
}
