#define DEBUGPRINT

// Sensor UDP API server address and port
const IPAddress udpHost(127,0,0,1);
const int udpPort(1234);

// Local UDP response port
const int udpLocalPort = 4567;

// Wifi AP's
const char* wifi_SSID = "Wifi AP SSID"; // Default Wifi AP SSID
const char* wifi_PWD = "wifipassword";   // Defult Wifi AP password
const char* wifi_SSID_ALT = "Wifi AP2 SSID"; // Secondary Wifi AP SSID
const char* wifi_PWD_ALT = "wifipassword2";   // Secondary Wifi AP password
const char* wifi_SSID_ALT_2 = "Helsingin kaupungin WLAN"; // Tertiary passwordless open wifi

// Using a static IP avoids DHCP and speeds up connection. Only used on primary wifi AP.
const IPAddress localIp(10,0,0,200);
const IPAddress gatewayIp(10,0,0,1); 
const IPAddress subnetIp(255,255,255,0);
const IPAddress dnsIp(127,0,0,1);

struct wifiAp {
  const int id;
  const bool static_ip;
  const char *ssid;
  const char *password;
  const int channel;
  const byte bssid[6];
};

const struct wifiAp wifiApList[5] = {
  { 1, true,  "Wifi McWifiFace", "password", 6, { 0xC0, 0xEE, 0xFB, 0xFA, 0xFF, 0x00 } },
  { 2, true,  "Wifi McWifiFace 2", "password", 6, { 0xC0, 0xEE, 0xFB, 0xFA, 0xFF, 0x01 } },
  { 3, false, "Helsingin kaupungin WLAN", NULL, 11, { 0x00, 0x87, 0x31, 0xD7, 0xFE, 0x01 } },
  { 4, false, "Helsinki City Open WLAN", NULL, 11, { 0x00, 0x87, 0x31, 0xD7, 0xFE, 0x02 } },
  { 5, false, "Helsingfors stads WLAN", NULL, 11, { 0x00, 0x87, 0x31, 0xD7, 0xFE, 0x03 } }
};