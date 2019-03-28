#define DEBUGPRINT

// There are shorter ways to type 16
#define AES_BLOCK_SIZE 16
// 16 raw bytes of AES KEY
uint8_t aesKey[AES_BLOCK_SIZE] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Sensor UDP API server address and port
const IPAddress apiHost(127,0,0,1);
const unsigned int udpLocalPort = 9999;
const unsigned int apiUDPPort = 9919;
const unsigned int apiTCPPort = 9920;

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
  { 1, true, "Sompasauna.fi", "xyz123", 6, { 0x6C, 0x8B, 0x2F, 0x5D, 0x19, 0xF7 } }, // the ssid, channel and bssid are correct, password is not.
  { 2, false,  "Wifi McWifiFace 2", "password", NULL, { 0xC0, 0xEE, 0xFB, 0xFA, 0xFF, 0x01 } },
  { 3, false, "Helsingin kaupungin WLAN", NULL, 11, { 0x00, 0x87, 0x31, 0xD7, 0xFE, 0x01 } }, // these should be correct
  { 4, false, "Helsinki City Open WLAN", NULL, 11, { 0x00, 0x87, 0x31, 0xD7, 0xFE, 0x02 } },
  { 5, false, "Helsingfors stads WLAN", NULL, 11, { 0x00, 0x87, 0x31, 0xD7, 0xFE, 0x03 } }
};
